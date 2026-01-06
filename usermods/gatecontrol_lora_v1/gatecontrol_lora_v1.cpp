#include "gatecontrol_lora_v1.h"
//#include <WiFi.h>  // fallback for WiFi.macAddress()

static LoraLink::Core ll{};
static LoraLink::Callbacks cb{};

// --- Last RX capture (for Info UI) ---
static uint8_t lastRxRaw[64];
static volatile uint8_t lastRxLen = 0;        // volatile: wird in anderem Kontext geschrieben
static char lastRxHex[(64 * 3) + 1];          // "FF " * 64 + '\0'
//static uint32_t lastRxSeenMs = 0;

static uint16_t debugCounter = 0;

static const char HEXLUT[] = "0123456789ABCDEF";

static inline void captureLastRxPacket(const uint8_t* buf, size_t len) {
  if (!buf) { lastRxLen = 0; lastRxHex[0] = '\0'; return; }

  size_t n = (len > sizeof(lastRxRaw)) ? sizeof(lastRxRaw) : len;

  // Rohbytes sichern
  memcpy(lastRxRaw, buf, n);
  lastRxLen = (uint8_t)n;
  //lastRxSeenMs = millis();

  // Ein-Zeilen-Hexstring bauen
  char* p = lastRxHex;
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = lastRxRaw[i];
    *p++ = HEXLUT[b >> 4];
    *p++ = HEXLUT[b & 0x0F];
    *p++ = ' ';
  }
  if (n) *(p - 1) = '\0';  // letztes Leerzeichen durch 0-terminator ersetzen
  else   *p = '\0';
}

template<typename T>
static inline bool um_read(const um_data_t* d, uint8_t idx, um_types_t expected, T& out) {
  if (!d || idx >= d->u_size) return false;
  if (!d->u_data[idx]) return false; // optional: check for null pointer
  if (d->u_type[idx] != expected) return false;
  out = *reinterpret_cast<T*>(d->u_data[idx]);
  return true;
}

// ======= Self-pointer for ISR =======
static UsermodGateControlLoRa* s_gateSelf = nullptr;

// ========= ISR (no args, RadioLib-compatible) =========
/* void IRAM_ATTR UsermodGateControlLoRa::onRxStatic() {
  if (s_gateSelf) s_gateSelf->rxFlag = true;
} */

// ========= Setup =========
void UsermodGateControlLoRa::setup() {
  // init defaults for current gate state
  current.groupId    = 0;
  current.state      = 0;
  current.effect     = 11;
  current.brightness = 128;

  // read MAC from efuse (no WiFi init required)
  // readEfuseMac(); // now in LoraLink::beginCommon()

  // init LoRa
  radioReady = radioInit();
  if (!radioReady) {
    DEBUG_PRINTLN(F("[GateLoRa] Radio init FAILED"));
    return;
  }

  cb.onRxPacket = &UsermodGateControlLoRa::on_rx_node;
  cb.onTxDone   = &UsermodGateControlLoRa::on_tx_done_node;
  cb.ctx        = this; // sehr wichtig: für handlePacket

  DEBUG_PRINTLN(F("[GateLoRa] Radio init OK"));
}

// ========= Loop =========
void UsermodGateControlLoRa::loop() {
  if (!radio) return;
  
  // ersetzt bisheriges Flag-/ISR-/onRx()-Handling
  LoraLink::service(ll, cb);

  if (!batteryUM) {
    // Späterer Retry, bis der Battery-UM seine Daten anbietet
    if (UsermodManager::getUMData(&batteryUM, USERMOD_ID_BATTERY)) {
      DEBUG_PRINTLN(F("[GateLoRa] Battery UM data acquired"));
    }
  }
}

// ========= Info (UI) =========
void UsermodGateControlLoRa::addToJsonInfo(JsonObject& root) {
  // "u" = Objekt; jede Zeile ist ein Array [labelValue1, labelValue2, ...]
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");

  // LoRa Init
  {
    char initMsg[32];
    snprintf(initMsg, sizeof(initMsg), "%s (code %d)", radioReady ? "OK" : "FAIL", (int)radioInitCode);
    JsonArray row = user.createNestedArray(F("LoRa Init"));
    row.add(initMsg);
  }

  // MyID 3B
  {
    char my3[12];
    snprintf(my3, sizeof(my3), "%02X:%02X:%02X", ll.myLast3[0], ll.myLast3[1], ll.myLast3[2]);
    JsonArray row = user.createNestedArray(F("LoRa MyID (3B)"));
    row.add(my3);
  }

  // RX counters
  {
    JsonArray row = user.createNestedArray(F("RX total"));
    row.add(String((unsigned long)ll.rxCountTotal));

    JsonArray row2 = user.createNestedArray(F("RX accepted"));
    row2.add(String((unsigned long)rxAccepted));

    JsonArray row3 = user.createNestedArray(F("TX total"));
    row3.add(String((unsigned long)ll.txCount));
  }

  // Last RSSI/SNR
  {
    char sig[24];
    snprintf(sig, sizeof(sig), "%d / %d", (int)ll.lastRssi, (int)ll.lastSnr);
    JsonArray row = user.createNestedArray(F("Last RSSI/SNR"));
    row.add(sig);
  }

  {
    JsonArray row = user.createNestedArray(F("Last RX"));
    if (lastRxLen == 0) {
      row.add(F("(none)"));
    } else {
      char meta[32];
      uint32_t age = (millis() - ll.lastRxAtMs) / 1000;
      snprintf(meta, sizeof(meta), "%uB (%lus ago)", lastRxLen, (unsigned long)age);
      row.add(meta);

  
      JsonArray rowHex = user.createNestedArray(F("Last RX Hex"));
      if (lastRxLen == 0) rowHex.add(F("(none)"));
      else                rowHex.add(lastRxHex);
    }
  }

  {
    char debug[16];
    //snprintf(debug, sizeof(debug), "%d", (int)debugCounter);
    snprintf(debug, sizeof(debug), "%d", (int)ll.debug);
    //snprintf(debug, sizeof(debug), "%d", (int)ll.toaUsMax17/1000U);
    JsonArray row = user.createNestedArray(F("Debug"));
    row.add(debug);
  }
  // Master 3B
  {
    JsonArray row = user.createNestedArray(F("Master (3B)"));
    if (masterKnown) {
      char m3[12];
      snprintf(m3, sizeof(m3), "%02X:%02X:%02X", masterLast3[0], masterLast3[1], masterLast3[2]);
      row.add(m3);
    } else {
      row.add(F("(none)"));
    }
  }

  // Master 6B not in use
/*   {
    JsonArray row = user.createNestedArray(F("Master (6B)"));
    if (masterFull6Known) {
      char m6[20];
      snprintf(m6, sizeof(m6), "%02X:%02X:%02X:%02X:%02X:%02X",
        masterFull6[0], masterFull6[1], masterFull6[2],
        masterFull6[3], masterFull6[4], masterFull6[5]);
      row.add(m6);
    } else {
      row.add(F("(unknown)"));
    }
  } */

  // Group
  {
    JsonArray row = user.createNestedArray(F("Group"));
    row.add(String(current.groupId));
  }
}

// ========= Config =========
void UsermodGateControlLoRa::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject("GateLoRa");
  top["groupId"] = current.groupId;
  top["macFilterEnabled"] = macFilterEnabled;  // default ON
  top["macFilterPersist"] = macFilterPersist;  // default OFF

  // masterLast3: nur persistieren, wenn Persistenz aktiv
  char m3[7+1];
  sprintf(m3, "%02X%02X%02X", masterLast3[0], masterLast3[1], masterLast3[2]);
  top["masterLast3"] = (macFilterPersist && masterKnown) ? String(m3) : String("000000");

  // masterFullMac: nur persistieren, wenn Persistenz aktiv
  char m6[12+1];
  sprintf(m6, "%02X%02X%02X%02X%02X%02X",
          masterFull6[0], masterFull6[1], masterFull6[2],
          masterFull6[3], masterFull6[4], masterFull6[5]);
  top["masterFullMac"] = (macFilterPersist && masterFull6Known) ? String(m6) : String("");

  // radio defaults
  JsonObject l = top.createNestedObject("lora");
  l["freq"] = LORA_FREQ_HZ;
  l["sf"]   = LORA_SF;
  l["bw"]   = (int)LORA_BW_KHZ;
  l["cr"]   = LORA_CR;
  l["sync"] = LORA_SYNC_WORD;
  l["txp"]  = LORA_TX_POWER;
}

bool UsermodGateControlLoRa::readFromConfig(JsonObject& root) {
  JsonObject top = root["GateLoRa"];
  if (top.isNull()) return false;

  // erst Flags lesen
  getJsonValue(top["groupId"], current.groupId, 0);
  getJsonValue(top["macFilterEnabled"], macFilterEnabled, true);
  getJsonValue(top["macFilterPersist"], macFilterPersist, false);

  // Master nur aus Config übernehmen, wenn Persistenz aktiv
  if (macFilterPersist) {
    String s3 = top["masterLast3"] | "000000";
    if (s3.length() == 6) {
      uint32_t val = strtoul(s3.c_str(), nullptr, 16);
      masterLast3[0] = (val >> 16) & 0xFF;
      masterLast3[1] = (val >> 8)  & 0xFF;
      masterLast3[2] = (val)       & 0xFF;
      masterKnown = (val != 0);
    }

    String s6 = top["masterFullMac"] | "";
    if (s6.length() == 12) {
      for (int i=0;i<6;i++) {
        masterFull6[i] = strtoul(s6.substring(2*i, 2*i+2).c_str(), nullptr, 16);
      }
      masterFull6Known = true;
    }
  }
  // Wenn Persistenz aus: Runtime-Werte NICHT überschreiben

  return true;
}

void UsermodGateControlLoRa::onStateChange(uint8_t mode) {
  // TODO: test! currently status does not send current bri/effect/state -> change this! -> send FW and maybe other params in identify reply instead
  current.effect     = effectCurrent;
  current.brightness = bri;
  current.state      = (bri > 0) ? 1 : 0;
}

// ========= Radio =========
bool UsermodGateControlLoRa::radioInit() {
  // SPI
  spi->begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_NSS);

  // RadioLib Module(cs, dio1, rst, busy, spi)
  static SX1262 r(new Module(LORA_PIN_NSS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY, *spi));
  radio = &r;

  LoraLink::PhyCfg phy;
  phy.freqMHz   = (float)(LORA_FREQ_HZ/1e6f);
  phy.bwKHz     = LORA_BW_KHZ;
  phy.sf        = LORA_SF;
  phy.crDen     = LORA_CR;
  phy.syncWord  = LORA_SYNC_WORD;
  phy.preamble  = LORA_PREAMBLE;
  phy.crcOn     = true;

  // C3/HT-CT62-spezifisch:
  phy.txPowerDbm   = LORA_TX_POWER;        // Default überschreiben
  phy.dio2RfSwitch = 1;                    // SX1262 (HT-CT62) oder auch LLCC68 (DreamLNK)
  phy.rxBoost      = -1;                   // oder 1/0 je nach Boardtests

  radioInitCode = LoraLink::beginCommon(*radio, ll, phy) ? RADIOLIB_ERR_NONE : -999;
  if (radioInitCode != RADIOLIB_ERR_NONE) { radio = nullptr; return false; }

  //ll.radio = radio;

  ll.lbtEnable = true;   // default: false
  
  LoraLink::attachDio1(*radio, ll);

  LoraLink::setDefaultRxContinuous(ll); // *** WICHTIG: Continuous RX über LL aktivieren ***

  return true;
}

bool UsermodGateControlLoRa::senderAllowed(const uint8_t s3[3], uint8_t opcode7) {
  using namespace LoraProto;
  if (!macFilterEnabled) return true;

  if (!masterKnown) {
    // solange kein Master gelernt wurde: nur Discovery/Grouping zulassen
    return (opcode7 == OPC_DEVICES || opcode7 == OPC_SET_GROUP);
  }
  // danach nur noch vom gelernten Master zulassen
  return (s3[0]==masterLast3[0] && s3[1]==masterLast3[1] && s3[2]==masterLast3[2]);
}

void UsermodGateControlLoRa::learnMasterFromSender(const uint8_t s3[3], bool persistIfEnabled) {
  memcpy(masterLast3, s3, 3);
  masterKnown = true;
  if (persistIfEnabled && macFilterPersist) persistMasterIfNeeded();
}

void UsermodGateControlLoRa::persistMasterIfNeeded() {
  // mark config to be saved (serialized via addToConfig)
  requestJSONBufferLock(10);
  releaseJSONBufferLock();
}

void UsermodGateControlLoRa::clearMaster() {
  masterKnown = false;
  memset(masterLast3, 0, 3);
}

void UsermodGateControlLoRa::handlePacket(const uint8_t* buf, size_t len) {
  using namespace LoraProto;

  if (len < sizeof(Header7)) return;

  Header7 h{};
  if (!parseHeader(buf, (uint8_t)len, h)) return;
  debugCounter=1;
  if (!LoraLink::receiverMatches(h.receiver, ll.myLast3)) return;  // broadcast ODER exakt meine 3B
  debugCounter=2;
  if (type_dir(h.type) != DIR_M2N) return;                       // nur Master->Node Requests hier
  debugCounter=3;
  const uint8_t opcode7 = type_base(h.type);

  // MAC-Filter (optional wie bisher)
  if (!senderAllowed(h.sender, opcode7)) return;
  debugCounter=4;

  // Gruppenlogik: für alle Requests, die groupId tragen
  auto groupMatch = [&](uint8_t inGroup) {
    return (inGroup == current.groupId || inGroup == 255);
  };

  bool acted = false;
  const uint8_t bodyLen = (uint8_t)(len - sizeof(Header7));

  debugCounter=5;
  switch (opcode7) {
    case OPC_DEVICES: { // GET_DEVICES
      debugCounter=6;
      P_GetDevices p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      debugCounter=7;
      if (!groupMatch(p.groupId)) break;
      debugCounter=8;

      memcpy(targetForReplyLast3, h.sender, 3);
      sendIdentifyReplyTo(targetForReplyLast3, true /* include full MAC */);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] GET_DEVICES -> schedule IDENTIFY_REPLY"));
    } break;

    case OPC_SET_GROUP: { // SET_GROUP -> apply + ACK
      P_SetGroup p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;

      // wie bisher: group 0 erlaubt Setzen / oder "255" Sonderlogiken…
      current.groupId = p.groupId;

      learnMasterFromSender(h.sender, /*persistIfEnabled*/true);

      // visuelles Feedback
      bri = 128;
      applyPreset(11, CALL_MODE_DIRECT_CHANGE);

      sendAckTo(h.sender, OPC_SET_GROUP, ACK_OK);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] SET_GROUP -> applied + ACK"));
    } break;

    case OPC_WLED_CONTROL: {
      LoraProto::P_WledControl p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;

      applyControl(p);         // fertig
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] WLED_CONTROL -> applied"));
    } break;

    case OPC_STATUS: { // GET_STATUS -> STATUS_REPLY
      P_GetStatus p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;

      sendStatusReplyTo(h.sender);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] GET_STATUS -> STATUS_REPLY"));
    } break;
  }

  if (acted) rxAccepted++;
}

void UsermodGateControlLoRa::sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac) {
  using namespace LoraProto;
  uint8_t out[32];

  P_IdentifyReply p{};
  p.fw = FW_VERSION;
  p.caps            = 0x01; // Bit0 = WLED vorhanden
  p.groupId         = current.groupId;

  if (includeFullMac && ll.macReadOK) {
    for (int i=0;i<6;i++) p.mac6[i] = ll.myMac6[i];
  } else {
    // Falls MAC nicht bekannt, sende 0
    for (int i=0;i<6;i++) p.mac6[i] = 0;
  }

  uint8_t t = make_type(DIR_N2M, OPC_DEVICES); // IDENTIFY_REPLY
  uint8_t n = build(out, ll.myLast3, destLast3, t, p);
  
  LoraLink::scheduleSend(ll, out, n);
}

void UsermodGateControlLoRa::sendAckTo(const uint8_t destLast3[3], uint8_t echoOpcode7, LoraProto::AckStatus st) {
  using namespace LoraProto;
  uint8_t out[32];
  P_Ack p{ echoOpcode7, (uint8_t)st, 0 /*seq*/ };
  
  uint8_t t = make_type(DIR_N2M, OPC_ACK);
  uint8_t n = build(out, ll.myLast3, destLast3, t, p);
  
  LoraLink::scheduleSend(ll, out, n);
}

void UsermodGateControlLoRa::sendStatusReplyTo(const uint8_t destLast3[3]) {
  using namespace LoraProto;
  uint8_t out[32];

  P_StatusReply p{};
  p.state      = current.state;
  p.effect     = current.effect;
  p.brightness = current.brightness;
  p.vbat_mV  = 0;

  //float v = WLED_BatteryVoltage();           // Volt
  //if (!isnan(v)) p.vbat_mV = (int32_t)lroundf(v * 1000.0f);

  if (batteryUM) {
    // Battery usermod: [0] = float voltage (UMT_FLOAT), [1] = byte level (UMT_BYTE)
    //float  voltage = NAN;
    //uint8_t level  = 255; // 0..100 erwartet

    // pointer deref (without um_read)
    float*   vptr = (float*)  batteryUM->u_data[0];
    //uint8_t* lptr = (uint8_t*)batteryUM->u_data[1];

    float voltage = (vptr != nullptr) ? *vptr : NAN;
    //uint8_t level   = (lptr != nullptr) ? *lptr : 255;

    // using um_read helper
    //(void)um_read<float>(batteryUM, 0, UMT_FLOAT, voltage);
    //(void)um_read<uint8_t>(batteryUM, 1, UMT_BYTE,  level);

    if (!isnan(voltage)) {
      if(voltage < 0.0f) voltage = 0.0f;
      p.vbat_mV = (uint16_t)lroundf(voltage * 1000.0f);
    }
  }
  
  p.rssi     = (int8_t)ll.lastRssi;
  p.snr      = (int8_t)ll.lastSnr;

  uint8_t t = make_type(DIR_N2M, OPC_STATUS); // STATUS_REPLY
  uint8_t n = build(out, ll.myLast3, destLast3, t, p);

  LoraLink::scheduleSend(ll, out, n);
  //LoraLink::scheduleSend(ll, out, n, 50, 2500);
}

// ========= Apply CONTROL =========
void UsermodGateControlLoRa::applyControl(const GateCore& in) {
  // state -> on/off via brightness, effect -> preset id, brightness direct
  bri = (in.state > 0) ? in.brightness : 0;
  applyPreset(in.effect);                    // setzt das WLED-Preset
  stateUpdated(CALL_MODE_DIRECT_CHANGE);     // zieht die Helligkeit

  current = in;                              // DIREKTE Übernahme statt Feld-für-Feld
  // mirror current state
  /* 
  current.state      = in.state;
  current.effect     = in.effect;
  current.brightness = in.brightness; 
  */
}

// --- Callback-Brücken als statische Member ---
void UsermodGateControlLoRa::on_rx_node(const uint8_t* pkt, uint8_t len,
                                        int16_t rssi, int8_t snr, void* ctx) {
  auto* self = static_cast<UsermodGateControlLoRa*>(ctx);
  if (!self || !pkt || len == 0) return;

/*   self->lastRssi = rssi;
  self->lastSnr  = snr;
  self->rxTotal++; */

  captureLastRxPacket(pkt, len);
  self->handlePacket(pkt, len);
}

void UsermodGateControlLoRa::on_tx_done_node(void* ctx) {
  auto* self = static_cast<UsermodGateControlLoRa*>(ctx);
  if (!self) return;
  // Optional: Node-spezifische TX-Events
}

// ========= MAC helpers =========
/* void UsermodGateControlLoRa::readEfuseMac() {
  if (LoraLink::readEfuseMac6(myMac6)) {
    macReadOK = true;
    LoraLink::last3FromMac6(myLast3, myMac6);
  } else {
    macReadOK = false;
    memset(myLast3, 0, 3);
  }
} */

// construct & register usermod
static UsermodGateControlLoRa gatecontrol_lora_v1;
REGISTER_USERMOD(gatecontrol_lora_v1);
