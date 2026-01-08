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
  current.flags      = 0;
  current.presetId     = 11;
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

  // Pending preset / sync status
  {
    JsonArray row = user.createNestedArray(F("Preset Armed"));
    row.add(pending.armed ? F("yes") : F("no"));

    JsonArray row2 = user.createNestedArray(F("Pending Preset"));
    row2.add(String(pending.presetId));

    JsonArray row3 = user.createNestedArray(F("Pending Flags"));
    char f[8];
    snprintf(f, sizeof(f), "0x%02X", pending.flags);
    row3.add(f);

    JsonArray row4 = user.createNestedArray(F("Last SYNC"));
    if (!haveSync) {
      row4.add(F("(none)"));
    } else {
      uint32_t ageMs = millis() - lastSyncLocalMs;
      char s[24];
      snprintf(s, sizeof(s), "%lums ago", (unsigned long)ageMs);
      row4.add(s);
    }
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
  // In this usermod, current.effect is treated as the last commanded PRESET ID (not effect index).
  // Avoid overwriting it with effectCurrent (which is an effect index) on generic state changes.
  current.brightness = bri;
  current.flags      = (bri > 0) ? 1 : 0; // TODO: handle ON/OFF properly // use flags
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

      // mirror state (preset id semantics)
      current.flags      = (bri > 0) ? 1 : 0; // TODO: handle ON/OFF properly // use flags
      current.presetId     = 11;
      current.brightness = bri;

      sendAckTo(h.sender, OPC_SET_GROUP, ACK_OK);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] SET_GROUP -> applied + ACK"));
    } break;

    case OPC_CONTROL: {
      // CONFIG (preset + flags) using the legacy 4B P_Control layout.
      // NOTE: We intentionally do NOT apply the preset immediately if GC_FLAG_ARM_ON_SYNC is set.
      LoraProto::P_Control p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;

      if (!groupMatch(p.groupId)) break;

      handleConfig(p);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] CONTROL -> config received"));
    } break;

    case OPC_SYNC: {
      // SYNC packet: [phase16_lo][phase16_hi][bri]
      LoraProto::P_Sync p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;

      const uint8_t* b = buf + sizeof(Header7);
      const uint8_t g = b[0];
      if (!groupMatch(g)) break;

      bool hasSeq = false;
      uint8_t seq = 0;
      uint16_t phase16 = 0;
      bool hasBri = false;
      uint8_t briFromPkt = 0;

      if (bodyLen == 3) {
        // [g][p0][p1]
        phase16 = (uint16_t)b[1] | ((uint16_t)b[2] << 8);
      } else {
        // [g][seq][p0][p1]...
        hasSeq = true;
        seq = b[1];
        phase16 = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
        if (bodyLen >= 5) {
          hasBri = true;
          briFromPkt = b[4];
        }
      }

      handleSync(phase16, briFromPkt);
      acted = true;
    } break;

    case OPC_CONFIG: {
      LoraProto::P_Config p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      //if (!groupMatch(p.groupId)) break;
      if (LoraLink::isBroadcast3(h.receiver)) return;  // only unicast allowed for config

      if (p.option == 0x01) { // MAC Filter Enable/Disable
        macFilterEnabled = (p.flags != 0);
      } else if (p.option == 0x02) { // Clear learned Master
        clearMaster();
      } else if (p.option == 0x03) { // MAC Filter Persist Enable/Disable
        macFilterPersist = (p.flags != 0);
      } else if (p.option == 0x04) { // Enable AP Mode
        if (p.flags != 0) WLED::instance().initAP(true);
        else {
          dnsServer.stop();
          WiFi.softAPdisconnect(true);
          apActive = false;
        }
      } else if (p.option == 0x05) { // Reboot Node
        if (p.flags != 0) doReboot = true;
      }
      else {
        // unknown option
        break;
      }
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] CONFIG -> applied"));
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
  p.flags      = current.flags;
  p.presetId     = current.presetId;
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

// ===================== WLED preset-sync implementation =====================

static inline uint32_t unwrap16_to32(uint16_t v16, uint32_t last32) {
  // Choose the 16-bit wrapped value that is closest to last32.
  uint32_t base = last32 & 0xFFFF0000UL;
  uint32_t cand = base | (uint32_t)v16;
  // If cand is more than half-range behind/ahead, wrap.
  if (cand + 0x8000UL < last32) cand += 0x10000UL;
  else if (cand > last32 + 0x8000UL) cand -= 0x10000UL;
  return cand;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void UsermodGateControlLoRa::handleConfig(const GateCore& cfg) {
  // cfg fields mapping (see header): groupId, flags(state), presetId(effect), bri(brightness)
  pending.presetId = cfg.presetId;
  pending.flags    = cfg.flags;
  pending.bri      = cfg.brightness;
  pending.rxAtMs   = millis();

  // Default behavior: wait for next SYNC before applying.
  pending.armed = (pending.flags & GC_FLAG_ARM_ON_SYNC) != 0;

  // If master explicitly does NOT want to wait for SYNC, apply immediately (not time-aligned).
  if (!pending.armed) {
    GateCore in = cfg;
    // interpret flags: bit0 = power
    uint8_t outBri = (pending.flags & GC_FLAG_HAS_BRI) ? pending.bri : (uint8_t)bri;
    if (!(pending.flags & GC_FLAG_POWER_ON)) outBri = 0;
    in.flags = (outBri > 0) ? 1 : 0;
    in.brightness = outBri;
    applyControl(in);
  }

  // Optionally learn master on first meaningful packet (keeps old behavior: discovery/grouping)
  // (Master learning is handled earlier in handlePacket for SET_GROUP; keep this only if desired)
}

void UsermodGateControlLoRa::handleSync(uint16_t phase16, uint8_t briFromPkt) {
  const uint32_t nowLocalMs = millis();

  // Convert compact phase to unwrapped ms
  // phase16 counts GC_SYNC_TICK_MS ticks
  uint32_t phaseTicks32 = 0;
  if (!haveSync) {
    phaseTicks32 = (uint32_t)phase16;
  } else {
    // unwrap in tick-domain, then convert to ms
    const uint32_t lastTicks32 = lastPhaseMs / GC_SYNC_TICK_MS;
    phaseTicks32 = unwrap16_to32(phase16, lastTicks32);
  }
  const uint32_t phaseMs32 = phaseTicks32 * (uint32_t)GC_SYNC_TICK_MS;

  // Compute desired WLED effect timebase so that (millis() + strip.timebase) == phaseMs32
  // WLED uses (millis() + strip.timebase) as the effect 'now' reference.
  // timebase is uint32; subtraction wrap is fine.
  const uint32_t desiredTimebase = (uint32_t)(phaseMs32 - nowLocalMs);

  bool hardSet = false;
  if (!haveSync || pending.armed) {
    hardSet = true;
  } else {
    // If sync interval is very long, do a hard set (unwrap ambiguity risk)
    uint32_t dt = nowLocalMs - lastSyncLocalMs;
    if (dt > 300000UL) { // >5 min
      hardSet = true;
    }
  }

  if (hardSet) {
    strip.timebase = desiredTimebase;
  } else {
    // Slew-limited correction to avoid visible jumps from packet jitter
    int32_t err = (int32_t)(desiredTimebase - strip.timebase);
    if (err > (int32_t)GC_SYNC_HARD_RESYNC_MS || err < -(int32_t)GC_SYNC_HARD_RESYNC_MS) {
      strip.timebase = desiredTimebase;
    } else {
      int32_t step = clamp_i32(err, -(int32_t)GC_SYNC_MAX_STEP_MS, (int32_t)GC_SYNC_MAX_STEP_MS);
      strip.timebase = (uint32_t)((int32_t)strip.timebase + step);
    }
  }

  // Start the armed preset exactly on the first SYNC after CONFIG
  if (pending.armed) {
    const uint8_t flags = pending.flags;

    // Optionally force transition delay to 0 for this start.
    // This avoids unsynced crossfades.
    uint16_t oldTransition = transitionDelay;
    if (flags & GC_FLAG_FORCE_TT0) {
      transitionDelay = 0;
    }

    // Decide whether to apply even if already active
    bool needApply = true;
    if (!(flags & GC_FLAG_FORCE_REAPPLY)) {
      if (current.presetId == pending.presetId && current.flags == ((flags & GC_FLAG_POWER_ON) ? 1 : 0)) {
        needApply = false;
      }
    }

    // Apply preset (RX-only; do not notify)
    if (needApply) {
      applyPreset(pending.presetId, CALL_MODE_NO_NOTIFY);
    }

    // Brightness handling
    uint8_t outBri = bri;
    if (flags & GC_FLAG_HAS_BRI) { // config packet specified a brightness
      outBri = pending.bri; // use that
    }
    else {
      outBri = briFromPkt; // use SYNC packet brightness
    }
    if (!(flags & GC_FLAG_POWER_ON)) { // power on flag not set
      outBri = 0;
    }
    bri = outBri;

    // Pull changes through without causing UDP notifications
    stateUpdated(CALL_MODE_NO_NOTIFY);

    // Restore transition delay
    if (flags & GC_FLAG_FORCE_TT0) {
      transitionDelay = oldTransition;
    }

    // Update current mirror
    //current.groupId    = current.groupId; // unchanged
    current.flags      = (outBri > 0) ? 1 : 0;
    current.presetId   = pending.presetId;
    current.brightness = outBri;

    pending.armed = false;
  } else {
    // Optional: allow live brightness via SYNC packet without restarting the preset
    if (!(pending.flags & GC_FLAG_HAS_BRI)) { // only if CONFIG did not specify a brightness
      if (bri != briFromPkt) {
        bri = briFromPkt;
        stateUpdated(CALL_MODE_NO_NOTIFY);
        current.brightness = briFromPkt;
        current.flags = (briFromPkt > 0) ? 1 : 0;
      }
    }
  }

  // Store sync history
  haveSync = true;
  lastPhaseMs = phaseMs32;
  lastSyncLocalMs = nowLocalMs;
}

// ========= Apply CONTROL =========
void UsermodGateControlLoRa::applyControl(const GateCore& in) {
  // state -> on/off via brightness, effect -> preset id, brightness direct
  bri = (in.flags > 0) ? in.brightness : 0;
  applyPreset(in.presetId, CALL_MODE_NO_NOTIFY);  // RX-only: no UDP/WiFi notify
  
  //effectPalette += 1;
  //if (effectPalette <= 50) effectPalette = in.palette;
  //colorUpdated(CALL_MODE_FX_CHANGED);
  
  stateUpdated(CALL_MODE_NO_NOTIFY);

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
