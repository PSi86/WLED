#include "gatecontrol_lora_v1.h"
//#include <WiFi.h>  // fallback for WiFi.macAddress()

#ifdef GC_EPAPER
#include "gc_epaper.h"
#endif

static LoraLink::Core ll{};
static LoraLink::Callbacks cb{};

// --- Last RX capture (for Info UI) ---
static uint8_t lastRxRaw[64];
static volatile uint8_t lastRxLen = 0;        // volatile: wird in anderem Kontext geschrieben
static char lastRxHex[(64 * 3) + 1];          // "FF " * 64 + '\0'
//static uint32_t lastRxSeenMs = 0;

static uint16_t debugCounter = 0;

static const char HEXLUT[] = "0123456789ABCDEF";

struct StartblockMsgV1 {
  uint8_t  slot;      // 1..4
  char     chan[3];   // "R1", null-terminated
  const uint8_t* name_ptr;
  uint8_t  name_len;
};

static bool parseStartblockV1(const uint8_t* data, size_t len, StartblockMsgV1& out) {
  if (!data || len < 5) return false;

  size_t pos = 0;
  uint8_t ver = data[pos++];
  if (ver != 0x01) return false;

  out.slot = data[pos++];

  out.chan[0] = (char)data[pos++];   // fixed 2 bytes
  out.chan[1] = (char)data[pos++];
  out.chan[2] = '\0';

  out.name_len = data[pos++];        // uint8
  if (pos + out.name_len > len) return false;

  out.name_ptr = &data[pos];
  return true;
}

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

// ======= um_data_t helpers =======
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
  current.presetId   = 11;
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
  
  #ifdef GC_EPAPER
    epaperInit();
  #endif
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
  #ifdef GC_EPAPER
  service_epaper(); // e-paper display service
  #endif
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

  // Control flags (compact)
  {
    char fbuf[40];
    fbuf[0] = '\0';

    // Power
    if (current.flags & GC_FLAG_POWER_ON) strlcat(fbuf, "ON", sizeof(fbuf));
    else                                 strlcat(fbuf, "OFF", sizeof(fbuf));

    // Start behaviour / sources
    if (current.flags & GC_FLAG_ARM_ON_SYNC)   strlcat(fbuf, " ARM", sizeof(fbuf));

    if (current.flags & GC_FLAG_HAS_BRI)       strlcat(fbuf, " BriCFG", sizeof(fbuf));
    else                                       strlcat(fbuf, " BriSYNC", sizeof(fbuf));

    // Options
    if (current.flags & GC_FLAG_FORCE_TT0)     strlcat(fbuf, " TT0", sizeof(fbuf));
    if (current.flags & GC_FLAG_FORCE_REAPPLY) strlcat(fbuf, " RE", sizeof(fbuf));

    JsonArray row = user.createNestedArray(F("Ctrl Flags"));
    row.add(fbuf);
  }

  // Sync age
  {
    char abuf[20];
    if (lastSyncLocalMs == 0) {
      strncpy(abuf, "n/a", sizeof(abuf));
      abuf[sizeof(abuf) - 1] = 0;
    } else {
      uint32_t ageMs = millis() - lastSyncLocalMs;
      uint32_t s = ageMs / 1000;
      uint32_t d = (ageMs % 1000) / 100;  // 0..9 -> 0.1s
      snprintf(abuf, sizeof(abuf), "%lu.%lus", (unsigned long)s, (unsigned long)d);
    }

    JsonArray row = user.createNestedArray(F("Sync Age"));
    row.add(abuf);
  }

  // Timebase error at last sync
  {
    char ebuf[20];
    if (lastSyncLocalMs == 0) {
      strncpy(ebuf, "n/a", sizeof(ebuf));
      ebuf[sizeof(ebuf) - 1] = 0;
    } else {
      snprintf(ebuf, sizeof(ebuf), "%ldms", (long)lastSyncTbErrMs);
    }

    JsonArray row = user.createNestedArray(F("TB Err (last)"));
    row.add(ebuf);
  }

  // Preset info (optional)
  {
    char pbuf[32];
    snprintf(pbuf, sizeof(pbuf), "cur=%u pend=%u %s",
             (unsigned)current.presetId,
             (unsigned)pending.presetId,
             pending.armed ? "ARMED" : "RUN");
    JsonArray row = user.createNestedArray(F("Preset"));
    row.add(pbuf);
  }

}

// ========= Config =========
void UsermodGateControlLoRa::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject("GateLoRa");
  top["groupId"] = current.groupId;
  top["macFilterEnabled"] = macFilterEnabled;  // default ON
  top["macFilterPersist"] = macFilterPersist;  // default OFF

  #if DEV_TYPE == 50
    top[F("Number of Slots (1-8)")] = numberOfSlots;
    top[F("First Slot (1-8)")] = firstSlot;
  #endif

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

  #if DEV_TYPE == 50
    uint8_t slots = numberOfSlots;
    uint8_t first = firstSlot;
    getJsonValue(top[F("Number of Slots (1-8)")], slots, 1);
    getJsonValue(top[F("First Slot (1-8)")], first, 1);
    numberOfSlots = constrain(slots, (uint8_t)1, (uint8_t)8);
    firstSlot = constrain(first, (uint8_t)1, (uint8_t)8);
  #endif

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
  // Mirror current runtime state for STATUS replies / UI.
  // IMPORTANT: Do NOT map effectCurrent -> presetId (different concept).
  current.brightness = bri;

  if (bri > 0) current.flags |= GC_FLAG_POWER_ON;
  else         current.flags &= (uint8_t)~GC_FLAG_POWER_ON;

  // currentPreset is the WLED preset index (0 = none)
  current.presetId = currentPreset;
}

// ========= Radio =========
bool UsermodGateControlLoRa::radioInit() {
  // SPI
  spi->begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_NSS);

  // RadioLib Module(cs, dio1, rst, busy, spi)
  #if defined(GATE_LORA_SX1262)
  static SX1262 r(new Module(LORA_PIN_NSS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY, *spi));
  #elif defined(GATE_LORA_LLCC68)
  static LLCC68 r(new Module(LORA_PIN_NSS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY, *spi));
  #else
  #error "No LoRa radio module defined"
  #endif
  
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

bool UsermodGateControlLoRa::handleStreamPacket(const uint8_t* buf, uint8_t len, const uint8_t senderLast3[3]) {
  using namespace LoraProto;
  if (len != (sizeof(Header7) + sizeof(P_Stream))) return false;

  const uint8_t* body = buf + sizeof(Header7);
  const uint8_t ctrl = body[0];
  const uint8_t totalPackets = (uint8_t)((ctrl >> 4) & 0x0F);
  const uint8_t packetIndex = (uint8_t)(ctrl & 0x0F);

  if (totalPackets == 0 || totalPackets > STREAM_MAX_PACKETS) return false;
  if (packetIndex >= totalPackets) return false;

  const uint8_t dataLen = STREAM_CHUNK_SIZE;

  if (packetIndex == 0 || totalPackets != streamTotalPackets) {
    streamReceivedMask = 0;
    streamTotalPackets = totalPackets;
    streamLength = 0;
  }

  const uint16_t offset = (uint16_t)packetIndex * STREAM_CHUNK_SIZE;
  if (offset + dataLen > STREAM_BUFFER_SIZE) return false;

  memcpy(streamBuffer + offset, body + 1, dataLen);
  streamReceivedMask |= (uint8_t)(1u << packetIndex);

  const uint8_t expectedMask = (uint8_t)((1u << totalPackets) - 1u);
  if (streamReceivedMask == expectedMask) {
    streamLength = (uint16_t)(totalPackets * STREAM_CHUNK_SIZE);
    sendAckTo(senderLast3, OPC_STREAM, ACK_OK);
    StartblockMsgV1 startblock{};
    if (parseStartblockV1(streamBuffer, streamLength, startblock)) {
      char nameBuf[STREAM_BUFFER_SIZE];
      size_t nameLen = startblock.name_len;
      if (nameLen >= sizeof(nameBuf)) nameLen = sizeof(nameBuf) - 1;
      memcpy(nameBuf, startblock.name_ptr, nameLen);
      nameBuf[nameLen] = '\0';

      char logBuf[160];
      snprintf(logBuf, sizeof(logBuf),
        "[GateLoRa] STREAM Startblock v1 slot %u chan %s name %s",
        startblock.slot, startblock.chan, nameBuf);
      DEBUG_PRINTLN(logBuf);
#ifdef GC_EPAPER
      setPilotSlotData(nameBuf, startblock.chan, startblock.slot);
#endif
    } else {
      DEBUG_PRINTLN(F("[GateLoRa] STREAM Startblock v1 parse failed"));
    }
    streamReceivedMask = 0;
    streamTotalPackets = 0;
    streamLength = 0;
    return true;
  }

  return false;
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

    case OPC_CONTROL: { // CONTROL: preset config (arm + optional flags/brightness)
      P_Control p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;

      handleControl(p);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] CONTROL -> configured"));
    } break;

    case OPC_SYNC: { // SYNC pulse (global)
      P_Sync p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;

      const uint32_t ts24 = ((uint32_t)p.ts24_0) | ((uint32_t)p.ts24_1 << 8) | ((uint32_t)p.ts24_2 << 16);
      handleSync(ts24, p.brightness);
      acted = true;
      //DEBUG_PRINTLN(F("[GateLoRa] SYNC -> processed"));
    } break;

    case OPC_CONFIG: {
      LoraProto::P_Config p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      //if (!groupMatch(p.groupId)) break;
      if (LoraLink::isBroadcast3(h.receiver)) return;  // only unicast allowed for config

      sendAckTo(h.sender, OPC_CONFIG, ACK_OK); // ACK first because some options may take time
      acted = true;

      if (p.option == 0x01) { // MAC Filter Enable/Disable
        macFilterEnabled = (p.data0 != 0);
      } else if (p.option == 0x02) { // Clear learned Master
        clearMaster();
      } else if (p.option == 0x03) { // MAC Filter Persist Enable/Disable
        macFilterPersist = (p.data0 != 0);
      } else if (p.option == 0x04) { // Enable AP Mode
        if (p.data0 != 0) WLED::instance().initAP(true);
        else {
          dnsServer.stop();
          WiFi.softAPdisconnect(true);
          apActive = false;
        }
      } else if (p.option == 0x81) { // Reboot Node
        if (p.data0 != 0) doReboot = true;
      }
      #if DEV_TYPE == 50
      else if (p.option == 0x8C) { // Number of Slots
        const uint8_t value = constrain(p.data0, (uint8_t)1, (uint8_t)8);
        if (numberOfSlots != value) {
          numberOfSlots = value;
          configNeedsWrite = true;
        }
      } else if (p.option == 0x8D) { // First Slot
        const uint8_t value = constrain(p.data0, (uint8_t)1, (uint8_t)8);
        if (firstSlot != value) {
          firstSlot = value;
          configNeedsWrite = true;
        }
      }
      #endif
      else {
        // unknown option
        break;
      }
      
      //DEBUG_PRINTLN(F("[GateLoRa] CONFIG -> applied"));
    } break;

    case OPC_STATUS: { // GET_STATUS -> STATUS_REPLY
      P_GetStatus p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;

      sendStatusReplyTo(h.sender);
      acted = true;
      DEBUG_PRINTLN(F("[GateLoRa] GET_STATUS -> STATUS_REPLY"));
    } break;

    case OPC_STREAM: {
      acted = handleStreamPacket(buf, (uint8_t)len, h.sender);
    } break;
  }

  if (acted) rxAccepted++;
}

void UsermodGateControlLoRa::sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac) {
  using namespace LoraProto;
  uint8_t out[32];

  P_IdentifyReply p{};
  p.fw = PROTO_VER_MAJOR; // fw = protocol version
  p.caps            = DEV_TYPE; // caps in dev_type umbenennen
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
  {
    uint8_t fl = current.flags;
    if (bri > 0) fl |= GC_FLAG_POWER_ON; else fl &= (uint8_t)~GC_FLAG_POWER_ON;
    p.flags      = fl;
    uint8_t cfg = 0;
    if (macFilterEnabled) cfg |= GC_CFG_MAC_FILTER_ENABLED;
    if (macFilterPersist) cfg |= GC_CFG_MAC_FILTER_PERSIST;
    if (apActive) cfg |= GC_CFG_AP_ACTIVE;
    p.configByte = cfg;
    p.presetId   = currentPreset;
    p.brightness = bri;
  }
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

// ========= Apply CONTROL (legacy immediate) =========
void UsermodGateControlLoRa::applyControl(const GateCore& in) {
  // Immediate apply (no SYNC). Kept for debug/compat.
  uint8_t desiredBri = 0;
  if (in.flags & GC_FLAG_POWER_ON) {
    if (in.flags & GC_FLAG_HAS_BRI) desiredBri = in.brightness;
    else desiredBri = bri; // keep current if not specified
  }
  bri = desiredBri;

  applyPreset(in.presetId, CALL_MODE_NO_NOTIFY);
  stateUpdated(CALL_MODE_NO_NOTIFY);

  // Preserve node groupId (CONTROL.groupId is a selector, not a "set group" command)
  current.flags      = in.flags;
  current.presetId   = in.presetId;
  current.brightness = bri;
  haveControl = true;
}

// ========= CONTROL (Preset/Brightness/Flags:ARM, GC_FLAG_FORCE_TT0, etc) handler =========
void UsermodGateControlLoRa::handleControl(const GateCore& cfg) {
  pending.presetId = cfg.presetId;
  pending.flags    = cfg.flags;
  pending.bri      = cfg.brightness;
  pending.rxAtMs   = millis();

  haveControl = true;

  // Mirror latest config (even before the preset is started)
  current.flags    = cfg.flags;
  current.presetId = cfg.presetId;
  if (cfg.flags & GC_FLAG_HAS_BRI) current.brightness = cfg.brightness;

  // Recommended path: arm and start on next SYNC
  pending.armed = (cfg.flags & GC_FLAG_ARM_ON_SYNC) != 0;

  if (!pending.armed) {
    // Apply immediately (will still be kept in phase by later SYNC pulses)
    uint16_t prevTT = 0;
    const bool ttForced = (cfg.flags & GC_FLAG_FORCE_TT0) != 0;
    if (ttForced) { prevTT = transitionDelay; transitionDelay = 0; }

    applyPreset(cfg.presetId, CALL_MODE_NO_NOTIFY);

    if (!(cfg.flags & GC_FLAG_POWER_ON)) {
      bri = 0;
    } else if (cfg.flags & GC_FLAG_HAS_BRI) {
      bri = cfg.brightness;
    }
    stateUpdated(CALL_MODE_NO_NOTIFY);

    if (ttForced) transitionDelay = prevTT;
  }
}

// ========= SYNC handler (global, irregular arrival OK) =========
void UsermodGateControlLoRa::handleSync(uint32_t ts24, uint8_t briFromPkt) {
  const uint32_t nowMs = millis();

// ---- unwrap 24-bit master timestamp (ms) to monotonic 32-bit ----
uint32_t masterAbsMs = 0;
if (!haveSync) {
  haveSync = true;
  masterAbsMs = (ts24 & 0x00FFFFFFUL);
} else {
  // Predict master time based on local elapsed time since last SYNC.
  const uint32_t pred = masterEpochAbsMs + (uint32_t)(nowMs - lastSyncLocalMs);

  // Place incoming 24-bit timestamp into the predicted 32-bit window.
  uint32_t cand = (pred & 0xFF000000UL) | (ts24 & 0x00FFFFFFUL);

  // Choose nearest wrap (half-range = 2^23 ms ≈ 2.33h).
  if (cand + 0x00800000UL < pred) cand += 0x01000000UL;
  else if (cand > pred + 0x00800000UL) cand -= 0x01000000UL;

  masterAbsMs = cand;
}

masterEpochAbsMs = masterAbsMs;
lastSyncLocalMs = nowMs;

  // ---- compute desired WLED timebase (wrap-safe) ----
  const uint32_t desiredTb = (uint32_t)(masterEpochAbsMs - nowMs);

  const int32_t err = (int32_t)(desiredTb - (uint32_t)strip.timebase);
  lastSyncTbErrMs = err; // debug/info
  const uint32_t aerr = (err < 0) ? (uint32_t)(-err) : (uint32_t)err;

  const bool hard = pending.armed || (aerr > (uint32_t)GC_SYNC_HARD_RESYNC_MS);
  if (hard) {
    strip.timebase = desiredTb;
  } else {
    int32_t step = err;
    if (step > (int32_t)GC_SYNC_MAX_STEP_MS) step = (int32_t)GC_SYNC_MAX_STEP_MS;
    if (step < -(int32_t)GC_SYNC_MAX_STEP_MS) step = -(int32_t)GC_SYNC_MAX_STEP_MS;
    strip.timebase = (uint32_t)((int32_t)strip.timebase + step);
  }

  // ---- start pending preset exactly on first SYNC after CONFIG ----
  if (pending.armed) {
    uint16_t prevTT = 0;
    const bool ttForced = (pending.flags & GC_FLAG_FORCE_TT0) != 0;
    if (ttForced) { prevTT = transitionDelay; transitionDelay = 0; }

    const bool needApply = ((pending.flags & GC_FLAG_FORCE_REAPPLY) != 0) || (currentPreset != pending.presetId);
    if (needApply) {
      applyPreset(pending.presetId, CALL_MODE_NO_NOTIFY);
    }

    // Brightness: use CONFIG.bri only if GC_FLAG_HAS_BRI is set.
    // If CONFIG did NOT include brightness, take it from SYNC packet (live brightness).
    if (!(pending.flags & GC_FLAG_POWER_ON)) {
      bri = 0;
    } else if (pending.flags & GC_FLAG_HAS_BRI) {
      bri = pending.bri;
    } else {
      bri = briFromPkt;
    }
    stateUpdated(CALL_MODE_NO_NOTIFY);

    current.flags      = pending.flags;
    current.presetId   = pending.presetId;
    current.brightness = bri;

    pending.armed = false;

    if (ttForced) transitionDelay = prevTT;
    return;
  }

  // ---- optional live brightness via SYNC (only if last CONFIG had NO brightness) ----
  if (haveControl && !(current.flags & GC_FLAG_HAS_BRI)) {
    const uint8_t desiredBri = (current.flags & GC_FLAG_POWER_ON) ? briFromPkt : 0;
    if (desiredBri != bri) {
      bri = desiredBri;
      current.brightness = bri;
      stateUpdated(CALL_MODE_NO_NOTIFY);
    }
  }
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
