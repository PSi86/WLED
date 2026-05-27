#include "racelink_wled.h"
//#include <WiFi.h>  // fallback for WiFi.macAddress()
#include "pin_manager.h"
#include "rf_config_nvs.h"

#ifdef RACELINK_EPAPER
#include "racelink_epaper.h"
#endif

static RaceLinkTransport::Core rl{};
static RaceLinkTransport::Callbacks cb{};

// Currently active LoRa PHY configuration. Initialised in radioInit()
// from NVS (with compile-time defaults as fallback) and consumed by
// the PhyCfg builder. Updated by the OPC_RF_CONFIG handler before the
// queued reboot so a same-boot GET_RF_CONFIG sees the new values.
static RaceLinkProto::P_RfConfig g_activeRfConfig{};

// Compile-time defaults expressed in the wire-format P_RfConfig layout.
// Used as the first-boot seed (so a fresh node persists a valid slot
// before its first OPC_RF_CONFIG) and as the boot-loop recovery target
// after BOOT_COUNTER_MAX strikes. The WLED-side define names differ
// from the gateway's (RACELINK_CR / RACELINK_SYNC_WORD vs the
// gateway's RACELINK_CR_DEN / RACELINK_SYNCWORD), which is why the
// helper lives at the call site rather than in rf_config_nvs.h.
static inline RaceLinkProto::P_RfConfig getCompileDefaultRfConfig() {
  RaceLinkProto::P_RfConfig c{};
  c.freq_hz       = RACELINK_FREQ_HZ;
  c.bw_khz_x10    = (uint16_t)(RACELINK_BW_KHZ * 10.0f + 0.5f);
  c.sf            = RACELINK_SF;
  c.cr_den        = RACELINK_CR;
  c.sync_word     = RACELINK_SYNC_WORD;
  c.tx_power_dbm  = (int8_t)RACELINK_TX_POWER;
  c.preamble      = RACELINK_PREAMBLE;
  return c;
}

// --- Last RX capture (for Info UI) ---
static uint8_t lastRxRaw[64];
static volatile uint8_t lastRxLen = 0;        // volatile: written from a different context
static char lastRxHex[(64 * 3) + 1];          // "FF " * 64 + '\0'
//static uint32_t lastRxSeenMs = 0;

static constexpr size_t STREAM_INFO_MAX = 128; // keep aligned with STREAM_BUFFER_SIZE
static uint8_t lastStreamRaw[STREAM_INFO_MAX];
static volatile uint8_t lastStreamLen = 0;
static char lastStreamHex[(STREAM_INFO_MAX * 3) + 1];
static uint32_t lastStreamAtMs = 0;

static uint16_t debugCounter = 0;
static constexpr uint32_t STARTUP_IDENTIFY_FIRST_DELAY_MS = 1000;
static constexpr uint32_t STARTUP_IDENTIFY_SECOND_DELAY_MS = 10000;

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

  // store raw bytes
  memcpy(lastRxRaw, buf, n);
  lastRxLen = (uint8_t)n;
  //lastRxSeenMs = millis();

  // build single-line hex string
  char* p = lastRxHex;
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = lastRxRaw[i];
    *p++ = HEXLUT[b >> 4];
    *p++ = HEXLUT[b & 0x0F];
    *p++ = ' ';
  }
  if (n) *(p - 1) = '\0';  // replace trailing space with NUL terminator
  else   *p = '\0';
}

static inline void captureLastStreamPacket(const uint8_t* buf, size_t len) {
  if (!buf) { lastStreamLen = 0; lastStreamHex[0] = '\0'; lastStreamAtMs = 0; return; }

  size_t n = (len > STREAM_INFO_MAX) ? STREAM_INFO_MAX : len;

  memcpy(lastStreamRaw, buf, n);
  lastStreamLen = (uint8_t)n;
  lastStreamAtMs = millis();

  char* p = lastStreamHex;
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = lastStreamRaw[i];
    *p++ = HEXLUT[b >> 4];
    *p++ = HEXLUT[b & 0x0F];
    *p++ = ' ';
  }
  if (n) *(p - 1) = '\0';
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

// TODO: remove s_gateSelf?
// ======= Self-pointer for ISR =======
static UsermodRaceLink* s_gateSelf = nullptr;

// ========= ISR (no args, RadioLib-compatible) =========
/* void IRAM_ATTR UsermodRaceLink::onRxStatic() {
  if (s_gateSelf) s_gateSelf->rxFlag = true;
} */

// ========= Setup =========
void UsermodRaceLink::setup() {
  // Re-apply fleet-uniformity defaults (FPS, ABL, gamma) before anything else
  // runs. By now WLED has deserialised cfg.json, so the affected globals carry
  // whatever the operator's saved config said — possibly drifted from the
  // fleet's expected values. RaceLink is the authoritative source for these
  // settings, not per-device cfg.json.
  applyRaceLinkDefaults();

  // init defaults for current gate state
  current.groupId    = 0;
  current.flags      = 0;
  current.presetId   = 11;
  current.brightness = 128;

  // RaceLink: nodes operate in AP+STA mode during firmware updates,
  // which means WLED's `Network.localIP()` returns the STA address
  // (not the AP's 4.3.2.1) and the `inSameSubnet` check at
  // wled_server.cpp:529 rejects AP-side OTA hosts even when they
  // hold a valid 4.3.2.x DHCP lease. Forcing this off makes /update
  // use the broader `inLocalSubnet` test which explicitly accepts
  // 4.3.2.0/24 when apActive. Runs after deserializeConfigFromFS()
  // so it overrides whatever cfg.json carries — survives factory
  // resets and persistent operator configs alike. Host-side
  // OTAService._wled_attempt_unlock is the runtime safety net for
  // any device flashed before this line was added.
  // otaSameSubnet = false;

// read MAC from efuse (no WiFi init required)
  // readEfuseMac(); // now in RaceLinkTransport::beginCommon()

  // init modem
  radioReady = radioInit();
  if (!radioReady) {
    DEBUG_PRINTLN(F("[RaceLink] Radio init FAILED"));
    return;
  }

  cb.onRxPacket = &UsermodRaceLink::on_rx_node;
  //cb.onTxDone   = &UsermodRaceLink::on_tx_done_node;
  cb.ctx        = this; // critical: required by handlePacket

  DEBUG_PRINTLN(F("[RaceLink] Radio init OK"));
  const uint32_t nowMs = millis();
  startupIdentifyStage = 0;
  startupIdentifyAtMs[0] = nowMs + STARTUP_IDENTIFY_FIRST_DELAY_MS;
  startupIdentifyAtMs[1] = nowMs + STARTUP_IDENTIFY_SECOND_DELAY_MS;
  
  #ifdef RACELINK_EPAPER
    {
      const PinManagerPinType epdPins[] = {
        { epdSck,  true  },
        { epdMosi, true  },
        { epdMiso, false },
        { epdCs,   true  },
        { epdDc,   true  },
        { epdRst,  true  },
        { epdBusy, false },
      };
      if (PinManager::allocateMultiplePins(epdPins,
                                           sizeof(epdPins) / sizeof(epdPins[0]),
                                           PinOwner::UM_Unspecified)) {
        epaperInit(epdMosi, epdSck, epdMiso, epdCs, epdDc, epdRst, epdBusy);
        #if DEV_TYPE == 50
          setDisplayLayout(numberOfSlots);
        #endif
      } else {
        DEBUG_PRINTLN(F("[RaceLink] ePaper PinManager allocation failed — display disabled"));
      }
    }
  #endif

  // Persistent boot color: rolled once on the very first boot (when cfg.json
  // has no value yet -> overrides.bootColorMode stays at 0xFF), saved
  // immediately so every subsequent boot reuses the same color. Operator can
  // change it later via the 1-click cycle; serviceBootColorSave() persists
  // the new pick after the post-click idle window. The runtime mirror in
  // btn.* feeds both the boot display and the SCENE_RESTORE_BOOT_COLOR path,
  // so the value is also kept consistent on units where bootPreset != 0
  // suppresses the local display.
  if (overrides.bootColorMode > 3) {
    overrides.bootColorMode = (uint8_t)(esp_random() % 3);
    overrides.bootColorR = 0;
    overrides.bootColorG = 0;
    overrides.bootColorB = 0;
    configNeedsWrite = true;
  }
  btn.bootColorMode   = overrides.bootColorMode;
  btn.bootColorRgb[0] = overrides.bootColorR;
  btn.bootColorRgb[1] = overrides.bootColorG;
  btn.bootColorRgb[2] = overrides.bootColorB;

  // Boot effect: operator did NOT set a boot preset -> show the persisted
  // boot color so it is visible that the device has booted.
  // When bootPreset != 0, WLED's standard path (beginStrip()/applyPreset)
  // runs unchanged and our display is suppressed.
  if (bootPreset == 0) {
    applyBootColor();
  }

  // Headless Mode auto-resume: if the device was last running as Headless
  // Master before the power-cycle, run the IDENTIFY_REPLY probe to verify
  // no real Gateway has taken over in the meantime. Two simultaneously
  // powered-on persisted-headless devices resolve cleanly via the jitter
  // inside tryStartHeadless() — the first one to finish its probe claims
  // master and answers the other's probe with OPC_SET_GROUP, demoting it.
  if (overrides.headlessPersistedActive) {
    tryStartHeadless();
  }
}

// ========= Loop =========
void UsermodRaceLink::loop() {
  // Button: poll the pin directly here so we sample at the full main-loop
  // rate. Going through WLED's UsermodManager::handleButton() dispatch is
  // unreliable for fast multi-tap because button.cpp:264 throttles the
  // dispatcher to one call per ~250 ms while strip.isUpdating(). Our
  // poll is independent of the radio so triple-tap recovery also works
  // when radio init failed.
  const uint32_t nowMs = millis();
  pollPhysicalButton(nowMs);
  serviceButtonFade(nowMs);
  serviceBootColorSave(nowMs);
  serviceHeadlessPersist(nowMs);
  serviceHeadlessReassign(nowMs);
  serviceSceneRebroadcast(nowMs);
  serviceIndicator(nowMs);

  if (!radio) return;

  // replaces the previous flag-/ISR-/onRx() handling
  RaceLinkTransport::service(rl, cb);
  serviceStartupIdentifyReplies();
  // Headless Mode probe state machine + keepalive. Cheap when neither
  // probing nor active (early-return on the first cycle).
  serviceHeadless(nowMs);
  // Offset-mode: fire any deferred apply whose deadline has elapsed.
  // Cheap when nothing is queued (single bool check).
  serviceDeferredApply();

  if (!batteryUM) {
    // Retry later until the Battery UM exposes its data
    if (UsermodManager::getUMData(&batteryUM, USERMOD_ID_BATTERY)) {
      DEBUG_PRINTLN(F("[RaceLink] Battery UM data acquired"));
    }
  }
  #ifdef RACELINK_EPAPER
  service_epaper(); // e-paper display service
  #endif
}

// ========= Info (UI) =========
void UsermodRaceLink::addToJsonInfo(JsonObject& root) {
  // "u" = object; each row is an array [labelValue1, labelValue2, ...]
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");

  // RaceLink Init
  {
    char initMsg[32];
    snprintf(initMsg, sizeof(initMsg), "%s (code %d)", radioReady ? "OK" : "FAIL", (int)radioInitCode);
    JsonArray row = user.createNestedArray(F("RaceLink Init"));
    row.add(initMsg);
  }

  // MyID 3B
  {
    char my3[12];
    snprintf(my3, sizeof(my3), "%02X:%02X:%02X", rl.myLast3[0], rl.myLast3[1], rl.myLast3[2]);
    JsonArray row = user.createNestedArray(F("RaceLink MyID (3B)"));
    row.add(my3);
  }

  // RX counters
  {
    JsonArray row = user.createNestedArray(F("RX total"));
    row.add(String((unsigned long)rl.rxCountTotal));

    JsonArray row2 = user.createNestedArray(F("RX accepted"));
    row2.add(String((unsigned long)rxAccepted));

    JsonArray row3 = user.createNestedArray(F("TX total"));
    row3.add(String((unsigned long)rl.txCount));
  }

  // Last RSSI/SNR
  {
    char sig[24];
    snprintf(sig, sizeof(sig), "%d / %d", (int)rl.lastRssi, (int)rl.lastSnr);
    JsonArray row = user.createNestedArray(F("Last RSSI/SNR"));
    row.add(sig);
  }

  {
    JsonArray row = user.createNestedArray(F("Last RX"));
    if (lastRxLen == 0) {
      row.add(F("(none)"));
    } else {
      char meta[32];
      uint32_t age = (millis() - rl.lastRxAtMs) / 1000;
      snprintf(meta, sizeof(meta), "%uB (%lus ago)", lastRxLen, (unsigned long)age);
      row.add(meta);

  
      JsonArray rowHex = user.createNestedArray(F("Last RX Hex"));
      if (lastRxLen == 0) rowHex.add(F("(none)"));
      else                rowHex.add(lastRxHex);
    }
  }

  {
    JsonArray row = user.createNestedArray(F("Last Stream"));
    if (lastStreamLen == 0) {
      row.add(F("(none)"));
    } else {
      char meta[32];
      uint32_t age = (millis() - lastStreamAtMs) / 1000;
      snprintf(meta, sizeof(meta), "%uB (%lus ago)", lastStreamLen, (unsigned long)age);
      row.add(meta);

      JsonArray rowHex = user.createNestedArray(F("Last Stream Hex"));
      rowHex.add(lastStreamHex);
    }
  }

  {
    char debug[16];
    //snprintf(debug, sizeof(debug), "%d", (int)debugCounter);
    snprintf(debug, sizeof(debug), "%d", (int)rl.debug);
    //snprintf(debug, sizeof(debug), "%d", (int)rl.toaUsMaxPkt/1000U);
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
    if (current.flags & RACELINK_FLAG_POWER_ON) strlcat(fbuf, "ON", sizeof(fbuf));
    else                                 strlcat(fbuf, "OFF", sizeof(fbuf));

    // Start behaviour / sources
    if (current.flags & RACELINK_FLAG_ARM_ON_SYNC)   strlcat(fbuf, " ARM", sizeof(fbuf));

    if (current.flags & RACELINK_FLAG_HAS_BRI)       strlcat(fbuf, " BriCFG", sizeof(fbuf));
    else                                       strlcat(fbuf, " BriSYNC", sizeof(fbuf));

    // Options
    if (current.flags & RACELINK_FLAG_FORCE_TT0)     strlcat(fbuf, " TT0", sizeof(fbuf));
    if (current.flags & RACELINK_FLAG_FORCE_REAPPLY) strlcat(fbuf, " RE", sizeof(fbuf));

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

  // Preset / pending CONTROL info
  {
    char pbuf[32];
    snprintf(pbuf, sizeof(pbuf), "cur=%u %s",
             (unsigned)current.presetId,
             pending.valid ? "ARMED" : "RUN");
    JsonArray row = user.createNestedArray(F("Preset"));
    row.add(pbuf);
  }

}

// ========= Config =========
void UsermodRaceLink::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject("RaceLink");
  top["groupId"] = current.groupId;
  top["macFilterEnabled"] = macFilterEnabled;  // default ON
  top["macFilterPersist"] = macFilterPersist;  // default OFF

  #if DEV_TYPE == 50
    top[F("Number of Slots (1-8)")] = numberOfSlots;
    top[F("First Slot (1-8)")] = firstSlot;
  #endif

  // Persisted master snapshot. We always serialise the stored slot, not
  // the live one — so toggling macFilterPersist or running OPC_RF_CONFIG
  // (which flips persistence off) doesn't lose the operator's pinned
  // binding. persistMasterIfNeeded() refreshes the slot when the
  // operator wants the current master pinned. clearMaster() only
  // touches the live state; the snapshot survives.
  char m3[7+1];
  sprintf(m3, "%02X%02X%02X",
          persistedMasterLast3[0], persistedMasterLast3[1], persistedMasterLast3[2]);
  top["masterLast3"] = String(m3);

  char m6[12+1];
  sprintf(m6, "%02X%02X%02X%02X%02X%02X",
          persistedMasterFull6[0], persistedMasterFull6[1], persistedMasterFull6[2],
          persistedMasterFull6[3], persistedMasterFull6[4], persistedMasterFull6[5]);
  top["masterFullMac"] = persistedMasterFull6Known ? String(m6) : String("");

  // radio defaults
  JsonObject l = top.createNestedObject("RL_RF");
  l["freq"] = RACELINK_FREQ_HZ;
  l["sf"]   = RACELINK_SF;
  l["bw"]   = (int)RACELINK_BW_KHZ;
  l["cr"]   = RACELINK_CR;
  l["sync"] = RACELINK_SYNC_WORD;
  l["txp"]  = RACELINK_TX_POWER;

  // RaceLink radio control pins. Defaults match the build profile (see
  // RACELINK_PIN_* macros in racelink_wled.h). Saved values are loaded into
  // the corresponding pin* members during readFromConfig() and applied when
  // radioInit() runs at the next boot.
  JsonObject pins = top.createNestedObject("pins");
  pins[F("SCK")]  = pinSck;
  pins[F("MISO")] = pinMiso;
  pins[F("MOSI")] = pinMosi;
  pins[F("NSS")]  = pinNss;
  pins[F("DIO1")] = pinDio1;
  pins[F("BUSY")] = pinBusy;
  pins[F("RST")]  = pinRst;

  #ifdef RACELINK_EPAPER
    JsonObject ep = top.createNestedObject("epaper_pins");
    ep[F("SCK")]  = epdSck;
    ep[F("MISO")] = epdMiso;
    ep[F("MOSI")] = epdMosi;
    ep[F("CS")]   = epdCs;
    ep[F("DC")]   = epdDc;
    ep[F("RST")]  = epdRst;
    ep[F("BUSY")] = epdBusy;
  #endif

  // RaceLink-authoritative overrides (settable via OPC_CONFIG 0x05..0x0A or
  // directly in this WebUI). All six slots are emitted unconditionally so
  // the operator always sees every override row in WLED Settings → Usermods,
  // and so OPC_GET_CONFIG can always round-trip. Slot values are pushed to
  // the live WLED globals by applyRaceLinkDefaults() — readFromConfig()
  // calls it after every Save so changes are live without a reboot.
  // Long descriptive keys: WLED's generic Settings → Usermods UI renders
  // JSON keys 1:1 as field labels, so the keys ARE the labels. Segment
  // fields are flat with a "Segment N " prefix because WLED does not
  // render parent-object names as section headers — nesting would just
  // produce four identical "Start LED"/"Stop LED" rows in the UI.
  JsonObject ov = top.createNestedObject(F("overrides"));
  ov[F("Target FPS")]               = overrides.fps;
  ov[F("ABL Max mA")]               = overrides.ablMaxMa;
  ov[F("Default Brightness")]       = overrides.briS;
  ov[F("Transition Duration (ms)")] = overrides.transitionMs;
  ov[F("Segment 0 Start LED")]      = overrides.seg0Start;
  ov[F("Segment 0 Stop LED")]       = overrides.seg0Stop;
  ov[F("Segment 1 Start LED")]      = overrides.seg1Start;
  ov[F("Segment 1 Stop LED")]       = overrides.seg1Stop;

  // Headless Mode runtime persistence (no WebUI rows — set via 5-click
  // gesture or via OPC_CONFIG in a future revision). Round-trips through
  // cfg.json so a power-cycle resumes Headless Master ownership.
  ov[F("Headless Active")]          = overrides.headlessPersistedActive;
  ov[F("Headless Group Counter")]   = overrides.headlessGroupCounter;
  ov[F("Headless Current Scene")]   = overrides.headlessCurrentScene;
  ov[F("Headless Broadcast Bri")]   = overrides.headlessBroadcastBri;

  // Persistent boot color identifier (per-device). See OverrideValues for the
  // mode encoding; R/G/B are only meaningful when mode == 3.
  ov[F("Boot Color Mode")]          = overrides.bootColorMode;
  ov[F("Boot Color R")]             = overrides.bootColorR;
  ov[F("Boot Color G")]             = overrides.bootColorG;
  ov[F("Boot Color B")]             = overrides.bootColorB;

  // Persistent Headless-master slave registry. Survives reboot/akku-swap so a
  // master that loses power mid-event can re-bind its slaves on resume.
  // Cleared by exitHeadlessMode().
  JsonArray slaves = ov.createNestedArray(F("Headless Slaves"));
  char hex[8];
  for (uint8_t i = 0; i < headlessSlavesCount; i++) {
    JsonObject o = slaves.createNestedObject();
    snprintf(hex, sizeof(hex), "%02X%02X%02X",
             headlessSlaves[i].addr3[0],
             headlessSlaves[i].addr3[1],
             headlessSlaves[i].addr3[2]);
    o[F("a")] = String(hex);
    o[F("g")] = headlessSlaves[i].groupId;
  }
}

bool UsermodRaceLink::readFromConfig(JsonObject& root) {
  JsonObject top = root["RaceLink"];
  if (top.isNull()) return false;

  // Snapshot before firstReadFromConfig is cleared further down. Determines
  // whether we should push the freshly-read overrides into live WLED globals
  // at the tail of this function (WebUI Save path) or skip it because
  // setup() will call applyRaceLinkDefaults() in a moment anyway (boot path).
  const bool wasFirstCall = firstReadFromConfig;

  // read flags first
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
    #ifdef RACELINK_EPAPER
      setDisplayLayout(numberOfSlots);
    #endif
  #endif

  // ALWAYS deserialise the persisted slot, regardless of
  // macFilterPersist — we keep the snapshot in RAM so addToConfig can
  // round-trip it intact through a Save the operator triggers for some
  // unrelated reason. macFilterPersist only gates whether we COPY the
  // snapshot into the live state below.
  {
    String s3 = top["masterLast3"] | "000000";
    if (s3.length() == 6) {
      uint32_t val = strtoul(s3.c_str(), nullptr, 16);
      persistedMasterLast3[0] = (val >> 16) & 0xFF;
      persistedMasterLast3[1] = (val >> 8)  & 0xFF;
      persistedMasterLast3[2] = (val)       & 0xFF;
    }

    String s6 = top["masterFullMac"] | "";
    if (s6.length() == 12) {
      for (int i=0;i<6;i++) {
        persistedMasterFull6[i] = strtoul(s6.substring(2*i, 2*i+2).c_str(), nullptr, 16);
      }
      persistedMasterFull6Known = true;
    } else {
      persistedMasterFull6Known = false;
      memset(persistedMasterFull6, 0, sizeof(persistedMasterFull6));
    }
  }

  // only adopt master from the snapshot into live state when persistence
  // is enabled. When off, the snapshot stays in RAM (and survives the
  // next addToConfig) but masterLast3/Full6 remain at their RAM defaults
  // so senderAllowed() goes through the !masterKnown discovery branch.
  if (macFilterPersist) {
    uint32_t val =
      ((uint32_t)persistedMasterLast3[0] << 16) |
      ((uint32_t)persistedMasterLast3[1] << 8) |
      ((uint32_t)persistedMasterLast3[2]);
    if (val != 0) {
      memcpy(masterLast3, persistedMasterLast3, 3);
      masterKnown = true;
    }
    if (persistedMasterFull6Known) {
      memcpy(masterFull6, persistedMasterFull6, 6);
      masterFull6Known = true;
    }
  }
  // When persistence is off: do NOT overwrite runtime values

  // RaceLink radio pins. Capture old values to detect a UI change so we can
  // request a reboot (SPI re-init at runtime is not supported by design).
  // On the very first call (boot-time deserialize), suppress the reboot —
  // the values being read ARE the boot-time values.
  const int8_t oldSck  = pinSck;
  const int8_t oldMiso = pinMiso;
  const int8_t oldMosi = pinMosi;
  const int8_t oldNss  = pinNss;
  const int8_t oldDio1 = pinDio1;
  const int8_t oldBusy = pinBusy;
  const int8_t oldRst  = pinRst;
  {
    JsonObject pins = top["pins"];
    getJsonValue(pins[F("SCK")],  pinSck,  RACELINK_PIN_SCK);
    getJsonValue(pins[F("MISO")], pinMiso, RACELINK_PIN_MISO);
    getJsonValue(pins[F("MOSI")], pinMosi, RACELINK_PIN_MOSI);
    getJsonValue(pins[F("NSS")],  pinNss,  RACELINK_PIN_NSS);
    getJsonValue(pins[F("DIO1")], pinDio1, RACELINK_PIN_DIO1);
    getJsonValue(pins[F("BUSY")], pinBusy, RACELINK_PIN_BUSY);
    getJsonValue(pins[F("RST")],  pinRst,  RACELINK_PIN_RST);
  }
  bool pinsChanged = (oldSck  != pinSck)  || (oldMiso != pinMiso) ||
                     (oldMosi != pinMosi) || (oldNss  != pinNss)  ||
                     (oldDio1 != pinDio1) || (oldBusy != pinBusy) ||
                     (oldRst  != pinRst);

  #ifdef RACELINK_EPAPER
    const int8_t oldEpdSck  = epdSck;
    const int8_t oldEpdMiso = epdMiso;
    const int8_t oldEpdMosi = epdMosi;
    const int8_t oldEpdCs   = epdCs;
    const int8_t oldEpdDc   = epdDc;
    const int8_t oldEpdRst  = epdRst;
    const int8_t oldEpdBusy = epdBusy;
    {
      JsonObject ep = top["epaper_pins"];
      getJsonValue(ep[F("SCK")],  epdSck,  RACELINK_EPAPER_SCK);
      getJsonValue(ep[F("MISO")], epdMiso, RACELINK_EPAPER_MISO);
      getJsonValue(ep[F("MOSI")], epdMosi, RACELINK_EPAPER_MOSI);
      getJsonValue(ep[F("CS")],   epdCs,   RACELINK_EPAPER_CS);
      getJsonValue(ep[F("DC")],   epdDc,   RACELINK_EPAPER_DC);
      getJsonValue(ep[F("RST")],  epdRst,  RACELINK_EPAPER_RST);
      getJsonValue(ep[F("BUSY")], epdBusy, RACELINK_EPAPER_BUSY);
    }
    pinsChanged = pinsChanged ||
                  (oldEpdSck  != epdSck)  || (oldEpdMiso != epdMiso) ||
                  (oldEpdMosi != epdMosi) || (oldEpdCs   != epdCs)   ||
                  (oldEpdDc   != epdDc)   || (oldEpdRst  != epdRst)  ||
                  (oldEpdBusy != epdBusy);
  #endif

  if (pinsChanged && !firstReadFromConfig) {
    DEBUG_PRINTLN(F("[RaceLink] Pin config changed — rebooting to apply"));
    doReboot = true;
  }
  firstReadFromConfig = false;

  // RaceLink-authoritative overrides. Slots are always present in cfg.json
  // (addToConfig writes them unconditionally); a missing key on first boot
  // after upgrade falls back to the RACELINK_DEFAULT_* compile constant via
  // the getJsonValue default arg.
  JsonObject ov = top["overrides"];
  getJsonValue(ov[F("Target FPS")],               overrides.fps,          (uint8_t)RACELINK_DEFAULT_FPS);
  getJsonValue(ov[F("ABL Max mA")],               overrides.ablMaxMa,     (uint16_t)RACELINK_DEFAULT_ABL_MAX_MA);
  getJsonValue(ov[F("Default Brightness")],       overrides.briS,         (uint8_t)RACELINK_DEFAULT_BRIS);
  getJsonValue(ov[F("Transition Duration (ms)")], overrides.transitionMs, (uint16_t)RACELINK_DEFAULT_TRANSITION_MS);
  getJsonValue(ov[F("Segment 0 Start LED")], overrides.seg0Start, (uint16_t)RACELINK_DEFAULT_SEG0_START);
  getJsonValue(ov[F("Segment 0 Stop LED")],  overrides.seg0Stop,  (uint16_t)RACELINK_DEFAULT_SEG0_STOP);
  getJsonValue(ov[F("Segment 1 Start LED")], overrides.seg1Start, (uint16_t)RACELINK_DEFAULT_SEG1_START);
  getJsonValue(ov[F("Segment 1 Stop LED")],  overrides.seg1Stop,  (uint16_t)RACELINK_DEFAULT_SEG1_STOP);

  getJsonValue(ov[F("Headless Active")],         overrides.headlessPersistedActive, false);
  getJsonValue(ov[F("Headless Group Counter")],  overrides.headlessGroupCounter,    (uint8_t)0);
  getJsonValue(ov[F("Headless Current Scene")],  overrides.headlessCurrentScene,    (uint8_t)0xFF);
  getJsonValue(ov[F("Headless Broadcast Bri")],  overrides.headlessBroadcastBri,    (uint8_t)128);

  // Default 0xFF on missing key signals "first ever boot" to setup(), which
  // then rolls esp_random() % 3 and forces a re-save.
  getJsonValue(ov[F("Boot Color Mode")], overrides.bootColorMode, (uint8_t)0xFF);
  getJsonValue(ov[F("Boot Color R")],    overrides.bootColorR,    (uint8_t)0);
  getJsonValue(ov[F("Boot Color G")],    overrides.bootColorG,    (uint8_t)0);
  getJsonValue(ov[F("Boot Color B")],    overrides.bootColorB,    (uint8_t)0);

  // Headless slave registry — wipe runtime first so a config reload after a
  // shrink (e.g. WebUI edit removing entries) does not leak stale records.
  // Entries are silently skipped on malformed strings or groupId==0.
  RaceLinkHeadless::clearSlaveTable(headlessSlaves, headlessSlavesCount,
                                    RaceLinkHeadless::HEADLESS_MAX_SLAVES);
  JsonArray slaves = ov[F("Headless Slaves")];
  if (!slaves.isNull()) {
    for (JsonObject o : slaves) {
      if (headlessSlavesCount >= RaceLinkHeadless::HEADLESS_MAX_SLAVES) break;
      String a = o[F("a")] | "";
      uint8_t g = o[F("g")] | 0;
      if (a.length() != 6 || g == 0) continue;
      uint32_t val = strtoul(a.c_str(), nullptr, 16);
      auto& rec = headlessSlaves[headlessSlavesCount];
      rec.addr3[0] = (val >> 16) & 0xFF;
      rec.addr3[1] = (val >> 8)  & 0xFF;
      rec.addr3[2] = (val)       & 0xFF;
      rec.groupId  = g;
      headlessSlavesCount++;
    }
  }

  // WebUI Save path: push the new override values into live WLED globals
  // immediately so a Save is observable without a reboot. Boot path skips
  // this — setup() runs applyRaceLinkDefaults() right after WLED's deserialise
  // anyway, and calling it here would just be duplicate work.
  if (!wasFirstCall) {
    applyRaceLinkDefaults();
  }

  return true;
}

void UsermodRaceLink::onStateChange(uint8_t mode) {
  // Mirror current runtime state for STATUS replies / UI. Picks up changes
  // from the WLED web UI, JSON API, presets being loaded externally, etc.
  current.brightness = bri;

  if (bri > 0) current.flags |= RACELINK_FLAG_POWER_ON;
  else         current.flags &= (uint8_t)~RACELINK_FLAG_POWER_ON;

  // currentPreset is the WLED preset index (0 = none).
  // Do NOT map effectCurrent -> presetId (different concept).
  current.presetId = currentPreset;

  refreshFieldsFromSegment();
}

// Snapshot the WLED main segment into current.fields. Marks every field
// as known (fieldMask=0xFF, extMask=0x0F) because the segment now has a
// defined value for every field we expose.
void UsermodRaceLink::refreshFieldsFromSegment() {
  Segment& seg = strip.getMainSegment();
  current.fields.brightness = bri;
  current.fields.mode       = seg.mode;
  current.fields.speed      = seg.speed;
  current.fields.intensity  = seg.intensity;
  current.fields.custom1    = seg.custom1;
  current.fields.custom2    = seg.custom2;
  current.fields.custom3    = seg.custom3;
  current.fields.check1     = seg.check1;
  current.fields.check2     = seg.check2;
  current.fields.check3     = seg.check3;
  current.fields.palette    = seg.palette;
  current.fields.color1     = seg.colors[0];
  current.fields.color2     = seg.colors[1];
  current.fields.color3     = seg.colors[2];
  current.fields.fieldMask  = 0xFF;
  current.fields.extMask    = 0x0F;
  current.lastUpdateMs      = millis();
}

// ========= Radio =========
bool UsermodRaceLink::radioInit() {
  // Allocate radio pins via WLED's PinManager so a misconfigured LED bus on
  // any of these pins fails loudly here instead of silently breaking SPI.
  // MISO is allowed to be -1 (some modules omit it). Skip it from the alloc
  // if so — PinManager rejects negative pins.
  const PinManagerPinType pinsToAlloc[] = {
    { pinSck,  true  },
    { pinMosi, true  },
    { pinMiso, false },
    { pinNss,  true  },
    { pinDio1, false },
    { pinBusy, false },
    { pinRst,  true  },
  };
  if (!PinManager::allocateMultiplePins(pinsToAlloc,
                                        sizeof(pinsToAlloc) / sizeof(pinsToAlloc[0]),
                                        PinOwner::UM_Unspecified)) {
    DEBUG_PRINTLN(F("[RaceLink] PinManager allocation failed (LED bus conflict?)"));
    return false;
  }

  // SPI bus init using the runtime-configured pins.
  spi->begin(pinSck, pinMiso, pinMosi, pinNss);

  // RadioLib Module(cs, dio1, rst, busy, spi)
  #if defined(RACELINK_SX1262)
  static SX1262 r(new Module(pinNss, pinDio1, pinRst, pinBusy, *spi));
  #elif defined(RACELINK_LLCC68)
  static LLCC68 r(new Module(pinNss, pinDio1, pinRst, pinBusy, *spi));
  #else
  #error "No RaceLink radio module defined"
  #endif
  
  radio = &r;

  // ---- RF config bring-up (NVS-backed, runtime-tunable since Stage 1 PR-3)
  //
  // Boot-loop recovery: tick the counter BEFORE radio.begin(). If
  // we've hit BOOT_COUNTER_MAX strikes the persisted slot is presumed
  // to be bricking the node (e.g. SF too high for the environment, or
  // a freq the master can no longer reach). Wipe the slot now; the
  // next load() returns false and we fall through to compile
  // defaults below.
  const uint8_t bootStrikes = RfConfigNvs::bootCounterIncrement();
  if (bootStrikes > RfConfigNvs::BOOT_COUNTER_MAX) {
    RfConfigNvs::wipe();
  }

  // Load the active config. Order of preference:
  //   1) Validated NVS slot (operator-set via OPC_RF_CONFIG, survives
  //      reboots).
  //   2) Compile-time defaults (RACELINK_FREQ_HZ etc.). On first boot
  //      we additionally persist these defaults so OPC_GET_RF_CONFIG
  //      always returns a complete picture and so the operator can
  //      pivot via a normal SET without first probing for "is anything
  //      there yet".
  if (!RfConfigNvs::load(g_activeRfConfig)) {
    g_activeRfConfig = getCompileDefaultRfConfig();
    RfConfigNvs::store(g_activeRfConfig);
  }

  RaceLinkTransport::PhyCfg phy;
  phy.freqMHz   = (float)g_activeRfConfig.freq_hz / 1e6f;
  phy.bwKHz     = (float)g_activeRfConfig.bw_khz_x10 / 10.0f;
  phy.sf        = g_activeRfConfig.sf;
  phy.crDen     = g_activeRfConfig.cr_den;
  phy.syncWord  = g_activeRfConfig.sync_word;
  phy.preamble  = g_activeRfConfig.preamble;
  phy.crcOn     = true;

  // C3 / HT-CT62 specific:
  phy.txPowerDbm   = g_activeRfConfig.tx_power_dbm;
  phy.dio2RfSwitch = 1;                    // SX1262 (HT-CT62) or also LLCC68 (DreamLNK)
  phy.rxBoost      = -1;                   // or 1/0 depending on board tests

  radioInitCode = RaceLinkTransport::beginCommon(*radio, rl, phy) ? RADIOLIB_ERR_NONE : -999;
  if (radioInitCode != RADIOLIB_ERR_NONE) { radio = nullptr; return false; }

  //rl.radio = radio;

  rl.lbtEnable = true;   // default: false

  RaceLinkTransport::attachDio1(*radio, rl);

  RaceLinkTransport::setDefaultRxContinuous(rl); // *** IMPORTANT: enable continuous RX via RL ***

  // Radio came up cleanly -- reset the strike counter so the next
  // clean boot starts at zero. A hang inside beginCommon() above
  // would leave the counter ticked, and the next boot (after the
  // operator power-cycles) re-enters the recovery branch.
  RfConfigNvs::bootCounterClear();

  return true;
}

bool UsermodRaceLink::senderAllowed(const uint8_t s3[3], uint8_t opcode7) {
  using namespace RaceLinkProto;
  if (!macFilterEnabled) return true;

  if (!masterKnown) {
    // while no master has been learned: only allow Discovery/Grouping
    return (opcode7 == OPC_DEVICES || opcode7 == OPC_SET_GROUP);
  }
  // after that only accept from the learned master
  return RaceLinkTransport::same3(s3, masterLast3);
}

void UsermodRaceLink::learnMasterFromSender(const uint8_t s3[3], bool persistIfEnabled) {
  memcpy(masterLast3, s3, 3);
  masterKnown = true;
  if (persistIfEnabled && macFilterPersist) persistMasterIfNeeded();
  // Pair event also counts as master contact -> trigger the quiet gate.
  noteMasterRx();
}

void UsermodRaceLink::persistMasterIfNeeded() {
  // Snapshot the currently-learned master into the persisted slot and
  // queue a cfg.json save. Callers gate on (persistIfEnabled &&
  // macFilterPersist) so this never fires when the operator has
  // persistence off. addToConfig() reads from the persisted slot, so
  // a same-tick Save (e.g. operator toggling something in WLED Settings)
  // would otherwise round-trip the stale snapshot back to disk.
  if (!masterKnown) return;
  memcpy(persistedMasterLast3, masterLast3, 3);
  if (masterFull6Known) {
    memcpy(persistedMasterFull6, masterFull6, 6);
    persistedMasterFull6Known = true;
  }
  configNeedsWrite = true;
}

void UsermodRaceLink::clearMaster() {
  masterKnown = false;
  memset(masterLast3, 0, 3);
  // In-RAM only by design: the persisted slot (persistedMasterLast3)
  // is owned by addToConfig/readFromConfig and macFilterPersist is the
  // gate that decides whether it's applied at next boot. Clearing
  // live state here doesn't (and shouldn't) touch the persisted value.
}

// ========= RaceLink-authoritative apply =========
// Push every override slot into the live WLED globals, plus enforce the
// non-override fleet-uniformity defaults (gamma, apBehavior). Called from:
//
//   * setup() — once at boot, after WLED's deserialiseConfig has populated
//     globals from cfg.json. RaceLink overrides whatever WLED loaded.
//   * readFromConfig() — after every WebUI Save (and any other re-deserialise),
//     so override changes are live without a reboot.
//   * OPC_CONFIG 0x0F — after the override slots have been reset to
//     RACELINK_DEFAULT_*; this drives the inline "factory reset" apply.
//
// Idempotent: each block compares the current global to the desired value
// and only writes (and logs) on actual divergence. A clean device produces
// a silent boot log; a drifted one prints exactly which setting was corrected.
// `configNeedsWrite=true` queues a cfg.json re-save so on-disk state matches
// what the device is actually rendering.
void UsermodRaceLink::applyRaceLinkDefaults() {
  bool changed = false;

  // ---- Target refresh rate ----------------------------------------------
  if (strip.getTargetFps() != overrides.fps) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing FPS %u (was %u)\n"),
                   (unsigned)overrides.fps, (unsigned)strip.getTargetFps());
    strip.setTargetFps(overrides.fps);
    changed = true;
  }

  // ---- Automatic Brightness Limiter -------------------------------------
  if (BusManager::ablMilliampsMax() != overrides.ablMaxMa) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing ABL %u mA (was %u mA)\n"),
                   (unsigned)overrides.ablMaxMa, (unsigned)BusManager::ablMilliampsMax());
    BusManager::setMilliampsMax(overrides.ablMaxMa);
    changed = true;
  }

  // ---- Gamma correction (non-override fleet-default) ---------------------
  // Three globals (col on/off, bri on/off, val) — recompute the gamma table
  // only if any of them actually changed.
  const bool  defGCol = RACELINK_DEFAULT_GAMMA_COL;
  const bool  defGBri = RACELINK_DEFAULT_GAMMA_BRI;
  const float defGVal = RACELINK_DEFAULT_GAMMA_VAL;
  if (gammaCorrectCol != defGCol || gammaCorrectBri != defGBri || gammaCorrectVal != defGVal) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing Gamma defaults: col=%d bri=%d val=%.2f (was col=%d bri=%d val=%.2f)\n"),
                   (int)defGCol, (int)defGBri, (double)defGVal,
                   (int)gammaCorrectCol, (int)gammaCorrectBri, (double)gammaCorrectVal);
    gammaCorrectCol = defGCol;
    gammaCorrectBri = defGBri;
    gammaCorrectVal = defGVal;
    NeoGammaWLEDMethod::calcGammaTable(gammaCorrectVal);
    changed = true;
  }

  // ---- AP open behaviour (non-override fleet-default) -------------------
  // RaceLink fleets keep the WiFi AP closed by default — operators reach a
  // node via the triple-tap recovery gesture (which calls initAP(true)
  // directly, bypassing apBehavior). Without this enforcement, a factory-
  // reset device falls back to WLED's default AP_BEHAVIOR_BOOT_NO_CONN (0)
  // and auto-opens its AP on every boot when no STA is configured — chatty
  // RF, breaks the "node is silent until I want to talk to it" property.
  if (apBehavior != (uint8_t)RACELINK_DEFAULT_AP_BEHAVIOR) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing apBehavior default %u (was %u)\n"),
                   (unsigned)RACELINK_DEFAULT_AP_BEHAVIOR, (unsigned)apBehavior);
    apBehavior = RACELINK_DEFAULT_AP_BEHAVIOR;
    changed = true;
  }

  // ---- Segment 0 geometry -----------------------------------------------
  if (strip.getSegmentsNum() >= 1) {
    Segment& s0 = strip.getSegment(0);
    if (s0.start != overrides.seg0Start || s0.stop != overrides.seg0Stop) {
      DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing seg[0] geometry %u..%u (was %u..%u)\n"),
                     (unsigned)overrides.seg0Start, (unsigned)overrides.seg0Stop,
                     (unsigned)s0.start, (unsigned)s0.stop);
      s0.setGeometry(overrides.seg0Start, overrides.seg0Stop);
      changed = true;
    }
  }

  // ---- Segment 1 geometry -----------------------------------------------
  // Sentinel: start == stop means "seg[1] not configured" — leave whatever
  // is already there alone (creating an empty seg[1] would just waste a
  // slot in WLED's segment vector).
  if (overrides.seg1Start != overrides.seg1Stop) {
    if (strip.getSegmentsNum() <= 1) {
      strip.appendSegment(overrides.seg1Start, overrides.seg1Stop);
      DEBUG_PRINTF_P(PSTR("[RaceLink] appended seg[1] %u..%u\n"),
                     (unsigned)overrides.seg1Start, (unsigned)overrides.seg1Stop);
      changed = true;
    }
    if (strip.getSegmentsNum() > 1) {
      Segment& s1 = strip.getSegment(1);
      if (s1.start != overrides.seg1Start || s1.stop != overrides.seg1Stop) {
        DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing seg[1] geometry %u..%u (was %u..%u)\n"),
                       (unsigned)overrides.seg1Start, (unsigned)overrides.seg1Stop,
                       (unsigned)s1.start, (unsigned)s1.stop);
        s1.setGeometry(overrides.seg1Start, overrides.seg1Stop);
        changed = true;
      }
    }
  }

  // ---- Default brightness (briS) ----------------------------------------
  // Affects the SAVED brightness (boot value, restore target) — the LIVE
  // brightness `bri` is driven by SYNC/CONTROL packets and is not touched
  // here. OPC_CONFIG 0x09 explicitly snaps `bri` for a UX-immediate update.
  if (briS != overrides.briS) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing briS %u (was %u)\n"),
                   (unsigned)overrides.briS, (unsigned)briS);
    briS = overrides.briS;
    changed = true;
  }

  // ---- Transition duration ----------------------------------------------
  if (transitionDelayDefault != overrides.transitionMs) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] enforcing transitionDelayDefault %u (was %u)\n"),
                   (unsigned)overrides.transitionMs, (unsigned)transitionDelayDefault);
    transitionDelayDefault = overrides.transitionMs;
    changed = true;
  }

  // Persist via WLED's canonical deferred-save mechanism: the main loop
  // calls serializeConfigToFS() in its next iteration when configNeedsWrite
  // is set. serializeConfig() reads the live runtime globals we just wrote,
  // so cfg.json ends up reflecting the RaceLink-enforced values. UI / JSON
  // API will therefore agree with what the device is actually rendering.
  if (changed) {
    DEBUG_PRINTLN(F("[RaceLink] cfg.json scheduled for re-save with enforced overrides"));
    configNeedsWrite = true;
    stateUpdated(CALL_MODE_NO_NOTIFY);
  }
}

// ========= Direct-effect visualisations =========

void UsermodRaceLink::showPairConfirmedEffect() {
  // White breathe, persistent. Replaces the old applyPreset(11, ...) — no
  // preset slot is reserved anymore, operator can freely assign all slots.
  Segment& seg = strip.getMainSegment();
  seg.setMode(FX_MODE_BREATH);
  seg.speed     = 128;
  seg.intensity = 128;
  seg.setColor(0, RGBW32(255, 255, 255, 0));   // white foreground
  seg.setColor(1, 0);                           // black background
  bri = 128;
  stateChanged = true;
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

void UsermodRaceLink::applyBootColor() {
  // Solid with the persisted boot color. Mode 0/1/2 paints a primary via
  // applyCycleColor(); mode 3 reuses the stored RGB triple verbatim so a
  // random cycle position survives reboot exactly. Called at boot (when
  // bootPreset == 0) and from the SCENE_RESTORE_BOOT_COLOR handler — both
  // need to land on the same visual without re-rolling.
  if (bri == 0) bri = briS ? briS : 128;

  if (btn.bootColorMode < 3) {
    // applyCycleColor mirrors mode + RGB back into btn.*; for the primary
    // case that's a no-op rewrite. It also drives the segment + stateUpdated
    // so we don't have to duplicate that here.
    applyCycleColor(btn.bootColorMode);
    // Seed the ring-buffer cycle: boot already shows one primary, the next
    // click jumps to (mode+1) % 3. Guarantees all three primaries reachable
    // within two more clicks before switching to random.
    btn.primariesShownCount = 1;
    btn.nextPrimaryIdx      = (uint8_t)((btn.bootColorMode + 1) % 3);
  } else {
    // Mode 3: paint the stored RGB directly. Cannot go through applyCycleColor
    // (idx>=3 would re-roll a fresh random there).
    colPri[0] = btn.bootColorRgb[0];
    colPri[1] = btn.bootColorRgb[1];
    colPri[2] = btn.bootColorRgb[2];
    colPri[3] = 0;
    Segment& seg = strip.getMainSegment();
    seg.setMode(FX_MODE_STATIC);
    seg.setColor(0, RGBW32(colPri[0], colPri[1], colPri[2], 0));
    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
    // Cycle is "full" — next click triggers the idle-reset re-seed path in
    // applyColorCycleStep (provided RACELINK_BTN_COLOR_RESET_MS has elapsed).
    btn.primariesShownCount = 3;
    btn.nextPrimaryIdx      = 0;
  }
  // applyBootColor is NOT an operator action — do not arm bootColorSavePending.
  // applyCycleColor sets the runtime mirror but the persisted value is already
  // in sync (we just read it).
  btn.bootColorSavePending = false;
}

// ========= Single-click color cycle =========

void UsermodRaceLink::applyCycleColor(uint8_t idx) {
  if (idx == 0)      { colPri[0] = 255; colPri[1] = 0;   colPri[2] = 0;   colPri[3] = 0; }
  else if (idx == 1) { colPri[0] = 0;   colPri[1] = 255; colPri[2] = 0;   colPri[3] = 0; }
  else if (idx == 2) { colPri[0] = 0;   colPri[1] = 0;   colPri[2] = 255; colPri[3] = 0; }
  else               { setRandomColor(colPri); colPri[3] = 0; }

  Segment& seg = strip.getMainSegment();
  seg.setMode(FX_MODE_STATIC);
  seg.setColor(0, RGBW32(colPri[0], colPri[1], colPri[2], 0));

  // Mirror the just-painted color into the boot-color runtime state so that
  // serviceBootColorSave() can persist whatever the operator is currently
  // looking at. Primary picks collapse to mode 0/1/2; any random pick lands
  // on mode 3 with the actual RGB captured verbatim.
  btn.bootColorMode   = (idx < 3) ? idx : 3;
  btn.bootColorRgb[0] = colPri[0];
  btn.bootColorRgb[1] = colPri[1];
  btn.bootColorRgb[2] = colPri[2];

  stateChanged = true;
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

void UsermodRaceLink::applyColorCycleStep(uint32_t now) {
  // User clicked on a slave with no master talking — they're driving the
  // local color cycle, so any pending indicator overlay should yield to
  // the click instead of restoring underneath it later.
  cancelIndicator();
  // Idle reset: after >RESET_MS without a single click, the cycle is re-seeded
  // with a new random start index from {0,1,2}. count=0 means "no primary
  // color shown yet in this cycle round" — so the next three clicks deliver
  // all three primaries before switching back to random. uint32 subtraction
  // handles millis() wrap correctly via underflow.
  if ((now - btn.lastColorClickMs) > RACELINK_BTN_COLOR_RESET_MS) {
    btn.primariesShownCount = 0;
    btn.nextPrimaryIdx      = (uint8_t)(esp_random() % 3);
  }

  if (btn.primariesShownCount < 3) {
    // Ring-buffer step: show nextPrimaryIdx, advance via (idx+1) % 3.
    applyCycleColor(btn.nextPrimaryIdx);
    btn.primariesShownCount++;
    btn.nextPrimaryIdx = (uint8_t)((btn.nextPrimaryIdx + 1) % 3);
  } else {
    // All three primary colors shown in this round -> random.
    applyCycleColor(3);
  }

  btn.lastColorClickMs = now;
  // Operator just changed the color — arm the deferred save. serviceBootColorSave()
  // will fire it once the click-burst has been quiet for RACELINK_BTN_COLOR_RESET_MS,
  // giving the operator a window to keep clicking without an interim flash-write.
  btn.bootColorSavePending = true;
  stateChanged = true;
  colorUpdated(CALL_MODE_BUTTON);
}

void UsermodRaceLink::serviceBootColorSave(uint32_t now) {
  // Edge-trigger: armed by applyColorCycleStep(), cleared here after the
  // post-click idle window elapses. lastColorClickMs == 0 means "no click in
  // this session yet" — applyBootColor() resets the pending flag at boot, so
  // hitting this branch would mean something armed without setting the
  // timestamp; skip rather than write stale state.
  if (!btn.bootColorSavePending) return;
  if (btn.lastColorClickMs == 0) return;
  if ((now - btn.lastColorClickMs) <= RACELINK_BTN_COLOR_RESET_MS) return;

  overrides.bootColorMode = btn.bootColorMode;
  overrides.bootColorR    = btn.bootColorRgb[0];
  overrides.bootColorG    = btn.bootColorRgb[1];
  overrides.bootColorB    = btn.bootColorRgb[2];
  configNeedsWrite        = true;
  btn.bootColorSavePending = false;
}

// ========= Master-contact gate =========

bool UsermodRaceLink::masterContactedRecently() const {
  if (!anyMasterRxSinceBoot) return false;
  return (millis() - lastMasterRxMs) < RACELINK_MASTER_QUIET_MS;
}

void UsermodRaceLink::noteMasterRx() {
  lastMasterRxMs = millis();
  anyMasterRxSinceBoot = true;
  // Headless promotion safety: ANY accepted master-side packet (or pairing
  // event) during our probe window proves a master is alive on this channel.
  // Refuse the promotion — the operator gets a red blink from
  // serviceHeadless() on its next tick. Covers both the explicit
  // OPC_SET_GROUP response and incidental traffic from a Gateway already
  // running other slaves.
  if (headless.probing) {
    headless.probeAborted = true;
  }
}

// ========= Custom button (GPIO 0) =========

bool UsermodRaceLink::handleButton(uint8_t b) {
  // Override hook: only intercept Button 0 (GPIO 0) so WLED's default
  // mapping (toggleOnOff/setRandomColor/AP-on-5s) does NOT fire. The actual
  // state machine is driven from loop() via pollPhysicalButton() — this hook
  // exists solely for suppression.
  if (b != 0) return false;
  if (b >= buttons.size()) return false;
  if (buttons[b].pin < 0) return false;
  if (buttons[b].type != BTN_TYPE_PUSH) return false;
  return true; // skip default handling in button.cpp:290+
}

void UsermodRaceLink::pollPhysicalButton(uint32_t now) {
  if (buttons.empty()) return;
  if (buttons[0].pin < 0) return;
  if (buttons[0].type != BTN_TYPE_PUSH) return;

  bool pressed = (digitalRead(buttons[0].pin) == LOW); // active-low (INPUT_PULLUP)
  handleRaceLinkButton(0, pressed, now);
}

void UsermodRaceLink::handleRaceLinkButton(uint8_t /*b*/, bool pressed, uint32_t now) {
  // -- Rising edge --
  if (pressed && !btn.down) {
    btn.down = true;
    btn.pressedAtMs = now;
    btn.longHandled = false;
    return;
  }

  // -- Held: long-press -> start brightness fade --
  if (pressed && btn.down) {
    if (!btn.longHandled && (now - btn.pressedAtMs) >= RACELINK_BTN_LONG_MS) {
      btn.longHandled = true;
      btn.briDirUp    = !btn.briDirUp;       // toggle direction on every hold
      btn.lastFadeTickMs = now;
      btn.pendingShortClicks = 0;             // hold is explicitly not a multi-tap
    }
    return;
  }

  // -- Falling edge --
  if (!pressed && btn.down) {
    btn.down = false;
    uint32_t dur = now - btn.pressedAtMs;

    if (btn.longHandled) {
      // Hold ended -> the fade ran during the hold via applyFinalBri()
      // (without notifications). Call stateUpdated() once now so WS/MQTT/UDP
      // clients see the final bri value.
      btn.longHandled = false;
      stateUpdated(CALL_MODE_BUTTON);

      // Headless: broadcast the final brightness exactly once on release.
      // Avoids flooding the LoRa channel with per-tick updates and lets
      // the operator dial in min/max comfortably with no mid-fade visual
      // drift on the slaves — the fleet snaps to the final value when
      // the operator releases the button.
      if (headless.active && rl.macReadOK) {
        uint8_t out[16];
        uint8_t n = RaceLinkHeadless::buildBrightnessPacket(out, rl.myLast3, bri);
        // armBlip=false: brightness broadcast is routine traffic, not pairing.
        if (n) headlessSendTx(out, n, /*armBlip=*/false);
        overrides.headlessBroadcastBri = bri;
        headless.broadcastBri          = bri;
        headless.lastBroadcastAtMs     = now;
        configNeedsWrite = true;  // persist final brightness across reboot
      }
      return;
    }

    if (dur < 50) return; // Debounce (matches WLED_DEBOUNCE_THRESHOLD)

    // short click -> accumulate into multi-tap window
    if (btn.pendingShortClicks < 255) btn.pendingShortClicks++;
    btn.lastShortReleaseMs = now;
    return;
  }

  // -- Idle: multi-tap window is evaluated once the pause >= MULTI_WINDOW --
  if (!pressed && !btn.down && btn.pendingShortClicks > 0) {
    if ((now - btn.lastShortReleaseMs) >= RACELINK_BTN_MULTI_WINDOW_MS) {
      uint8_t clicks = btn.pendingShortClicks;
      btn.pendingShortClicks = 0;

      // Click-count dispatch. Order matters: 5-click is the headless
      // toggle and must beat the AP recovery branch. 4-click and 6+-click
      // are explicit typo guards — a slipped 4-tap or an over-shot 6-tap
      // must NOT accidentally open the AP. AP recovery is bound to
      // exactly 3 clicks (was clicks >= 3, which made 6-click open the AP
      // instead of toggling headless — confused operators field-testing
      // the 5-click toggle).
      if (clicks == 5) {
        tryStartHeadless();
        return;
      }
      if (clicks == 4 || clicks >= 6) {
        // Typo guard — no action.
        return;
      }
      if (clicks == 3) {
        // Hotspot recovery — ALWAYS, independent of the master-quiet-gate.
        WLED::instance().initAP(true);
        return;
      }
      // clicks == 2 -> intentionally no action (reserved for future use).
      if (clicks == 1) {
        if (headless.active) {
          // Headless Master: single-click advances the broadcast scene.
          // No master-quiet gate — this device IS the master.
          headlessAdvanceScene();
        } else if (!masterContactedRecently()) {
          // Cycle color through R -> G -> B -> random cycle. Idle reset (>10s
          // since last click) falls back to R.
          applyColorCycleStep(now);
        }
      }
    }
  }
}

void UsermodRaceLink::serviceButtonFade(uint32_t now) {
  if (!btn.down || !btn.longHandled) return;
  // Master-quiet gate: lock out if the paired master speaks during a hold.
  // The headless master clears masterKnown on entry (see enterHeadlessMode),
  // so masterContactedRecently() stays false for its entire session — no
  // extra headless-specific qualifier needed here.
  if (masterContactedRecently()) return;
  if ((now - btn.lastFadeTickMs) < RACELINK_BTN_FADE_TICK_MS) return;

  btn.lastFadeTickMs = now;

  // Non-linear step ("S-curve"): smaller deltas near the limits so the
  // operator can dial in min/max comfortably, larger deltas in the middle
  // so a full sweep still completes in a usable ~4 s. Total ticks =
  // 32×1 + 32×2 + 32×1 + 32×2 + 32×1 ≈ 128 × 30 ms ≈ 3.8 s for a full
  // 0→255→0 ping-pong leg. Tunable by editing the band breakpoints below.
  uint8_t briStep;
  if      (bri < 32 || bri > 223)   briStep = 1;
  else if (bri < 64 || bri > 191)   briStep = 2;
  else                              briStep = 4;

  // Ping-pong: invert direction at the limits so the fade keeps running
  // continuously as long as the button is held.
  int next = (int)bri;
  if (btn.briDirUp) {
    next += briStep;
    if (next >= 255) { next = 255; btn.briDirUp = false; }
  } else {
    next -= briStep;
    if (next <= 1)   { next = 1;   btn.briDirUp = true; } // not fully off -> avoid offMode
  }
  if ((uint8_t)next == bri) return;
  bri = (uint8_t)next;

  // applyFinalBri() bypasses WLED's brightness transition entirely (briOld=briT=
  // bri + applyBri + strip.trigger). Without this, the transition keeps running
  // for transitionDelay ms after release and the fade feels extremely sluggish.
  applyFinalBri();

  // No headless brightness broadcast here — the fade is local-only during
  // the hold. Sending per-tick would flood the single-slot TX queue
  // (~30 ms tick vs LBT backoff + ToA). The final value is broadcast once
  // on button release; see handleRaceLinkButton's falling-edge block.
}

bool UsermodRaceLink::handleStreamPacket(const uint8_t* buf, uint8_t len, const uint8_t senderLast3[3]) {
  using namespace RaceLinkProto;
  if (len != (sizeof(Header7) + sizeof(P_Stream))) return false;

  P_Stream p{};
  if (!parseBody(buf, len, p)) return false;

  const RaceLinkTransport::StreamStatus status = RaceLinkTransport::handleStreamPacket(rl, p);
  if (status != RaceLinkTransport::StreamStatus::StreamEnd) return false;

  sendAckTo(senderLast3, OPC_STREAM, ACK_OK);

  uint8_t streamLen = 0;
  const uint8_t* streamData = RaceLinkTransport::streamBuffer(rl, streamLen);
  captureLastStreamPacket(streamData, streamLen);
  StartblockMsgV1 startblock{};
  if (parseStartblockV1(streamData, streamLen, startblock)) {
    char nameBuf[STREAM_BUFFER_SIZE];
    size_t nameLen = startblock.name_len;
    if (nameLen >= sizeof(nameBuf)) nameLen = sizeof(nameBuf) - 1;
    memcpy(nameBuf, startblock.name_ptr, nameLen);
    nameBuf[nameLen] = '\0';

    char logBuf[160];
    snprintf(logBuf, sizeof(logBuf),
      "[RaceLink] STREAM Startblock v1 slot %u chan %s name %s",
      startblock.slot, startblock.chan, nameBuf);
    DEBUG_PRINTLN(logBuf);
#ifdef RACELINK_EPAPER
    bool slotValid = true;
    uint8_t displaySlot = startblock.slot;
    #if DEV_TYPE == 50
      const uint8_t slotCount = constrain(numberOfSlots, (uint8_t)1, (uint8_t)8);
      const uint8_t slotFirst = constrain(firstSlot, (uint8_t)1, (uint8_t)8);
      if (startblock.slot < slotFirst || startblock.slot >= (uint8_t)(slotFirst + slotCount)) {
        slotValid = false;
      } else {
        displaySlot = startblock.slot - slotFirst + 1;
      }
    #endif
    if (slotValid && displaySlot <= 4) {
      setPilotSlotData(nameBuf, startblock.chan, displaySlot);
    }
#endif
  } else {
    DEBUG_PRINTLN(F("[RaceLink] STREAM Startblock v1 parse failed"));
  }

  RaceLinkTransport::clearStreamReady(rl);
  return true;
}

void UsermodRaceLink::handlePacket(const uint8_t* buf, size_t len) {
  using namespace RaceLinkProto;

  if (len < sizeof(Header7)) return;

  Header7 h{};
  if (!parseHeader(buf, (uint8_t)len, h)) return;
  debugCounter=1;
  // DIAGNOSTIC (Headless re-bind investigation 2026-05-18): one compact line
  // per RX-after-header-parse showing addressing + raw type. Remove once the
  // re-bind investigation is closed.
  DEBUG_PRINTF_P(PSTR("[RaceLink] RX hdr sender=%02X%02X%02X recv=%02X%02X%02X type=0x%02X len=%u\n"),
                 h.sender[0], h.sender[1], h.sender[2],
                 h.receiver[0], h.receiver[1], h.receiver[2],
                 (unsigned)h.type, (unsigned)len);
  if (!RaceLinkTransport::receiverMatches(h.receiver, rl.myLast3)) {
    DEBUG_PRINTLN(F("[RaceLink] RX -> receiverMatches REJECT"));
    return;  // broadcast OR exactly my 3B
  }
  debugCounter=2;

  // Headless Master: accept the otherwise-rejected N2M IDENTIFY_REPLY
  // broadcasts so we can auto-pair arriving nodes. Runs BEFORE the DIR_M2N
  // filter (a slave would normally drop N2M packets). The headless master
  // is the only node that needs to listen for them.
  if (headless.active && type_dir(h.type) == DIR_N2M
      && type_base(h.type) == OPC_DEVICES) {
    P_IdentifyReply ir{};
    if (parseBody(buf, (uint8_t)len, ir)) {
      headlessAssignGroupTo(h.sender, ir.groupId);
      rxAccepted++;
    }
    return;
  }

  // Headless Master: also accept N2M OPC_ACK so the re-bind sweep is
  // observable. Slaves respond with OPC_ACK after applying an OPC_SET_GROUP;
  // without this branch the ACK falls through the DIR_M2N filter (slaves'
  // default drop) and we have no operator-visible confirmation that the
  // pairing landed. Logged-only — no visual indicator by user choice.
  if (headless.active && type_dir(h.type) == DIR_N2M
      && type_base(h.type) == OPC_ACK) {
    P_Ack ack{};
    if (parseBody(buf, (uint8_t)len, ack)) {
      DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: RX ACK from %02X%02X%02X echoOpcode=0x%02X status=%u\n"),
                     h.sender[0], h.sender[1], h.sender[2],
                     (unsigned)ack.echo_opcode7, (unsigned)ack.status);
      rxAccepted++;
    }
    return;
  }

  // Master-alive detector: any M2N packet that survives the receiverMatches
  // gate and was NOT sent by us proves a master is talking on the channel.
  // Runs BEFORE senderAllowed so we catch packets regardless of the local
  // MAC-filter state — e.g. a Gateway's periodic OPC_SYNC autosync that
  // would otherwise be dropped because masterKnown is false after a
  // power-cycle. The user-stated principle is "a real Gateway always wins
  // over headless mode", so:
  //   probing -> mark probeAborted; serviceHeadless red-blinks and refuses
  //              promotion on its next tick.
  //   active  -> step down immediately. Re-arming headless is a deliberate
  //              5-click away once the gateway is gone.
  // We never react to our own broadcasts (we don't receive them on the
  // radio anyway, but the same3-against-self check is defensive). Normal
  // dispatch continues below so e.g. an OPC_SET_GROUP can still finish the
  // re-pair on the same packet.
  if (type_dir(h.type) == DIR_M2N
      && !RaceLinkTransport::same3(h.sender, rl.myLast3)) {
    if (headless.probing) {
      headless.probeAborted = true;
    } else if (headless.active) {
      exitHeadlessMode();
    }
  }

  if (type_dir(h.type) != DIR_M2N) {
    DEBUG_PRINTLN(F("[RaceLink] RX -> direction REJECT (not DIR_M2N)"));
    return;                       // only Master->Node requests here
  }
  debugCounter=3;
  const uint8_t opcode7 = type_base(h.type);

  // MAC filter (optional, as before)
  if (!senderAllowed(h.sender, opcode7)) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] RX -> senderAllowed REJECT opcode=0x%02X macFilterEnabled=%u masterKnown=%u stored=%02X%02X%02X\n"),
                   (unsigned)opcode7, (unsigned)macFilterEnabled, (unsigned)masterKnown,
                   masterLast3[0], masterLast3[1], masterLast3[2]);
    return;
  }
  debugCounter=4;

  // Master quiet gate: every accepted packet from the already-paired master
  // resets the session timer. Pairing events (OPC_SET_GROUP from an unknown
  // sender) are handled separately in learnMasterFromSender.
  if (masterKnown && RaceLinkTransport::same3(h.sender, masterLast3)) {
    noteMasterRx();
  }

  // Group logic: applies to every request that carries a groupId
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
      DEBUG_PRINTLN(F("[RaceLink] GET_DEVICES -> schedule IDENTIFY_REPLY"));
    } break;

    case OPC_SET_GROUP: { // SET_GROUP -> apply + ACK
      // DIAGNOSTIC (Headless re-bind investigation 2026-05-18): every
      // SET_GROUP that survives sender/receiver/direction gates lands here.
      // Remove once the re-bind issue is closed.
      DEBUG_PRINTF_P(PSTR("[RaceLink] RX OPC_SET_GROUP sender=%02X%02X%02X recv=%02X%02X%02X len=%u\n"),
                     h.sender[0], h.sender[1], h.sender[2],
                     h.receiver[0], h.receiver[1], h.receiver[2], (unsigned)len);
      P_SetGroup p{};
      if (!parseBody(buf, (uint8_t)len, p)) {
        DEBUG_PRINTLN(F("[RaceLink] RX OPC_SET_GROUP -> parseBody REJECT (length mismatch)"));
        break;
      }
      DEBUG_PRINTF_P(PSTR("[RaceLink] RX OPC_SET_GROUP body groupId=%u (was=%u) headless.active=%u\n"),
                     (unsigned)p.groupId, (unsigned)current.groupId, (unsigned)headless.active);

      // Headless takeover by a real Gateway: a foreign master is claiming
      // us, so step down from Headless Master and accept the new pairing
      // like any other slave. ``exitHeadlessMode`` clears the persisted-
      // active flag so we will not re-promote at the next reboot — the
      // operator can re-enable headless via 5-click whenever desired.
      // (probeAborted for the probing case is set in noteMasterRx().)
      if (headless.active) {
        exitHeadlessMode();
      }

      // as before: group 0 allows setting / or "255" special-case logic...
      current.groupId = p.groupId;

      learnMasterFromSender(h.sender, /*persistIfEnabled*/true);

      // Visual feedback via the central indicator system: 3-second white
      // breathe overlay, then auto-restore to whatever was on the strip
      // before (boot-random for fresh slaves, last scene for re-pair).
      // Replaces the previous persistent showPairConfirmedEffect() call.
      applyLocalIndicator(RaceLinkIndicators::IND_PAIR_CONFIRMED, 5);

      sendAckTo(h.sender, OPC_SET_GROUP, ACK_OK);
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] SET_GROUP -> applied + ACK"));
    } break;

    case OPC_PRESET: { // PRESET: preset config (arm + optional flags/brightness)
      P_Preset p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;
      // Offset acceptance gate: drop if OFFSET_MODE flag mismatches the
      // device's *effective* mode (pending if set, else active). PRESET
      // applies immediately, so on accept we materialise pending here too.
      if (!offsetGateAccepts(p.flags)) {
        DEBUG_PRINTLN(F("[RaceLink] PRESET -> dropped by offset gate"));
        break;
      }
      materialisePendingChange();

      handlePreset(p);
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] PRESET -> configured"));
    } break;

    case OPC_CONTROL: { // CONTROL: direct effect-parameter remote control (variable-length body)
      // decide_response() skipped the length check (req_len=0). Enforce bounds here.
      if (bodyLen < 3 || bodyLen > MAX_P_CONTROL) break;
      const uint8_t* body = buf + sizeof(Header7);
      uint8_t grp = 0, fl = 0;
      AdvancedFields f{};
      if (!parseControlBody(body, bodyLen, grp, fl, f)) break;
      if (!groupMatch(grp)) break;
      // Offset acceptance gate: drop if OFFSET_MODE flag mismatches the
      // effective mode (see racelink_wled.h, P_Offset comment in
      // racelink_proto.h). For arm-on-sync packets, materialisation defers
      // to the SYNC handler so a queued effect inherits the new mode/offset
      // exactly when it fires; for immediate-apply packets we materialise
      // now (in handleControl) so the new active_mode applies right away.
      if (!offsetGateAccepts(fl)) {
        DEBUG_PRINTLN(F("[RaceLink] CONTROL -> dropped by offset gate"));
        break;
      }

      handleControl(fl, f);
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] CONTROL -> configured"));
    } break;

    case OPC_SYNC: { // SYNC pulse (global, variable 4..5 B body)
      // RULES has req_len=0 for OPC_SYNC; validate length inline.
      // 4 B = legacy clock-tick (no flags), 5 B = trailing flags byte.
      // Bit 0 of flags (SYNC_FLAG_TRIGGER_ARMED) gates pending arm-on-sync
      // materialisation; without it, only the timebase is adjusted.
      if (bodyLen < 4 || bodyLen > 5) break;
      const uint8_t* body = buf + sizeof(Header7);
      const uint32_t ts24 = ((uint32_t)body[0])
                          | ((uint32_t)body[1] << 8)
                          | ((uint32_t)body[2] << 16);
      const uint8_t brightness = body[3];
      const uint8_t syncFlags  = (bodyLen >= 5) ? body[4] : 0;
      handleSync(ts24, brightness, syncFlags);
      acted = true;
      //DEBUG_PRINTLN(F("[RaceLink] SYNC -> processed"));
    } break;

    case OPC_OFFSET: { // variable-length offset config (2..7 B body)
      // decide_response() skipped the length check (req_len=0). Bounds and
      // mode-specific payload sizes are validated inside handleOffset().
      if (bodyLen < 2 || bodyLen > MAX_P_OFFSET) break;
      const uint8_t* body = buf + sizeof(Header7);
      if (!handleOffset(body, bodyLen)) break;
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] OFFSET -> pending update"));
    } break;

    case OPC_CONFIG: {
      RaceLinkProto::P_Config p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      //if (!groupMatch(p.groupId)) break;
      if (RaceLinkTransport::isBroadcast3(h.receiver)) return;  // only unicast allowed for config

      sendAckTo(h.sender, OPC_CONFIG, ACK_OK); // ACK first because some options may take time
      acted = true;

      if (p.option == 0x01) { // MAC Filter Enable/Disable
        macFilterEnabled = (p.data0 != 0);
      } else if (p.option == 0x02) { // Clear learned Master
        clearMaster();
      } else if (p.option == 0x03) { // MAC Filter Persist Enable/Disable
        const bool wasOn = macFilterPersist;
        macFilterPersist = (p.data0 != 0);
        // On the off → on edge, snapshot whatever master is currently
        // learned so the persisted slot reflects the operator's intent
        // ("pin this binding"). If nothing is learned yet, leave the
        // existing snapshot intact — the operator may have pinned a
        // previous master that they want to re-arm at the next boot.
        if (!wasOn && macFilterPersist) {
          persistMasterIfNeeded();  // copies masterLast3/Full6 + queues save
        } else {
          // Toggle alone still needs persisting so cfg.json reflects
          // the new macFilterPersist value at next boot.
          configNeedsWrite = true;
        }
      } else if (p.option == 0x04) { // Enable AP Mode
        if (p.data0 != 0) WLED::instance().initAP(true);
        else {
          dnsServer.stop();
          WiFi.softAPdisconnect(true);
          apActive = false;
        }
      } else if (p.option == 0x05) { // Set FPS override
        overrides.fps = p.data0;
        strip.setTargetFps(overrides.fps);
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: FPS set to %u\n"), (unsigned)overrides.fps);
      } else if (p.option == 0x06) { // Set Segment 0 geometry
        overrides.seg0Start = (uint16_t)p.data0 | ((uint16_t)p.data1 << 8);
        overrides.seg0Stop  = (uint16_t)p.data2 | ((uint16_t)p.data3 << 8);
        if (strip.getSegmentsNum() >= 1) {
          strip.getSegment(0).setGeometry(overrides.seg0Start, overrides.seg0Stop);
        }
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: seg[0] geometry set to %u..%u\n"),
                       (unsigned)overrides.seg0Start, (unsigned)overrides.seg0Stop);
      } else if (p.option == 0x07) { // Set Segment 1 geometry
        overrides.seg1Start = (uint16_t)p.data0 | ((uint16_t)p.data1 << 8);
        overrides.seg1Stop  = (uint16_t)p.data2 | ((uint16_t)p.data3 << 8);
        // Append seg[1] if missing (no-op if already present or max segments reached)
        if (strip.getSegmentsNum() <= 1) {
          strip.appendSegment(overrides.seg1Start, overrides.seg1Stop);
        }
        if (strip.getSegmentsNum() > 1) {
          strip.getSegment(1).setGeometry(overrides.seg1Start, overrides.seg1Stop);
        }
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: seg[1] geometry set to %u..%u\n"),
                       (unsigned)overrides.seg1Start, (unsigned)overrides.seg1Stop);
      } else if (p.option == 0x08) { // Set ABL max mA override
        overrides.ablMaxMa = (uint16_t)p.data0 | ((uint16_t)p.data1 << 8);
        BusManager::setMilliampsMax(overrides.ablMaxMa);
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: ABL set to %u mA\n"), (unsigned)overrides.ablMaxMa);
      } else if (p.option == 0x09) { // Set default brightness override
        overrides.briS = p.data0;
        briS = overrides.briS;
        // Snap LIVE brightness too so the strip visibly updates without a
        // reboot — operators editing the row in the host dialog expect
        // immediate feedback. The live value can still be overridden later
        // by OPC_CONTROL/OPC_SYNC; this is just the at-save snapshot.
        bri = overrides.briS;
        stateUpdated(CALL_MODE_NO_NOTIFY);
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: briS set to %u (live bri synced)\n"),
                       (unsigned)overrides.briS);
      } else if (p.option == 0x0A) { // Set transition duration override
        overrides.transitionMs = (uint16_t)p.data0 | ((uint16_t)p.data1 << 8);
        transitionDelayDefault = overrides.transitionMs;
        configNeedsWrite = true;
        DEBUG_PRINTF_P(PSTR("[RaceLink] OPC_CONFIG: transitionDelayDefault set to %u ms\n"),
                       (unsigned)overrides.transitionMs);
      } else if (p.option == 0x0F) { // Reset every override slot to RACELINK_DEFAULT_*
        overrides.fps          = (uint8_t)RACELINK_DEFAULT_FPS;
        overrides.ablMaxMa     = (uint16_t)RACELINK_DEFAULT_ABL_MAX_MA;
        overrides.seg0Start    = (uint16_t)RACELINK_DEFAULT_SEG0_START;
        overrides.seg0Stop     = (uint16_t)RACELINK_DEFAULT_SEG0_STOP;
        overrides.seg1Start    = (uint16_t)RACELINK_DEFAULT_SEG1_START;
        overrides.seg1Stop     = (uint16_t)RACELINK_DEFAULT_SEG1_STOP;
        overrides.briS         = (uint8_t)RACELINK_DEFAULT_BRIS;
        overrides.transitionMs = (uint16_t)RACELINK_DEFAULT_TRANSITION_MS;

        // resetSegments() rebuilds the segment vector to WLED's default
        // (one segment spanning the full strip). Unsafe during
        // strip.service() — guard with suspend/resume. applyRaceLinkDefaults()
        // below then re-asserts our seg0 sentinel (stop==0 → full strip),
        // which is a no-op against this fresh state.
        strip.suspend();
        strip.resetSegments();
        strip.resume();

        // Snap LIVE brightness too (same reasoning as 0x09).
        bri = (uint8_t)RACELINK_DEFAULT_BRIS;

        // applyRaceLinkDefaults() handles FPS/ABL/briS/transition/seg
        // enforcement + stateUpdated + configNeedsWrite uniformly.
        applyRaceLinkDefaults();

        DEBUG_PRINTLN(F("[RaceLink] OPC_CONFIG 0x0F: every override reset to RACELINK_DEFAULT_* (no reboot required)"));
      } else if (p.option == 0x81) { // Reboot Node
        if (p.data0 != 0) doReboot = true;
      }
      #if DEV_TYPE == 50
      else if (p.option == 0x8C) { // Number of Slots
        const uint8_t value = constrain(p.data0, (uint8_t)1, (uint8_t)8);
        if (numberOfSlots != value) {
          numberOfSlots = value;
          #ifdef RACELINK_EPAPER
            setDisplayLayout(numberOfSlots);
          #endif
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
      
      //DEBUG_PRINTLN(F("[RaceLink] CONFIG -> applied"));
    } break;

    case OPC_STATUS: { // GET_STATUS -> STATUS_REPLY
      P_GetStatus p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (!groupMatch(p.groupId)) break;

      sendStatusReplyTo(h.sender);
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] GET_STATUS -> STATUS_REPLY"));
    } break;

    case OPC_GET_CONFIG: { // read-back of an OPC_CONFIG-style option
      // Body shape: 1 byte (option to read). Reply reuses P_Config (option +
      // data0..3) so the host's OPC_CONFIG codec can decode it unchanged.
      // Returns the *live* runtime value — and because applyRaceLinkDefaults()
      // runs after every OPC_CONFIG / WebUI Save, the live value is always
      // in sync with the override slot. Host sees "what is the device using
      // right now", which equals what cfg.json holds.
      RaceLinkProto::P_GetConfig p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (RaceLinkTransport::isBroadcast3(h.receiver)) return; // unicast-only, like OPC_CONFIG

      uint8_t d0 = 0, d1 = 0, d2 = 0, d3 = 0;
      bool readable = true;
      switch (p.option) {
        case 0x05: { // FPS
          d0 = strip.getTargetFps();
        } break;
        case 0x06: { // Segment 0 geometry
          uint16_t s = 0, e = 0;
          if (strip.getSegmentsNum() >= 1) {
            Segment& seg = strip.getSegment(0);
            s = seg.start; e = seg.stop;
          }
          d0 = (uint8_t)(s & 0xFF); d1 = (uint8_t)(s >> 8);
          d2 = (uint8_t)(e & 0xFF); d3 = (uint8_t)(e >> 8);
        } break;
        case 0x07: { // Segment 1 geometry
          uint16_t s = 0, e = 0;
          if (strip.getSegmentsNum() >= 2) {
            Segment& seg = strip.getSegment(1);
            s = seg.start; e = seg.stop;
          }
          d0 = (uint8_t)(s & 0xFF); d1 = (uint8_t)(s >> 8);
          d2 = (uint8_t)(e & 0xFF); d3 = (uint8_t)(e >> 8);
        } break;
        case 0x08: { // ABL max mA
          const uint16_t mA = BusManager::ablMilliampsMax();
          d0 = (uint8_t)(mA & 0xFF); d1 = (uint8_t)(mA >> 8);
        } break;
        case 0x09: { // briS
          d0 = briS;
        } break;
        case 0x0A: { // transitionDelayDefault
          const uint16_t ms = transitionDelayDefault;
          d0 = (uint8_t)(ms & 0xFF); d1 = (uint8_t)(ms >> 8);
        } break;
        #if DEV_TYPE == 50
        case 0x8C: { // STARTBLOCK: number of slots
          d0 = numberOfSlots;
        } break;
        case 0x8D: { // STARTBLOCK: first slot
          d0 = firstSlot;
        } break;
        #endif
        default: {
          // Unknown / write-only option (e.g. 0x02 ClearMaster, 0x0F ClearAll,
          // 0x81 Reboot). No reply -> host's send_and_wait_for_reply times out
          // gracefully and the dialog row shows "device: ? [Retry]".
          readable = false;
        } break;
      }
      if (!readable) break;
      sendGetConfigReplyTo(h.sender, p.option, d0, d1, d2, d3);
      acted = true;
      DEBUG_PRINTF_P(PSTR("[RaceLink] GET_CONFIG opt=0x%02X -> %u %u %u %u\n"),
                     (unsigned)p.option, (unsigned)d0, (unsigned)d1, (unsigned)d2, (unsigned)d3);
    } break;

    case OPC_STREAM: {
      acted = handleStreamPacket(buf, (uint8_t)len, h.sender);
    } break;

    case OPC_HEADLESS: { // Headless Mode broadcast (2B body)
      P_Headless p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      applyLocalScene(p.sceneId, p.brightness);
      acted = true;
      DEBUG_PRINTF_P(PSTR("[RaceLink] HEADLESS -> sceneId=%u bri=%u\n"),
                     (unsigned)p.sceneId, (unsigned)p.brightness);
    } break;

    case OPC_INDICATE: { // Central status-indicator overlay (2B body)
      P_Indicate p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      applyLocalIndicator(p.type, p.durationSec);
      acted = true;
      DEBUG_PRINTF_P(PSTR("[RaceLink] INDICATE -> type=%u dur=%us\n"),
                     (unsigned)p.type, (unsigned)p.durationSec);
    } break;

    case OPC_RF_CONFIG: { // Write LoRa PHY config (12 B P_RfConfig)
      // Unicast-only by design: a broadcast OPC_RF_CONFIG would knock
      // every reachable node off-channel simultaneously and brick the
      // fleet. The wire-level rule lives in racelink_proto.h; we
      // enforce it again here so a future broadcast-permitting rules
      // table change doesn't silently brick anyone.
      if (RaceLinkTransport::isBroadcast3(h.receiver)) {
        DEBUG_PRINTLN(F("[RaceLink] RF_CONFIG -> broadcast rejected"));
        break;
      }
      P_RfConfig p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;

      // Range validation: out-of-band freq, unsupported BW / SF / CR /
      // TX power / preamble all NACK with ACK_BAD_LEN. We could carry
      // a richer reason via P_Ack.status, but the host already
      // distinguishes "rejected by node" (any non-OK ACK) from
      // "node unreachable" (no ACK at all), so the existing status
      // space is sufficient.
      const auto reason = RfConfigNvs::validate(p);
      if (reason != RaceLinkProto::RF_CHANGE_OK) {
        sendAckTo(h.sender, OPC_RF_CONFIG, ACK_BAD_LEN);
        DEBUG_PRINTF_P(PSTR("[RaceLink] RF_CONFIG -> NACK reason=%u\n"),
                       (unsigned)reason);
        acted = true;
        break;
      }

      // Persist before ACK so a tight-timed reboot can't drop the
      // write. store() defensively re-validates; we treat any non-OK
      // result here as NVS failure (the only remaining failure mode).
      if (RfConfigNvs::store(p) != RaceLinkProto::RF_CHANGE_OK) {
        sendAckTo(h.sender, OPC_RF_CONFIG, ACK_ERROR);
        DEBUG_PRINTLN(F("[RaceLink] RF_CONFIG -> NVS write failed"));
        acted = true;
        break;
      }

      // Update the in-RAM mirror so a same-boot OPC_GET_RF_CONFIG read
      // before reboot reflects the freshly persisted values.
      g_activeRfConfig = p;

      // An RF-config write implies a network move: the previously-
      // learned master belongs to the OLD radio settings and won't
      // exist on the destination radio. Disable master persistence so
      // the boot-time loader (readFromConfig) ignores the persisted
      // slot — the node will start with masterKnown=false and
      // senderAllowed() will accept the next OPC_DEVICES / OPC_SET_GROUP
      // from the new gateway. The persisted slot itself is intentionally
      // left intact (the operator can re-enable persistence later to
      // snapshot whatever master is then live). Force the cfg.json
      // flush before restart -- the loop never gets a chance otherwise.
      macFilterPersist = false;
      configNeedsWrite = true;
      serializeConfigToFS();

      sendAckTo(h.sender, OPC_RF_CONFIG, ACK_OK);
      acted = true;
      DEBUG_PRINTF_P(PSTR("[RaceLink] RF_CONFIG -> stored + persistence disabled, rebooting (freq=%lu sf=%u bw=%u sw=0x%02X)\n"),
                     (unsigned long)p.freq_hz, (unsigned)p.sf,
                     (unsigned)p.bw_khz_x10, (unsigned)p.sync_word);

      // Apply the new PHY by rebooting -- radioInit() in setup() will
      // reload NVS on the way back up. RadioLib has no clean "reset
      // PHY mid-flight" path on a node that may be in the middle of a
      // streaming RX, and a node-side reboot is operator-visible (a
      // brief outage on a single device) rather than fleet-wide. The
      // 50 ms delay lets the ACK frame finish leaving the radio
      // before the SoC restarts.
      delay(50);
      ESP.restart();
      // not reached
    } break;

    case OPC_GET_RF_CONFIG: { // Read-back of the active PHY config
      if (RaceLinkTransport::isBroadcast3(h.receiver)) {
        // Unicast-only: a broadcast read-back would have every node
        // reply simultaneously and saturate the channel.
        break;
      }
      P_GetRfConfig p{};
      if (!parseBody(buf, (uint8_t)len, p)) break;
      if (p.reserved != 0) {
        // Reserved must be 0 today (future channel-slot selector).
        // Non-zero is a structurally invalid request -- no reply,
        // matching how OPC_GET_CONFIG handles unknown options.
        break;
      }
      sendRfConfigReplyTo(h.sender);
      acted = true;
      DEBUG_PRINTLN(F("[RaceLink] GET_RF_CONFIG -> reply sent"));
    } break;
  }

  if (acted) rxAccepted++;
}

bool UsermodRaceLink::sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac) {
  using namespace RaceLinkProto;
  uint8_t out[32];

  P_IdentifyReply p{};
  p.fw = PROTO_VER_MAJOR; // fw = protocol version
  p.caps            = DEV_TYPE; // rename caps to dev_type
  p.groupId         = current.groupId;

  if (includeFullMac && rl.macReadOK) {
    for (int i=0;i<6;i++) p.mac6[i] = rl.myMac6[i];
  } else {
    // If MAC unknown, send 0
    for (int i=0;i<6;i++) p.mac6[i] = 0;
  }

  uint8_t t = make_type(DIR_N2M, OPC_DEVICES); // IDENTIFY_REPLY
  uint8_t n = build(out, rl.myLast3, destLast3, t, p);
  
  return RaceLinkTransport::scheduleSend(rl, out, n);
}

void UsermodRaceLink::serviceStartupIdentifyReplies() {
  if (!radioReady) return;
  if (startupIdentifyStage >= 2) return;
  if (current.groupId != 0) {
    startupIdentifyStage = 2;
    return;
  }

  const uint32_t nowMs = millis();
  if ((int32_t)(nowMs - startupIdentifyAtMs[startupIdentifyStage]) < 0) return;

  static constexpr uint8_t BROADCAST_LAST3[3] = {0xFF, 0xFF, 0xFF};
  if (!sendIdentifyReplyTo(BROADCAST_LAST3, true)) return;

  startupIdentifyStage++;
  if (startupIdentifyStage >= 2) {
    DEBUG_PRINTLN(F("[RaceLink] Startup IDENTIFY_REPLY sequence complete"));
  }
}

void UsermodRaceLink::sendAckTo(const uint8_t destLast3[3], uint8_t echoOpcode7, RaceLinkProto::AckStatus st) {
  using namespace RaceLinkProto;
  uint8_t out[32];
  P_Ack p{ echoOpcode7, (uint8_t)st, 0 /*seq*/ };
  
  uint8_t t = make_type(DIR_N2M, OPC_ACK);
  uint8_t n = build(out, rl.myLast3, destLast3, t, p);
  
  RaceLinkTransport::scheduleSend(rl, out, n);
}

void UsermodRaceLink::sendStatusReplyTo(const uint8_t destLast3[3]) {
  using namespace RaceLinkProto;
  uint8_t out[32];

  P_StatusReply p{};
  {
    uint8_t fl = current.flags;
    if (bri > 0) fl |= RACELINK_FLAG_POWER_ON; else fl &= (uint8_t)~RACELINK_FLAG_POWER_ON;
    p.flags      = fl;
    uint8_t cfg = 0;
    if (macFilterEnabled) cfg |= RACELINK_CFG_MAC_FILTER_ENABLED;
    if (macFilterPersist) cfg |= RACELINK_CFG_MAC_FILTER_PERSIST;
    if (apActive) cfg |= RACELINK_CFG_AP_ACTIVE;
    p.configByte = cfg;
    p.effectId   = strip.getMainSegment().mode;  // active segment mode (renamed from presetId)
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
  
  p.rssi     = (int8_t)rl.lastRssi;
  p.snr      = (int8_t)rl.lastSnr;

  uint8_t t = make_type(DIR_N2M, OPC_STATUS); // STATUS_REPLY
  uint8_t n = build(out, rl.myLast3, destLast3, t, p);

  RaceLinkTransport::scheduleSend(rl, out, n);
  //RaceLinkTransport::scheduleSend(rl, out, n, 50, 2500);
}

void UsermodRaceLink::sendGetConfigReplyTo(const uint8_t destLast3[3], uint8_t option,
                                           uint8_t data0, uint8_t data1,
                                           uint8_t data2, uint8_t data3) {
  // GET_CONFIG_REPLY: same opcode as the request with the N2M direction bit
  // and a P_Config-shaped body (option + data0..3). Reuses the standard
  // build() helper so the byte layout matches every other OPC_CONFIG frame
  // on the wire. Data byte packing per option follows the matching write
  // command's layout (see OPC_CONFIG dispatcher above).
  using namespace RaceLinkProto;
  uint8_t out[32];

  P_Config p{};
  p.option = option;
  p.data0  = data0;
  p.data1  = data1;
  p.data2  = data2;
  p.data3  = data3;

  uint8_t t = make_type(DIR_N2M, OPC_GET_CONFIG);
  uint8_t n = build(out, rl.myLast3, destLast3, t, p);

  RaceLinkTransport::scheduleSend(rl, out, n);
}

void UsermodRaceLink::sendRfConfigReplyTo(const uint8_t destLast3[3]) {
  // GET_RF_CONFIG_REPLY: same opcode as the request with the N2M
  // direction bit and a P_RfConfig-shaped body (12 B). Reuses the
  // standard build() helper so the byte layout matches the
  // OPC_RF_CONFIG write frame on the wire. Returns the in-RAM
  // g_activeRfConfig, which mirrors the persisted NVS slot 1:1 after
  // radioInit() (and equals the freshly-persisted value during the
  // tiny window between an OPC_RF_CONFIG ACK and the ESP.restart()).
  using namespace RaceLinkProto;
  uint8_t out[32];

  uint8_t t = make_type(DIR_N2M, OPC_GET_RF_CONFIG);
  uint8_t n = build(out, rl.myLast3, destLast3, t, g_activeRfConfig);

  RaceLinkTransport::scheduleSend(rl, out, n);
}

// ========= PRESET handler (always immediate) =========
// Most of p.flags is IGNORED here — control flags are processed only by
// OPC_CONTROL (single-writer rule). PRESET cannot be armed and cannot drive
// FORCE_TT0 / FORCE_REAPPLY. A pending CONTROL is intentionally NOT cleared:
// the queued effect-parameter overrides will overlay on top of the loaded
// preset on the next SYNC. The OFFSET_MODE flag IS honoured because the
// gate (in handlePacket) already filtered on it; we use it to propagate the
// per-device phase offset onto strip.timebase.
void UsermodRaceLink::handlePreset(const RaceLinkProto::P_Preset& p) {
  haveControl = true;
  // PRESET is a new authoritative state — preempt any running indicator.
  cancelIndicator();

  applyPreset(p.presetId, CALL_MODE_NO_NOTIFY);
  bri = p.brightness;

  current.presetId   = p.presetId;
  current.brightness = p.brightness;

  refreshFieldsFromSegment();
  stateUpdated(CALL_MODE_NO_NOTIFY);

  // Update the per-device phase offset for cyclic effects in the loaded
  // preset. With OFFSET_MODE flag set, compute against the now-active
  // config (pendingChange already materialised by the dispatch); without
  // the flag, reset to 0.
  const int32_t newOffsetMs = (p.flags & RACELINK_FLAG_OFFSET_MODE)
                              ? (int32_t)computeOffsetMs(active)
                              : 0;
  setActivePhaseOffsetMs(newOffsetMs);
}

// ========= CONTROL parser =========
// Variable-length body (3..MAX_P_CONTROL). Layout see P_Control in
// racelink_proto.h. Returns false if the body is shorter than the fieldMask/
// extMask imply, or if any trailing bytes remain unconsumed (length mismatch).
// Pre-rename: parseControlAdvBody.
bool UsermodRaceLink::parseControlBody(const uint8_t* body, uint8_t len,
                                       uint8_t& groupId, uint8_t& flags,
                                       AdvancedFields& out) {
  using namespace RaceLinkProto;
  if (!body || len < 3) return false;

  out = AdvancedFields{};
  uint8_t p = 0;
  groupId       = body[p++];
  flags         = body[p++];
  const uint8_t fm = body[p++];
  out.fieldMask = fm;

  auto readU8 = [&](uint8_t& v) -> bool {
    if (p >= len) return false;
    v = body[p++];
    return true;
  };
  auto readRGB = [&](uint32_t& c) -> bool {
    if ((uint16_t)p + 3 > len) return false;
    const uint8_t r = body[p++];
    const uint8_t g = body[p++];
    const uint8_t b = body[p++];
    c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b; // W=0
    return true;
  };

  if (fm & RL_CTRL_F_BRIGHTNESS) { if (!readU8(out.brightness)) return false; }
  if (fm & RL_CTRL_F_MODE)       { if (!readU8(out.mode))       return false; }
  if (fm & RL_CTRL_F_SPEED)      { if (!readU8(out.speed))      return false; }
  if (fm & RL_CTRL_F_INTENSITY)  { if (!readU8(out.intensity))  return false; }
  if (fm & RL_CTRL_F_CUSTOM1)    { if (!readU8(out.custom1))    return false; }
  if (fm & RL_CTRL_F_CUSTOM2)    { if (!readU8(out.custom2))    return false; }
  if (fm & RL_CTRL_F_CUSTOM3_CHECKS) {
    uint8_t v = 0;
    if (!readU8(v)) return false;
    out.custom3 = (uint8_t)(v & RL_CTRL_C3_MASK);
    out.check1  = (v & RL_CTRL_CHECK1_BIT) != 0;
    out.check2  = (v & RL_CTRL_CHECK2_BIT) != 0;
    out.check3  = (v & RL_CTRL_CHECK3_BIT) != 0;
  }
  if (fm & RL_CTRL_F_EXT) {
    uint8_t em = 0;
    if (!readU8(em)) return false;
    out.extMask = em;
    if (em & RL_CTRL_E_PALETTE) { if (!readU8(out.palette)) return false; }
    if (em & RL_CTRL_E_COLOR1)  { if (!readRGB(out.color1)) return false; }
    if (em & RL_CTRL_E_COLOR2)  { if (!readRGB(out.color2)) return false; }
    if (em & RL_CTRL_E_COLOR3)  { if (!readRGB(out.color3)) return false; }
  }

  // Length must match exactly -- any trailing byte indicates a malformed
  // packet or an unknown reserved bit the sender expected us to understand.
  return (p == len);
}

// ========= CONTROL apply (segment-level field writes) =========
// Applies only fields whose bit is set in fieldMask/extMask. Power/brightness
// are driven by the existing RACELINK_FLAG_* semantics, identical to OPC_PRESET.
void UsermodRaceLink::applyAdvancedFields(uint8_t flags, const AdvancedFields& f) {
  using namespace RaceLinkProto;

  uint16_t prevTT = 0;
  const bool ttForced = (flags & RACELINK_FLAG_FORCE_TT0) != 0;
  if (ttForced) { prevTT = transitionDelay; transitionDelay = 0; }

  Segment& seg = strip.getMainSegment();

  if (f.fieldMask & RL_CTRL_F_MODE)      seg.setMode(f.mode);
  if (f.fieldMask & RL_CTRL_F_SPEED)     seg.speed      = f.speed;
  if (f.fieldMask & RL_CTRL_F_INTENSITY) seg.intensity  = f.intensity;
  if (f.fieldMask & RL_CTRL_F_CUSTOM1)   seg.custom1    = f.custom1;
  if (f.fieldMask & RL_CTRL_F_CUSTOM2)   seg.custom2    = f.custom2;
  if (f.fieldMask & RL_CTRL_F_CUSTOM3_CHECKS) {
    seg.custom3 = f.custom3;
    seg.check1  = f.check1;
    seg.check2  = f.check2;
    seg.check3  = f.check3;
  }
  if (f.extMask & RL_CTRL_E_PALETTE) seg.setPalette(f.palette);
  if (f.extMask & RL_CTRL_E_COLOR1)  seg.setColor(0, f.color1);
  if (f.extMask & RL_CTRL_E_COLOR2)  seg.setColor(1, f.color2);
  if (f.extMask & RL_CTRL_E_COLOR3)  seg.setColor(2, f.color3);

  // Power/brightness: identical semantics to OPC_PRESET path.
  //  - POWER_ON=0     -> bri forced to 0
  //  - HAS_BRI + BRI  -> explicit brightness from body
  //  - otherwise      -> bri left untouched (SYNC may drive it later)
  if (!(flags & RACELINK_FLAG_POWER_ON)) {
    bri = 0;
  } else if ((flags & RACELINK_FLAG_HAS_BRI) && (f.fieldMask & RL_CTRL_F_BRIGHTNESS)) {
    bri = f.brightness;
  }

  // FORCE_REAPPLY: re-trigger render even if all writes were no-ops.
  stateUpdated(CALL_MODE_NO_NOTIFY);
  if (flags & RACELINK_FLAG_FORCE_REAPPLY) {
    seg.markForReset();
    stateUpdated(CALL_MODE_NO_NOTIFY);
  }

  if (ttForced) transitionDelay = prevTT;
}

// ========= CONTROL handler (single flag writer; arm-on-sync or immediate) =========
// OPC_CONTROL is the ONLY writer of current.flags. Behaviour is centralized
// here so flag-driven semantics (ARM, FORCE_*, OFFSET_MODE) are interpreted
// in exactly one place.
void UsermodRaceLink::handleControl(uint8_t flags, const AdvancedFields& f) {
  haveControl = true;
  current.flags = flags;
  // CONTROL is a new authoritative state — preempt any running indicator.
  cancelIndicator();

  if (flags & RACELINK_FLAG_ARM_ON_SYNC) {
    pending.fields = f;
    pending.rxAtMs = millis();
    // Evaluate the *effective* config (pending if set, else active) against
    // this device's groupId and snapshot the result. Using the effective
    // config lets the user send OPC_OFFSET followed immediately by an
    // arming OPC_CONTROL — the new formula's per-device value is captured
    // even though pendingChange has not yet materialised (that happens in
    // handleSync below). A subsequent OPC_OFFSET cannot retroactively
    // shift this queued effect — its new value is picked up at the next
    // arm.
    if (flags & RACELINK_FLAG_OFFSET_MODE) {
      const OffsetConfig& effCfg = pendingChangeValid ? pendingChange : active;
      pending.offsetMs = computeOffsetMs(effCfg);
    } else {
      pending.offsetMs = 0;
    }
    pending.valid  = true;
    // A new arm cancels any deferred apply still waiting from a previous
    // SYNC: the operator chose to re-arm before that deadline elapsed.
    pendingDeferred = false;
    return;
  }

  // Immediate-apply path. Also clears any previously-armed CONTROL — sending
  // a CONTROL with ARM_ON_SYNC=0 is the documented way to cancel a pending
  // queued effect. Also cancels a deferred apply for the same reason.
  // The offset acceptance gate already accepted this packet; the dispatch
  // switch materialised pendingChange before invoking handleControl, so
  // ``active`` already reflects the new state.
  applyAdvancedFields(flags, f);
  mergeFieldsIntoSnapshot(f);

  if ((flags & RACELINK_FLAG_HAS_BRI) && (f.fieldMask & RaceLinkProto::RL_CTRL_F_BRIGHTNESS)) {
    current.brightness = f.brightness;
  }

  // Update the per-device phase offset for cyclic effects. With OFFSET_MODE,
  // compute this device's offset against the now-active config; without it
  // (offset mode disabled) reset to 0. Keeps strip.timebase consistent with
  // the new effect's phase semantics immediately.
  const int32_t newOffsetMs = (flags & RACELINK_FLAG_OFFSET_MODE)
                              ? (int32_t)computeOffsetMs(active)
                              : 0;
  setActivePhaseOffsetMs(newOffsetMs);

  pending.valid = false;
  pendingDeferred = false;
}

// ========= OFFSET handler =========
// Variable-length parser. Body layout (see racelink_proto.h):
//   Byte 0: groupId   (0..254 = filter; 255 = broadcast all)
//   Byte 1: mode      (OffsetMode)
//   Byte 2..: mode-specific payload
//
// Filters on groupId so a broadcast (255) is accepted by every device
// while a specific group only matches its members. Returns false on
// malformed bodies; caller treats false as drop.
//
// A new OPC_OFFSET overwrites any prior pendingChange — the documented
// way to cancel a not-yet-materialised activation.
bool UsermodRaceLink::handleOffset(const uint8_t* body, uint8_t bodyLen) {
  using namespace RaceLinkProto;
  if (bodyLen < 2) return false;
  const uint8_t groupId = body[0];
  if (groupId != 255 && groupId != current.groupId) return false;

  const uint8_t mode = body[1];
  OffsetConfig cfg{};
  cfg.mode = mode;

  switch (mode) {
    case OFFSET_MODE_NONE:
      if (bodyLen != 2) return false;
      break;
    case OFFSET_MODE_EXPLICIT:
      if (bodyLen != 4) return false;
      cfg.explicit_ms = (uint16_t)body[2] | ((uint16_t)body[3] << 8);
      break;
    case OFFSET_MODE_LINEAR:
      if (bodyLen != 6) return false;
      cfg.base_ms = (int16_t)((uint16_t)body[2] | ((uint16_t)body[3] << 8));
      cfg.step_ms = (int16_t)((uint16_t)body[4] | ((uint16_t)body[5] << 8));
      break;
    case OFFSET_MODE_VSHAPE:
      if (bodyLen != 7) return false;
      cfg.base_ms = (int16_t)((uint16_t)body[2] | ((uint16_t)body[3] << 8));
      cfg.step_ms = (int16_t)((uint16_t)body[4] | ((uint16_t)body[5] << 8));
      cfg.center  = body[6];
      break;
    case OFFSET_MODE_MODULO:
      if (bodyLen != 7) return false;
      cfg.base_ms = (int16_t)((uint16_t)body[2] | ((uint16_t)body[3] << 8));
      cfg.step_ms = (int16_t)((uint16_t)body[4] | ((uint16_t)body[5] << 8));
      cfg.cycle   = body[6] ? body[6] : 1;
      break;
    default:
      return false;  // unknown mode (forward-compat: drop)
  }

  pendingChange      = cfg;
  pendingChangeValid = true;
  return true;
}

bool UsermodRaceLink::offsetGateAccepts(uint8_t packetFlags) const {
  // Strict symmetric gate (2026-04-30 correction):
  //
  // The packet's OFFSET_MODE flag MUST match the receiver's effective
  // offset state. Both directions are strict:
  //
  //   F=1 + E=1  -> ACCEPT  (use stored offset)
  //   F=0 + E=0  -> ACCEPT  (normal immediate apply)
  //   F=1 + E=0  -> DROP    (use-offset request without configured offset)
  //   F=0 + E=1  -> DROP    (immediate-apply request to an offset-configured
  //                          device; the device "stays in offset mode" until
  //                          OPC_OFFSET(NONE) followed by a materialisation
  //                          event explicitly transitions it out)
  //
  // Design rule: state transitions between "in offset mode" and "not in
  // offset mode" happen ONLY via OPC_OFFSET. CONTROL/PRESET packets just
  // dispatch effects within the current state; they never transition the
  // device's offset configuration. This keeps Strategy A (broadcast OPC_CONTROL
  // with F=1 lands on exactly the configured subset) functional and gives
  // the operator a single, explicit transition mechanism.
  //
  // Effective config = pending if set, else active. To leave offset mode,
  // the operator sends OPC_OFFSET(NONE) (which sets pending=NONE so
  // subsequent F=0 packets accept), then sends a F=0 packet that
  // materialises pending into active (an OPC_PRESET, or an ARM_ON_SYNC
  // OPC_CONTROL followed by OPC_SYNC). The host-side scene_runner's
  // ``offset_group(mode=none)`` container performs both steps in one
  // operator action.
  const OffsetConfig& eff = pendingChangeValid ? pendingChange : active;
  const bool effIsActive = (eff.mode != RaceLinkProto::OFFSET_MODE_NONE);
  const bool packetWantsActive = (packetFlags & RACELINK_FLAG_OFFSET_MODE) != 0;
  return packetWantsActive == effIsActive;
}

void UsermodRaceLink::materialisePendingChange() {
  if (!pendingChangeValid) return;
  active = pendingChange;
  pendingChangeValid = false;
}

// ========= Formula evaluator =========
// Mirrors ``racelink/domain/offset_formula.py`` on the host. Both must
// produce byte-identical results for any (config, group_id) triple.
uint16_t UsermodRaceLink::computeOffsetMs(const OffsetConfig& cfg) const {
  using namespace RaceLinkProto;
  const int32_t gid = (int32_t)current.groupId;
  int32_t v = 0;
  switch (cfg.mode) {
    case OFFSET_MODE_NONE:
      v = 0;
      break;
    case OFFSET_MODE_EXPLICIT:
      v = (int32_t)cfg.explicit_ms;
      break;
    case OFFSET_MODE_LINEAR:
      v = (int32_t)cfg.base_ms + gid * (int32_t)cfg.step_ms;
      break;
    case OFFSET_MODE_VSHAPE: {
      const int32_t d = gid - (int32_t)cfg.center;
      const int32_t da = (d < 0) ? -d : d;
      v = (int32_t)cfg.base_ms + da * (int32_t)cfg.step_ms;
      break;
    }
    case OFFSET_MODE_MODULO: {
      const int32_t cycle = (cfg.cycle > 0) ? cfg.cycle : 1;
      v = (int32_t)cfg.base_ms + (gid % cycle) * (int32_t)cfg.step_ms;
      break;
    }
    default:
      v = 0;
      break;
  }
  if (v < 0) v = 0;
  if (v > 0xFFFF) v = 0xFFFF;
  return (uint16_t)v;
}

// ========= Deferred apply tick =========
// Replays the same apply sequence handleSync() runs inline when offsetMs == 0.
// Called from loop() each iteration; cheap when no deferred apply is queued.
void UsermodRaceLink::serviceDeferredApply() {
  if (!pendingDeferred) return;
  // Wrap-safe deadline test: signed difference >= 0 means deadline passed.
  if ((int32_t)(millis() - pendingDeferredAt) < 0) return;

  applyAdvancedFields(pendingDeferredFlags, pendingDeferredFields);

  if ((pendingDeferredFlags & RACELINK_FLAG_POWER_ON) &&
      !(pendingDeferredFlags & RACELINK_FLAG_HAS_BRI)) {
    if (pendingDeferredBri != bri) {
      bri = pendingDeferredBri;
      stateUpdated(CALL_MODE_NO_NOTIFY);
    }
  }

  mergeFieldsIntoSnapshot(pendingDeferredFields);
  current.brightness = bri;

  // Promote the captured offset to the device's persistent phase offset.
  // Adjusts strip.timebase by the delta so direct-time-based effects
  // (Breathe, Pacifica, ...) immediately get the configured phase shift
  // without waiting for the next SYNC.
  setActivePhaseOffsetMs((int32_t)pendingDeferredOffsetMs);

  pendingDeferred = false;
}

// ========= Phase-offset re-assert (after HARD sync only) =========
// strip.timebase has just been written to the master-aligned desiredTb
// (no offset baked in). Subtract activePhaseOffsetMs so this device's
// strip.now runs that many ms behind master. SOFT sync does NOT call
// this — it converges naturally because err is computed against the
// logical timebase (strip.timebase + activePhaseOffsetMs).
void UsermodRaceLink::applyPhaseOffsetAfterHardSync() {
  strip.timebase = (uint32_t)((int32_t)strip.timebase - activePhaseOffsetMs);
}

// ========= Switch active phase offset =========
// Transitions activePhaseOffsetMs from its current value to ``newOffsetMs``,
// adjusting strip.timebase by the delta so the device's effective phase
// shift updates immediately (no need to wait for the next SYNC). Used by
// apply paths to switch into / out of offset mode and to update the offset
// when a new effect arrives with a different per-group value.
void UsermodRaceLink::setActivePhaseOffsetMs(int32_t newOffsetMs) {
  const int32_t delta = activePhaseOffsetMs - newOffsetMs;
  strip.timebase = (uint32_t)((int32_t)strip.timebase + delta);
  activePhaseOffsetMs = newOffsetMs;
}

void UsermodRaceLink::mergeFieldsIntoSnapshot(const AdvancedFields& f) {
  using namespace RaceLinkProto;
  AdvancedFields& snap = current.fields;
  if (f.fieldMask & RL_CTRL_F_BRIGHTNESS) snap.brightness = f.brightness;
  if (f.fieldMask & RL_CTRL_F_MODE)       snap.mode       = f.mode;
  if (f.fieldMask & RL_CTRL_F_SPEED)      snap.speed      = f.speed;
  if (f.fieldMask & RL_CTRL_F_INTENSITY)  snap.intensity  = f.intensity;
  if (f.fieldMask & RL_CTRL_F_CUSTOM1)    snap.custom1    = f.custom1;
  if (f.fieldMask & RL_CTRL_F_CUSTOM2)    snap.custom2    = f.custom2;
  if (f.fieldMask & RL_CTRL_F_CUSTOM3_CHECKS) {
    snap.custom3 = f.custom3;
    snap.check1  = f.check1;
    snap.check2  = f.check2;
    snap.check3  = f.check3;
  }
  if (f.extMask & RL_CTRL_E_PALETTE) snap.palette = f.palette;
  if (f.extMask & RL_CTRL_E_COLOR1)  snap.color1  = f.color1;
  if (f.extMask & RL_CTRL_E_COLOR2)  snap.color2  = f.color2;
  if (f.extMask & RL_CTRL_E_COLOR3)  snap.color3  = f.color3;
  snap.fieldMask |= f.fieldMask;
  snap.extMask   |= f.extMask;
  current.lastUpdateMs = millis();
}

// ========= SYNC handler (global, irregular arrival OK) =========
// ``syncFlags`` carries SYNC_FLAG_* bits from the 5 B SYNC form (or 0 for
// the legacy 4 B form). Timebase unwrap + correction is unconditional;
// pending arm-on-sync materialisation is gated on SYNC_FLAG_TRIGGER_ARMED
// so an autosync pulse cannot fire armed effects ahead of the scene
// runner's deliberate sync.
void UsermodRaceLink::handleSync(uint32_t ts24, uint8_t briFromPkt, uint8_t syncFlags) {
  using namespace RaceLinkProto;  // for SYNC_FLAG_TRIGGER_ARMED
  const bool fireArmed = (syncFlags & SYNC_FLAG_TRIGGER_ARMED) != 0;
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

  // Drift correction is computed against the LOGICAL (master-aligned)
  // timebase = strip.timebase + activePhaseOffsetMs. Without this, a
  // non-zero activePhaseOffsetMs would always look like err ==
  // activePhaseOffsetMs and trigger endless hard-resyncs.
  const uint32_t logicalTb = (uint32_t)((int32_t)strip.timebase + activePhaseOffsetMs);
  const int32_t err = (int32_t)(desiredTb - logicalTb);
  lastSyncTbErrMs = err; // debug/info
  const uint32_t aerr = (err < 0) ? (uint32_t)(-err) : (uint32_t)err;

  const bool hard = pending.valid || (aerr > (uint32_t)RACELINK_SYNC_HARD_RESYNC_MS);
  if (hard) {
    // Hard reset to master-aligned timebase, then re-bake the per-device
    // phase offset (so cyclic effects keep their phase relative to other
    // groups). Soft sync below does NOT need this — its step is computed
    // against logicalTb, so strip.timebase converges to desiredTb -
    // activePhaseOffsetMs naturally.
    strip.timebase = desiredTb;
    applyPhaseOffsetAfterHardSync();
  } else {
    int32_t step = err;
    if (step > (int32_t)RACELINK_SYNC_MAX_STEP_MS) step = (int32_t)RACELINK_SYNC_MAX_STEP_MS;
    if (step < -(int32_t)RACELINK_SYNC_MAX_STEP_MS) step = -(int32_t)RACELINK_SYNC_MAX_STEP_MS;
    strip.timebase = (uint32_t)((int32_t)strip.timebase + step);
  }

  // ---- apply pending CONTROL on first TRIGGER SYNC after it was armed ----
  // Gate on SYNC_FLAG_TRIGGER_ARMED: only the deliberate fire from the host
  // (scene runner / operator) clears `pending.valid`. Autosync pulses skip
  // this block entirely so they cannot race the deliberate sync.
  if (fireArmed && pending.valid) {
    // Materialise any queued offset-mode change *first*: the queued effect
    // semantically belongs to the new mode. This is the second of two
    // materialisation sites (the immediate-apply path materialises in the
    // dispatch switch before invoking handleControl). After this, future
    // OPC_CONTROL/OPC_PRESET packets see the new activeMode for gate checks.
    materialisePendingChange();

    // Two paths depending on whether the arming CONTROL requested an offset:
    //   offsetMs == 0 -> apply immediately (legacy fast path; zero overhead)
    //   offsetMs >  0 -> snapshot state and defer until ``millis() + offsetMs``;
    //                    the per-loop ``serviceDeferredApply()`` runs the apply.
    if (pending.offsetMs == 0) {
      applyAdvancedFields(current.flags, pending.fields);

      // If the queued CONTROL had no brightness but POWER_ON is set, take live
      // brightness from the SYNC packet.
      if ((current.flags & RACELINK_FLAG_POWER_ON) &&
          !(current.flags & RACELINK_FLAG_HAS_BRI)) {
        if (briFromPkt != bri) {
          bri = briFromPkt;
          stateUpdated(CALL_MODE_NO_NOTIFY);
        }
      }

      mergeFieldsIntoSnapshot(pending.fields);
      current.brightness = bri;

      // Group with offset 0 (or no offset mode) → transition to a 0 offset.
      // Adjusts strip.timebase by the delta so the running effect's phase
      // is correct immediately, without waiting for the next SYNC.
      setActivePhaseOffsetMs(0);

      pending.valid = false;
      return;
    }

    // Deferred path: snapshot everything we need so a subsequent OPC_OFFSET
    // / OPC_CONTROL packet between now and the deadline does not shift this
    // queued effect. ``handleControl()`` clears ``pendingDeferred`` if a
    // brand-new CONTROL re-arms before the deadline.
    pendingDeferredFlags    = current.flags;
    pendingDeferredBri      = briFromPkt;
    pendingDeferredFields   = pending.fields;
    pendingDeferredAt       = nowMs + pending.offsetMs;
    pendingDeferredOffsetMs = pending.offsetMs;  // captured for serviceDeferredApply
    pendingDeferred         = true;

    pending.valid = false;
    return;
  }

  // ---- optional live brightness via SYNC (only if last CONTROL had NO brightness) ----
  if (haveControl && !(current.flags & RACELINK_FLAG_HAS_BRI)) {
    const uint8_t desiredBri = (current.flags & RACELINK_FLAG_POWER_ON) ? briFromPkt : 0;
    if (desiredBri != bri) {
      bri = desiredBri;
      current.brightness = bri;
      stateUpdated(CALL_MODE_NO_NOTIFY);
    }
  }
}


// --- Callback bridges as static members ---
void UsermodRaceLink::on_rx_node(const uint8_t* pkt, uint8_t len,
                                        int16_t rssi, int8_t snr,
                                        void* ctx) {
  auto* self = static_cast<UsermodRaceLink*>(ctx);
  if (!self || !pkt || len == 0) return;

/*   self->lastRssi = rssi;
  self->lastSnr  = snr;
  self->rxTotal++; */

  captureLastRxPacket(pkt, len);
  self->handlePacket(pkt, len);
}

/* void UsermodRaceLink::on_tx_done_node(void* ctx) {
  auto* self = static_cast<UsermodRaceLink*>(ctx);
  if (!self) return;
  // Optional: node-specific TX events
} */

// ========= MAC helpers =========
/* void UsermodRaceLink::readEfuseMac() {
  if (RaceLinkTransport::readEfuseMac6(myMac6)) {
    macReadOK = true;
    RaceLinkTransport::last3FromMac6(myLast3, myMac6);
  } else {
    macReadOK = false;
    memset(myLast3, 0, 3);
  }
} */

// ===================== Headless Mode =====================
// Wire-stable definitions live in racelink_headless.h so external Gateway-
// side software (e.g. FPVGate) can ``#include`` it and emit the same
// scene-id / packet bytes that the WLED Headless master uses.

void UsermodRaceLink::tryStartHeadless() {
  // Toggle off when already active. No probe needed — we know we're the
  // master right now, so a "stop" is unambiguous.
  if (headless.active) {
    exitHeadlessMode();
    return;
  }
  // Already in the probe window — silently ignore (a stuck 5-click is
  // probably the user being impatient, not a request to re-probe).
  if (headless.probing) return;
  if (!radioReady || !rl.macReadOK) {
    DEBUG_PRINTLN(F("[RaceLink] Headless: radio/MAC not ready, refusing probe"));
    return;
  }

  // Pick the jittered first-send time. esp_random() is the hardware RNG
  // so two simultaneously powered-on persisted-headless devices end up
  // with distinct probe schedules.
  const uint32_t now = millis();
  const uint32_t range = RaceLinkHeadless::HEADLESS_PROBE_JITTER_MAX_MS
                       - RaceLinkHeadless::HEADLESS_PROBE_JITTER_MIN_MS;
  const uint32_t j = RaceLinkHeadless::HEADLESS_PROBE_JITTER_MIN_MS
                   + (esp_random() % (range + 1));
  headless.probing            = true;
  headless.probeAborted       = false;
  headless.probeFirstSent     = false;
  headless.probeSecondSent    = false;
  headless.probeFirstSendAtMs = now + j;
  headless.probeSecondSendAtMs= now + j + RaceLinkHeadless::HEADLESS_PROBE_RETRY_OFFSET_MS;
  headless.probeStartedAtMs   = 0; // set when the first probe actually goes out
  DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: probe scheduled (+%lu ms)\n"),
                 (unsigned long)j);
}

void UsermodRaceLink::serviceHeadless(uint32_t now) {
  if (!headless.probing && !headless.active) return;

  if (headless.probing) {
    // Abort path: noteMasterRx() set probeAborted because some master
    // proved it is alive on the channel during our probe window.
    if (headless.probeAborted) {
      headless.probing = false;
      headless.probeAborted = false;
      // Indicator system drives the 3-second red-strobe overlay AND auto-
      // restores the pre-probe segment state (typically the boot-random
      // color) when the duration expires. Replaces the old btn.blink*
      // state machine + manual applyCycleColor red/black toggle.
      applyLocalIndicator(RaceLinkIndicators::IND_PROBE_REJECTED, 5);
      DEBUG_PRINTLN(F("[RaceLink] Headless: probe rejected — master active"));
      return;
    }

    // First scheduled probe send (start-of-window anchor).
    if (!headless.probeFirstSent && (int32_t)(now - headless.probeFirstSendAtMs) >= 0) {
      if (radioReady && rl.macReadOK) {
        uint8_t out[32];
        uint8_t n = RaceLinkHeadless::buildIdentifyProbe(
            out, rl.myLast3, rl.myMac6, RaceLinkProto::PROTO_VER_MAJOR, DEV_TYPE);
        // armBlip=false: probe runs while headless.active is still false anyway,
        // and a TX flash during the "am I the master?" handshake would mislead
        // the operator into thinking promotion already happened.
        if (n) headlessSendTx(out, n, /*armBlip=*/false);
      }
      headless.probeFirstSent   = true;
      headless.probeStartedAtMs = now;
    }

    // Second probe — single-loss coverage at SF7.
    if (headless.probeFirstSent && !headless.probeSecondSent
        && (int32_t)(now - headless.probeSecondSendAtMs) >= 0) {
      if (radioReady && rl.macReadOK) {
        uint8_t out[32];
        uint8_t n = RaceLinkHeadless::buildIdentifyProbe(
            out, rl.myLast3, rl.myMac6, RaceLinkProto::PROTO_VER_MAJOR, DEV_TYPE);
        if (n) headlessSendTx(out, n, /*armBlip=*/false);
      }
      headless.probeSecondSent = true;
    }

    // Timeout: no abort fired within the probe window → promote.
    if (headless.probeFirstSent
        && (now - headless.probeStartedAtMs) >= RaceLinkHeadless::HEADLESS_PROBE_TIMEOUT_MS) {
      enterHeadlessMode();
    }
    return;
  }

  // active: SYNC keepalive. 4B autosync (timebase + bri) every
  // HEADLESS_SYNC_KEEPALIVE_MS so paired slaves' masterContactedRecently()
  // stays true AND their strip.timebase stays anchored to the master
  // clock — cyclic effects like Breathe then hold phase across the fleet.
  // Scene state is NOT re-broadcast: a slave that joins mid-session is
  // pair-confirmed via showPairConfirmedEffect and stays on its boot
  // visual until the operator drives a new scene (1-click) — exactly the
  // same UX a Gateway+Host pair produces.
  if (headless.active) {
    if ((now - headless.lastBroadcastAtMs) >= RaceLinkHeadless::HEADLESS_SYNC_KEEPALIVE_MS) {
      headlessBroadcastSync(now);
    }
  }
}

void UsermodRaceLink::enterHeadlessMode() {
  headless.active  = true;
  headless.probing = false;
  // Promoting to headless master is a deliberate role transition; any
  // running indicator overlay should yield to the entry cue (solid blue)
  // and the subsequent scene broadcast.
  cancelIndicator();
  // The master conceptually owns Group 1 (Group 0 is the unconfigured pool,
  // slaves start at HEADLESS_FIRST_GROUP_ID = 2). Setting current.groupId
  // explicitly here keeps the wire-level receiver filter coherent for any
  // direct-to-1 packets and makes the role visible in /json/cfg.
  current.groupId = RaceLinkHeadless::HEADLESS_MASTER_GROUP_ID;
  if (!overrides.headlessPersistedActive) {
    overrides.headlessPersistedActive = true;
    configNeedsWrite = true;
  }
  // The headless master conceptually has no paired master. Zero the entire
  // master-tracking state — that way noteMasterRx() can no longer fire
  // (its handlePacket call site is gated by masterKnown && same3) and
  // masterContactedRecently() stays definitionally false for the duration
  // of headless operation. Cascades into a simpler serviceButtonFade gate
  // and a cleaner senderAllowed filter (only OPC_DEVICES/OPC_SET_GROUP
  // from foreign senders pass — exactly the right surface for "I am the
  // master here"). On clean takeover the OPC_SET_GROUP handler rebinds
  // masterLast3 via learnMasterFromSender.
  clearMaster();
  masterFull6Known     = false;
  memset(masterFull6, 0, sizeof(masterFull6));
  anyMasterRxSinceBoot = false;
  lastMasterRxMs       = 0;
  // Visual success cue: 3-second blue breathe via the central indicator
  // system. If a persisted scene is restored right after (see below),
  // headlessBroadcastCurrentScene -> applyLocalScene -> cancelIndicator
  // preempts this overlay immediately and the operator sees the scene
  // instead — matching the previous solid-blue behavior.
  applyLocalIndicator(RaceLinkIndicators::IND_HEADLESS_ENTER, 5);
  DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: ACTIVE (counter=%u, scene=%u)\n"),
                 (unsigned)overrides.headlessGroupCounter,
                 (unsigned)overrides.headlessCurrentScene);
  // Master self-sync before the first broadcast: applyLocalScene() (invoked
  // from headlessBroadcastCurrentScene below) re-applies setActivePhaseOffsetMs
  // delta-based against the existing strip.timebase. Without resetting the
  // invariant first, perturbations accumulated since the last reboot persist
  // through the new headless session. See headlessBroadcastSync() for the
  // permanent per-keepalive re-anchor.
  strip.timebase = (uint32_t)(-(int32_t)activePhaseOffsetMs);

  bool sceneRestored = false;
  if (overrides.headlessCurrentScene != 0xFF) {
    // Resolve sceneId -> catalog index for the in-memory state.
    headless.currentSceneIdx = 0xFF;
    for (uint8_t i = 0; i < RaceLinkHeadless::SCENE_CATALOG_SIZE; ++i) {
      if (RaceLinkHeadless::SCENE_CATALOG[i].sceneId == overrides.headlessCurrentScene) {
        headless.currentSceneIdx = i;
        break;
      }
    }
    if (headless.currentSceneIdx != 0xFF) {
      headless.broadcastBri = overrides.headlessBroadcastBri;
      headlessBroadcastCurrentScene();
      sceneRestored = true;
    }
  }

  // Initial SYNC anchors slave timebases as soon as the master comes up.
  // Skipped when we just broadcast a scene packet — back-to-back
  // scheduleSend would lose the second one to the single-slot TX queue.
  // In that case the first 30 s keepalive tick will fire the SYNC.
  if (!sceneRestored) {
    headlessBroadcastSync(millis());
  }
  // Proactive re-bind: every persisted slave gets a fresh SET_GROUP with
  // its stored ID, staggered so the LoRa duty cycle stays sane and the
  // single-slot TX queue does not drop packets. Slaves that did NOT
  // power-cycle alongside the master (typical: battery-powered race lights
  // staying on through a master swap) regain their pairing without having
  // to re-emit IDENTIFY_REPLY themselves.
  startHeadlessReassign();
}

void UsermodRaceLink::exitHeadlessMode() {
  if (!headless.active && !overrides.headlessPersistedActive) return;
  headless.active  = false;
  headless.probing = false;
  // Role transition out of headless — clear any indicator overlay so the
  // boot-color exit cue below is what the operator sees.
  cancelIndicator();
  // Cancel any pending re-bind sweep so we don't keep paging slaves with a
  // group we no longer own.
  RaceLinkHeadless::abortReassign(headlessReassign);
  // Deactivation is the operator signal to drop the whole pairing context.
  // Counter, slave registry, and our own master groupId all reset so the
  // next Headless promotion starts from a clean slate (Group 2 onward).
  overrides.headlessGroupCounter = 0;
  current.groupId                = 0;
  RaceLinkHeadless::clearSlaveTable(headlessSlaves, headlessSlavesCount,
                                    RaceLinkHeadless::HEADLESS_MAX_SLAVES);
  // Force an immediate write (skip the pairing-burst debounce) — exit is
  // rare and the operator expects "off means off" to survive a battery pull.
  RaceLinkHeadless::persistConsumed(headlessPersist);
  if (overrides.headlessPersistedActive) {
    overrides.headlessPersistedActive = false;
  }
  configNeedsWrite = true;
  // Visual confirmation via the central indicator system: 3-second
  // magenta breathe overlay distinguishable from enter (blue) and
  // probe-rejected (red). Fires for ALL exit paths (manual 5-click,
  // runtime Gateway-takeover via the master-alive detector, OPC_SET_GROUP
  // takeover). After expiry the indicator restores whatever scene the
  // master was last showing — slaves are on that same scene, so the
  // ex-master visually rejoins them.
  applyLocalIndicator(RaceLinkIndicators::IND_HEADLESS_EXIT, 5);
  DEBUG_PRINTLN(F("[RaceLink] Headless: INACTIVE"));
}

void UsermodRaceLink::headlessAdvanceScene() {
  // Wrap from 0xFF (= none) to 0; otherwise step + wrap-around.
  if (headless.currentSceneIdx >= RaceLinkHeadless::SCENE_CATALOG_SIZE) {
    headless.currentSceneIdx = 0;
  } else {
    headless.currentSceneIdx = RaceLinkHeadless::nextSceneIdx(headless.currentSceneIdx);
  }
  const RaceLinkHeadless::HeadlessScene& s
      = RaceLinkHeadless::SCENE_CATALOG[headless.currentSceneIdx];
  overrides.headlessCurrentScene = s.sceneId;
  configNeedsWrite = true;
  headlessBroadcastCurrentScene();
}

void UsermodRaceLink::headlessBroadcastCurrentScene() {
  using namespace RaceLinkProto;
  if (!radioReady || !rl.macReadOK) return;
  if (headless.currentSceneIdx >= RaceLinkHeadless::SCENE_CATALOG_SIZE) return;
  const RaceLinkHeadless::HeadlessScene& s
      = RaceLinkHeadless::SCENE_CATALOG[headless.currentSceneIdx];
  const uint8_t bri8 = headless.broadcastBri ? headless.broadcastBri
                                              : overrides.headlessBroadcastBri;

  uint8_t out[32];

  // Single OPC_HEADLESS broadcast — receivers expand it from their local
  // catalog, including per-group phase offset for SCENE_FLAG_USE_OFFSET
  // rows. We deliberately do NOT pre-emit an OPC_OFFSET here: the
  // transport's single-slot TX queue would reject the second scheduleSend
  // (LBT jitter + ToA easily exceeds the click->click gap), silently
  // dropping the scene packet. The catalog row is the single source of
  // truth on both ends, so empfängerseitiges Expand reicht.
  uint8_t n = RaceLinkHeadless::buildHeadlessPacket(out, rl.myLast3, s.sceneId, bri8);
  // armBlip=false: scene broadcast is routine traffic, not pairing.
  if (n) headlessSendTx(out, n, /*armBlip=*/false);

  // Apply locally so the headless master's own strip mirrors what it
  // just told everyone else to do.
  applyLocalScene(s.sceneId, bri8);

  headless.lastBroadcastAtMs = millis();
  DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: broadcast scene %u (%s) bri=%u\n"),
                 (unsigned)s.sceneId, s.label, (unsigned)bri8);
}

void UsermodRaceLink::headlessBroadcastSync(uint32_t now) {
  using namespace RaceLinkProto;
  if (!radioReady || !rl.macReadOK) return;

  // 4B autosync form (no flags byte) — timebase-only, never fires armed
  // effects. Mirrors the Gateway's idle-period autosync behavior so slaves
  // can adjust strip.timebase against the master clock and keep their
  // masterContactedRecently() gate closed without needing the heavier
  // scene re-broadcast as a keepalive proxy.
  static const uint8_t bcast[3] = { 0xFF, 0xFF, 0xFF };
  uint8_t out[16];
  uint8_t n = build_empty(out, rl.myLast3, bcast, make_type(DIR_M2N, OPC_SYNC));
  const uint32_t ts24 = now & 0x00FFFFFFu;
  out[n++] = (uint8_t)(ts24 & 0xFF);
  out[n++] = (uint8_t)((ts24 >> 8) & 0xFF);
  out[n++] = (uint8_t)((ts24 >> 16) & 0xFF);
  out[n++] = bri;  // current brightness — handleSync uses it only when haveControl && !HAS_BRI

  // SYNC bypasses LBT via scheduleSend's jitterMaxMs=0 universal-bypass
  // branch (see racelink_transport_core.h scheduleSend doc). The precision
  // of the ts24 timestamp dominates the slaves' drift-correction quality
  // and the default LBT 50..300 ms random jitter between our millis()
  // sample and the actual TX would inflate slaves' lastSyncTbErrMs from
  // ~15 ms (Gateway baseline) to ~250 ms. SYNC is broadcast-only and
  // low-frequency (30 s keepalive + on-demand), so the collision-avoidance
  // trade-off is acceptable. Headless-master pairing indicator stays off
  // for SYNC by design (routine traffic, not pairing), so we go directly
  // to scheduleSend rather than through headlessSendTx().
  if (!RaceLinkTransport::scheduleSend(rl, out, n, /*jitterMaxMs=*/0)) return;
  headless.lastBroadcastAtMs = now;

  // Master self-sync: mirror what slaves run in handleSync() for M=S (master
  // is its own clock). Slaves do strip.timebase = (M - S) - activePhaseOffsetMs;
  // with M=S that collapses to strip.timebase = -activePhaseOffsetMs. Without
  // this, the master's strip.timebase keeps whatever value the last
  // setActivePhaseOffsetMs() delta-adjustment left behind — a one-time
  // perturbation by WLED-internal effect transitions / setMode resets stays
  // uncorrected and the master drifts away from the slave fleet over time.
  strip.timebase = (uint32_t)(-(int32_t)activePhaseOffsetMs);
}

// ========= Headless persistence pump =========
// Slave-registry data ops (find/upsert/clear) live in racelink_headless.h
// as free functions in the RaceLinkHeadless namespace — they are WLED-
// neutral and reusable from external Gateway-side code. Callers in this
// file invoke them directly on the headlessSlaves[] member.

void UsermodRaceLink::markHeadlessPersistDirty() {
  RaceLinkHeadless::markPersistDirty(headlessPersist, millis());
}

void UsermodRaceLink::serviceHeadlessPersist(uint32_t now) {
  if (!RaceLinkHeadless::persistDebounceElapsed(
          headlessPersist, now,
          RaceLinkHeadless::HEADLESS_PERSIST_DEBOUNCE_MS)) return;
  configNeedsWrite = true;
  RaceLinkHeadless::persistConsumed(headlessPersist);
}

// ========= Headless scene rebroadcast (debounced one-shot) =========

void UsermodRaceLink::scheduleSceneRebroadcast() {
  // No-op without a current scene — nothing to rebroadcast. Headless master
  // that booted into the no-scene state (currentSceneIdx == 0xFF) has
  // nothing to push; freshly bound slaves stay on their boot color until
  // the operator picks a scene via 1-click.
  if (headless.currentSceneIdx >= RaceLinkHeadless::SCENE_CATALOG_SIZE) return;
  RaceLinkHeadless::scheduleSceneRebroadcast(
      headlessSceneRebroadcast, millis(),
      RaceLinkHeadless::HEADLESS_SCENE_REBROADCAST_DEBOUNCE_MS);
}

void UsermodRaceLink::serviceSceneRebroadcast(uint32_t now) {
  if (!RaceLinkHeadless::sceneRebroadcastReady(headlessSceneRebroadcast, now)) return;
  RaceLinkHeadless::sceneRebroadcastConsumed(headlessSceneRebroadcast);
  // Lost master role mid-debounce — drop the rebroadcast silently.
  if (!headless.active) return;
  headlessBroadcastCurrentScene();
}

// ========= Headless TX wrapper + re-bind sequencer =========

bool UsermodRaceLink::headlessSendTx(const uint8_t* pkt, uint8_t n, bool armBlip) {
  bool ok = RaceLinkTransport::scheduleSend(rl, pkt, n);
  if (!ok || !armBlip || !headless.active) return ok;
  if (!RaceLinkHeadless::shouldFirePairingBlip(
          lastPairingBlipAtMs, millis(),
          RaceLinkHeadless::HEADLESS_PAIRING_INDICATOR_THROTTLE_MS)) {
    return ok;
  }
  applyLocalIndicatorMs(RaceLinkIndicators::IND_PAIRING_TX,
                        RaceLinkHeadless::HEADLESS_PAIRING_INDICATOR_DURATION_MS);
  return ok;
}

void UsermodRaceLink::startHeadlessReassign() {
  // Grace delay: enterHeadlessMode() emits a scene/SYNC broadcast immediately
  // before calling startHeadlessReassign(). Without this delay the first
  // SET_GROUP try fires in the same loop tick and finds the TX queue still
  // occupied by the broadcast (rl.txPending == true) — scheduleSend returns
  // false. The retry-on-busy logic in serviceHeadlessReassign would recover,
  // but the explicit grace delay avoids the noise.
  RaceLinkHeadless::startReassign(
      headlessReassign, headlessSlavesCount,
      millis() + RaceLinkHeadless::HEADLESS_REASSIGN_INTERVAL_MS);
  if (headlessSlavesCount > 0) {
    DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: re-bind sweep over %u slaves\n"),
                   (unsigned)headlessSlavesCount);
  }
}

void UsermodRaceLink::serviceHeadlessReassign(uint32_t now) {
  // Sweep complete? Fire the post-pairing scene rebroadcast once, then idle.
  if (RaceLinkHeadless::reassignSweepCompleted(headlessReassign, headlessSlavesCount)) {
    scheduleSceneRebroadcast();
    RaceLinkHeadless::abortReassign(headlessReassign);
    return;
  }
  // Lost master role or radio dropped mid-sweep — abort.
  if (headlessReassign.cursor != 0xFF
      && (!headless.active || !radioReady || !rl.macReadOK)) {
    RaceLinkHeadless::abortReassign(headlessReassign);
    return;
  }

  const uint8_t idx = RaceLinkHeadless::pickReassignTarget(
      headlessReassign, headlessSlavesCount, now);
  if (idx == 0xFF) return;   // idle / not yet due

  const auto& s = headlessSlaves[idx];
  uint8_t out[32];
  uint8_t n = RaceLinkHeadless::buildSetGroupPacket(out, rl.myLast3, s.addr3, s.groupId);
  if (!n) {
    // Packet builder failure — never expected for SET_GROUP, but if it
    // happens we skip this slot to avoid an infinite loop and move on.
    DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: re-bind skip idx=%u (build failed)\n"),
                   (unsigned)idx);
    RaceLinkHeadless::confirmReassignSent(headlessReassign, now,
        RaceLinkHeadless::HEADLESS_REASSIGN_INTERVAL_MS);
    return;
  }
  if (!headlessSendTx(out, n, /*armBlip=*/true)) {
    // TX queue busy — the initial scene/SYNC broadcast or a preceding slave's
    // ACK is still occupying the single-slot transport. Defer same slot.
    DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: re-bind retry idx=%u (TX busy)\n"),
                   (unsigned)idx);
    RaceLinkHeadless::deferReassignRetry(headlessReassign, now,
        RaceLinkHeadless::HEADLESS_REASSIGN_INTERVAL_MS);
    return;
  }
  RaceLinkHeadless::confirmReassignSent(headlessReassign, now,
      RaceLinkHeadless::HEADLESS_REASSIGN_INTERVAL_MS);
}

void UsermodRaceLink::headlessAssignGroupTo(const uint8_t senderLast3[3],
                                            uint8_t inGroupId) {
  using namespace RaceLinkProto;
  if (!radioReady || !rl.macReadOK) return;

  // Case A: slave reports an existing groupId. Sync our table so a later
  // master reboot can re-bind it proactively, but do NOT send SET_GROUP —
  // re-assigning a working pairing risks a group collision elsewhere.
  if (inGroupId != 0) {
    if (RaceLinkHeadless::upsertSlave(headlessSlaves, headlessSlavesCount,
                                      RaceLinkHeadless::HEADLESS_MAX_SLAVES,
                                      senderLast3, inGroupId)) {
      markHeadlessPersistDirty();
    }
    return;
  }

  // Case B: slave reports groupId=0 (fresh, factory-reset, or pool member).
  // If we already know this MAC, recycle its previous group ID instead of
  // burning a fresh counter slot. Otherwise pull the next free ID.
  int8_t existing = RaceLinkHeadless::findSlaveIdx(headlessSlaves,
                                                   headlessSlavesCount,
                                                   senderLast3);
  uint8_t assigned;
  bool fromCounter = false;
  if (existing >= 0) {
    assigned = headlessSlaves[existing].groupId;
  } else {
    // Reserve the next free ID. Header helper clamps below-range counters
    // up to HEADLESS_FIRST_GROUP_ID and returns 0 on exhaustion.
    assigned    = RaceLinkHeadless::reserveNextGroupId(overrides.headlessGroupCounter);
    fromCounter = true;
    if (assigned == 0) {
      DEBUG_PRINTLN(F("[RaceLink] Headless: group counter exhausted — refusing assignment"));
      return;
    }
    if (!RaceLinkHeadless::upsertSlave(headlessSlaves, headlessSlavesCount,
                                       RaceLinkHeadless::HEADLESS_MAX_SLAVES,
                                       senderLast3, assigned)) {
      // Roll back the counter bump so we don't leave a gap.
      overrides.headlessGroupCounter = assigned;
      DEBUG_PRINTLN(F("[RaceLink] Headless: slave table full — refusing assignment"));
      return;
    }
  }

  uint8_t out[32];
  uint8_t n = RaceLinkHeadless::buildSetGroupPacket(out, rl.myLast3, senderLast3, assigned);
  if (!n) {
    // Roll back the counter bump so we don't leave a gap if we never sent.
    if (fromCounter) overrides.headlessGroupCounter = assigned;
    return;
  }
  headlessSendTx(out, n, /*armBlip=*/true);
  markHeadlessPersistDirty();
  headless.lastBroadcastAtMs = millis();
  // Schedule a one-shot scene rebroadcast so the freshly paired slave snaps
  // to the current visual state instead of staying on its boot color until
  // the operator next changes the scene. Debounced, so a burst of pairings
  // (e.g. multiple slaves powering on within seconds) collapses to one
  // OPC_HEADLESS packet.
  scheduleSceneRebroadcast();
  DEBUG_PRINTF_P(PSTR("[RaceLink] Headless: assigned group %u to %02X:%02X:%02X (%s)\n"),
                 (unsigned)assigned,
                 senderLast3[0], senderLast3[1], senderLast3[2],
                 fromCounter ? "new" : "recycled");
}

void UsermodRaceLink::applyLocalScene(uint8_t sceneId, uint8_t brightness) {
  const RaceLinkHeadless::HeadlessScene* s = RaceLinkHeadless::findSceneById(sceneId);
  if (!s) {
    // Unknown scene-id from a future catalog version — silently drop.
    return;
  }
  // A new scene preempts any running indicator overlay: the scene visual
  // is the new authoritative state, so an end-of-indicator restore would
  // just overwrite it. cancelIndicator drops the active flag without
  // touching the segment — the writes below set the final state.
  cancelIndicator();

  // Per-group phase offset for staggered fleet effects (e.g. Offset Breathe).
  // Computed locally from the catalog row × this device's groupId so the
  // wire surface stays a single OPC_HEADLESS packet — no separate OPC_OFFSET
  // is needed (which would race the scene packet in the single-slot TX
  // queue). Receivers and the Headless master itself run the same math.
  // Non-offset rows snap back to a zero phase offset to clean up after a
  // previous offset scene.
  int32_t newOffsetMs = 0;
  if (s->offsetMode == RaceLinkProto::OFFSET_MODE_LINEAR) {
    int32_t v = (int32_t)s->offsetBase
              + (int32_t)current.groupId * (int32_t)s->offsetStep;
    if (v < 0)        v = 0;
    if (v > 0xFFFF)   v = 0xFFFF;
    newOffsetMs = v;
  }

  if (s->flags & RaceLinkHeadless::SCENE_FLAG_ALL_OFF) {
    bri = 0;
    setActivePhaseOffsetMs(newOffsetMs);
    stateUpdated(CALL_MODE_NO_NOTIFY);
    return;
  }

  if (s->flags & RaceLinkHeadless::SCENE_FLAG_RESTORE_LOCAL) {
    // Each device returns to ITS OWN persisted boot color. setup() guarantees
    // overrides.bootColorMode is in 0..3 (rolling + saving a fresh value on
    // first boot), so applyBootColor() is always well-defined here regardless
    // of whether this device showed its boot color locally (bootPreset == 0).
    applyBootColor();
    bri = brightness ? brightness : briS;
    setActivePhaseOffsetMs(newOffsetMs);
    stateUpdated(CALL_MODE_NO_NOTIFY);
    return;
  }

  // Normal scene: write mode/speed/intensity/color1 directly to the main
  // segment. No fancy fieldMask logic — the catalog row already filtered.
  Segment& seg = strip.getMainSegment();
  seg.setMode(s->fxMode);
  if (s->speed)     seg.speed     = s->speed;
  if (s->intensity) seg.intensity = s->intensity;
  // color1 is 0xRRGGBB; pack to RGBW32 with W=0.
  const uint8_t r = (uint8_t)((s->color1 >> 16) & 0xFF);
  const uint8_t g = (uint8_t)((s->color1 >>  8) & 0xFF);
  const uint8_t b = (uint8_t)((s->color1      ) & 0xFF);
  seg.setColor(0, RGBW32(r, g, b, 0));

  bri = brightness ? brightness : briS;
  setActivePhaseOffsetMs(newOffsetMs);
  stateUpdated(CALL_MODE_NO_NOTIFY);
}

// ===================== Indicators =====================
// Catalog + state struct live in racelink_indicators.h so external Gateway
// software (FPVGate) can build OPC_INDICATE packets via the same shared
// definitions. The local-apply / overlay-render logic is WLED-specific
// and stays here.
//
// Rendering: the indicator is painted as a frame-buffer overlay in
// handleOverlayDraw(), which WLED calls after every segment effect has
// rendered + blended, immediately before strip.show(). The underlying
// effect (Traffic Light, Palette, Fireworks, …) keeps running
// untouched — its SEGENV state, palette, colours, and any heap data
// remain intact for the full indicator duration, so fleet phase sync
// is preserved automatically with no catch-up burst on exit.

void UsermodRaceLink::applyLocalIndicator(uint8_t type, uint8_t durationSec) {
  if (durationSec == 0) {
    // Wire-level cancel: stop overlay immediately; the underlying
    // effect's pixels become visible on the next strip.show().
    cancelIndicator();
    return;
  }

  const RaceLinkIndicators::IndicatorDef* d = RaceLinkIndicators::findIndicator(type);
  if (!d) return; // unknown type — silently drop (forward compat)

  // Capture the catalog values the overlay renderer needs each frame.
  // No segment mutation here — handleOverlayDraw() does all painting.
  indicator.active            = true;
  indicator.expiresAtMs       = millis() + (uint32_t)durationSec * 1000u;
  indicator.activeColor1      = d->color1;
  indicator.activeSpeed       = d->speed;
  indicator.activeBrightness  = d->brightness;

  DEBUG_PRINTF_P(PSTR("[RaceLink] Indicator: type=%u dur=%us (%s)\n"),
                 (unsigned)d->type, (unsigned)durationSec, d->label);
}

void UsermodRaceLink::applyLocalIndicatorMs(uint8_t type, uint32_t durationMs) {
  if (durationMs == 0) {
    cancelIndicator();
    return;
  }
  const RaceLinkIndicators::IndicatorDef* d = RaceLinkIndicators::findIndicator(type);
  if (!d) return;
  indicator.active            = true;
  indicator.expiresAtMs       = millis() + durationMs;
  indicator.activeColor1      = d->color1;
  indicator.activeSpeed       = d->speed;
  indicator.activeBrightness  = d->brightness;
}

void UsermodRaceLink::serviceIndicator(uint32_t now) {
  if (!indicator.active) return;
  if ((int32_t)(now - indicator.expiresAtMs) < 0) return;
  // Expired: just drop the flag. The underlying effect was never
  // disturbed; its pixels are already correct, so the next
  // strip.show() — running without the overlay overwrite — shows
  // them immediately. No setMode / setColor / setPalette restore
  // required.
  indicator.active = false;
}

void UsermodRaceLink::cancelIndicator() {
  // Drop the overlay; underlying effect surfaces on the next frame.
  indicator.active = false;
}

// Called by strip's show-callback after all segment effects have been
// rendered and blended, just before pixels are pushed to hardware.
// We overwrite the main segment's pixels for the strobe's on-frame and
// blank them for the off-frame — a covered overlay that's visually
// identical to the legacy setMode(STROBE) implementation but without
// touching any segment state. The underlying effect renders normally
// each frame; we simply replace its output for the indicator duration.
void UsermodRaceLink::handleOverlayDraw() {
  if (!indicator.active) return;

  // Strobe waveform mirrored from FX.cpp blink() (with strobe=true)
  // so the catalog ``speed`` field produces the same visual tempo as
  // the legacy STROBE-effect path did. cycleTime grows with lower
  // speed; onTime is one frame (~16 ms) for a sharp strobe pulse.
  const uint32_t cycleTime = ((uint32_t)(255 - indicator.activeSpeed)) * 20u
                           + (uint32_t)FRAMETIME * 2u;
  const uint32_t rem       = strip.now % cycleTime;
  const bool     onFrame   = rem < (uint32_t)FRAMETIME;

  // Pre-scale catalog colour by catalog brightness. We can't write the
  // global ``bri`` (would also dim the underlying effect's frame); the
  // per-pixel pre-scale gives an equivalent visual under the global
  // bri pipeline that runs later in strip.show().
  uint32_t paintColor = 0;
  if (onFrame) {
    const uint32_t c   = indicator.activeColor1;
    const uint32_t bri8 = indicator.activeBrightness;
    const uint8_t r = (uint8_t)((((c >> 16) & 0xFF) * bri8) / 255u);
    const uint8_t g = (uint8_t)((((c >>  8) & 0xFF) * bri8) / 255u);
    const uint8_t b = (uint8_t)((((c      ) & 0xFF) * bri8) / 255u);
    paintColor = RGBW32(r, g, b, 0);
  } // else paintColor stays 0 = black (covered overlay)

  // Write directly into the strip frame-buffer via strip.setPixelColor(),
  // NOT via seg.setPixelColor(). The segment-relative writer targets the
  // segment's own pixel array, which has already been blended into the
  // strip frame-buffer (FX_fcn.cpp:1648-1650) BEFORE this callback fires
  // — segment-level writes here would land in a dead buffer that's never
  // pushed to hardware. Same pattern as the built-in analog clock overlay
  // (overlay.cpp:44-49).
  Segment& seg = strip.getMainSegment();
  if (seg.stop == 0) return; // invalid segment guard
  for (unsigned i = seg.start; i < seg.stop; ++i) {
    strip.setPixelColor(i, paintColor);
  }
}

// construct & register usermod
static UsermodRaceLink racelink_wled;
REGISTER_USERMOD(racelink_wled);
