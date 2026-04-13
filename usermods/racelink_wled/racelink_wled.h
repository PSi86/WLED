#pragma once

#include "wled.h"
#include "racelink_transport_core.h"
//#include "racelink_proto.h"

// ===================== RaceLink / RadioLib =====================
//#include <RadioLib.h>
extern "C" {
  #include "esp_system.h"
  //#include "esp_wifi.h"
  #include "esp_mac.h"   // for esp_read_mac / ESP_MAC_WIFI_STA
}

#define FW_VERSION 2 // Version of the RaceLink WLED Plugin

#ifndef DEV_TYPE
  #define DEV_TYPE 10 // WLED Device type for RaceLink
#endif

using GateCore = RaceLinkProto::P_Control;  // 4B: groupId, flags, presetId, brightness

// ===================== WLED Sync over RaceLink =====================
//
// Preset-based, RX-only nodes:
// - CONFIG packet: arm preset + optional flags/brightness, but DO NOT start rendering yet.
// - SYNC packet: carries a compact 24-bit millisecond timestamp from master. On first SYNC after CONFIG
//   the node applies the pending preset and aligns WLED's effect timebase. Subsequent SYNC packets
//   only adjust the timebase (optionally slew-limited) to keep effects in phase even if SYNC
//   arrives irregularly.
//
// Important: The numeric opcode for SYNC must match your master implementation.
// If your protocol header already defines OPC_SYNC, that value will be used.

// Slew limit for timebase correction (ms per received SYNC). Keeps visual jitter <~50ms.
#ifndef RACELINK_SYNC_MAX_STEP_MS
  #define RACELINK_SYNC_MAX_STEP_MS 15
#endif

// If phase error exceeds this threshold, snap timebase hard (ms).
#ifndef RACELINK_SYNC_HARD_RESYNC_MS
  #define RACELINK_SYNC_HARD_RESYNC_MS 200
#endif

// Call mode used when applying changes from RaceLink (avoid UDP/WiFi notifications)
#ifndef CALL_MODE_NO_NOTIFY
  #define CALL_MODE_NO_NOTIFY CALL_MODE_DIRECT_CHANGE
#endif

// CONFIG packet uses the existing 4B P_Control layout, but interprets the fields as:
//   groupId    : group selector
//   flags      : flags bitfield (see below)
//   presetId   : presetId (1..250)
//   brightness : brightness (0..255)
//
// Bitfield for CONFIG.flags (stored in P_Control.flags)
enum GateCfgFlags : uint8_t {
  RACELINK_FLAG_POWER_ON        = 1u << 0,  // desired power state (1=on). If 0 -> bri forced to 0 on start.
  RACELINK_FLAG_ARM_ON_SYNC     = 1u << 1,  // if set: do not apply preset until first SYNC arrives
  RACELINK_FLAG_HAS_BRI         = 1u << 2,  // if set: CONFIG includes explicit brightness; if not set: brightness may come from SYNC (live)
  RACELINK_FLAG_FORCE_TT0       = 1u << 3,  // if set: force transition delay to 0 when starting the preset
  RACELINK_FLAG_FORCE_REAPPLY   = 1u << 4,  // if set: re-apply preset even if it's already active
  RACELINK_FLAG_OFFSET_MODE     = 1u << 5,  // if set: respect the groupDelay and groupQuotient (enable timebase offsets between groups, e.g. for staggered effects)
};

// StatusReply config byte flags
enum GateStatusCfgFlags : uint8_t {
  RACELINK_CFG_MAC_FILTER_ENABLED  = 1u << 0,
  RACELINK_CFG_MAC_FILTER_PERSIST  = 1u << 1,
  RACELINK_CFG_AP_ACTIVE           = 1u << 2,
};

// README WLAN Behaviour in WLED
// wled.h (~Line 380):  WLED_GLOBAL byte apBehavior      _INIT(AP_BEHAVIOR_BUTTON_ONLY); // modified for WLED RaceLink Gates
// wled.h (~Line 380):  WLED_GLOBAL byte apBehavior      _INIT(AP_BEHAVIOR_BOOT_NO_CONN); // Original for ESP32
// use AP_BEHAVIOR_TEMPORARY instead of AP_BEHAVIOR_BUTTON_ONLY to start AP on no-WiFi at boot without button?
// Button Only: Start AP only on button press (5s)
// Short Button press: toogle on/off
// Long Button press (600ms): Toggle Preset


// ------------ HT-CT62 (ESP32-C3 + SX1262) fixed pins ------------
#ifndef RACELINK_PIN_MOSI
  #define RACELINK_PIN_MOSI   7
#endif
#ifndef RACELINK_PIN_MISO
  #define RACELINK_PIN_MISO   6
#endif
#ifndef RACELINK_PIN_SCK
  #define RACELINK_PIN_SCK    10
#endif
#ifndef RACELINK_PIN_NSS
  #define RACELINK_PIN_NSS    8   // strapping pin on HT-CT62 (already wired to CS)
#endif
#ifndef RACELINK_PIN_DIO1
  #define RACELINK_PIN_DIO1   3
#endif
#ifndef RACELINK_PIN_BUSY
  #define RACELINK_PIN_BUSY   4
#endif
#ifndef RACELINK_PIN_RST
  #define RACELINK_PIN_RST    5
#endif

// ------------ RaceLink RaceLink defaults (EU868) ------------
#ifndef RACELINK_FREQ_HZ
  #define RACELINK_FREQ_HZ    868000000UL
#endif
#ifndef RACELINK_BW_KHZ
  #define RACELINK_BW_KHZ     125.0
#endif
#ifndef RACELINK_SF
  #define RACELINK_SF         7
#endif
#ifndef RACELINK_CR
  #define RACELINK_CR         5      // 4/5
#endif
#ifndef RACELINK_SYNC_WORD
  #define RACELINK_SYNC_WORD  0x12
#endif
#ifndef RACELINK_TX_POWER
  #define RACELINK_TX_POWER   14     // dBm
#endif
#ifndef RACELINK_PREAMBLE
  #define RACELINK_PREAMBLE   8
#endif


class UsermodRaceLink : public Usermod {
public:
  // ----- WLED Usermod API -----
  void setup() override;
  void loop() override;
  void addToJsonInfo(JsonObject& root) override;
  void addToConfig(JsonObject& root) override;
  bool readFromConfig(JsonObject& root) override;
  void onStateChange(uint8_t mode) override;
  uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

private:
  static constexpr uint8_t STREAM_BUFFER_SIZE = 128;
  static constexpr uint8_t STREAM_CHUNK_SIZE = 8;
  static constexpr uint8_t STREAM_MAX_PACKETS = STREAM_BUFFER_SIZE / STREAM_CHUNK_SIZE;

  // Radio / SPI
  SPIClass* spi = &SPI;
  #if defined(RACELINK_SX1262)
    SX1262* radio = nullptr;
  #elif defined(RACELINK_LLCC68)
    LLCC68* radio = nullptr;
  #else
    #error "No RaceLink radio module defined"
  #endif
  

  //UsermodBattery* bat = nullptr;
  um_data_t* batteryUM = nullptr;

  // Node identity
  // now in RaceLink::rl struct
/*   uint8_t myMac6[6] = {0};
  uint8_t myLast3[3] = {0};
  bool macReadOK = false; */

  // Config / State
  GateCore current{};  // init in setup()

  // ===== WLED preset-sync state =====
  struct PendingCfg {
    bool    armed = false;       // waiting for SYNC to start
    uint8_t presetId = 0;        // WLED preset ID
    uint8_t flags = 0;           // GateCfgFlags bitfield
    uint8_t bri = 0;             // desired brightness (if RACELINK_FLAG_HAS_BRI)
    uint32_t rxAtMs = 0;         // when CONFIG was received
  } pending;

  bool haveControl = false;   // set true after first CONTROL/CONFIG packet

  // last received SYNC phase (for unwrap/filter)
  bool     haveSync = false;
  uint32_t masterEpochAbsMs = 0; // unwrapped master timestamp in ms (32-bit, derived from 24-bit ts24)
  uint32_t lastSyncLocalMs = 0;  // local millis() when last SYNC was processed
  int32_t  lastSyncTbErrMs = 0;  // last timebase error (desiredTb - strip.timebase) in ms (debug)

  // Master filter options
  bool macFilterEnabled = true;   // default: ON
  bool macFilterPersist = false;  // default: OFF
  bool masterKnown = false;
  uint8_t masterLast3[3] = {0};
  uint8_t masterFull6[6] = {0};
  bool   masterFull6Known = false;

  // Discovery reply target
  uint8_t targetForReplyLast3[3] = {0};

  #if DEV_TYPE == 50
    uint8_t numberOfSlots = 1;
    uint8_t firstSlot = 1;
  #endif

  // ===== Debug/Diag =====
  bool    radioReady = false;
  int16_t radioInitCode = 0;
  //uint32_t rxTotal = 0;      // all packets successfully read
  uint32_t rxAccepted = 0;   // packets that passed filters AND triggered actions (difers from rxTotal and rxCountFiltered in racelink core)
  //int16_t  lastRssi = 0;     // last packet RSSI (approx)
  //int8_t   lastSnr  = 0;     // last SNR (if available)

  // Helpers
  //void readEfuseMac();
  //static bool isBroadcast3(const uint8_t last3[3]);
  //bool receiverMatchesMe(const uint8_t r3[3]);
  bool senderAllowed(const uint8_t s3[3], uint8_t opcode7);
  void learnMasterFromSender(const uint8_t s3[3], bool persistIfEnabled);
  void persistMasterIfNeeded();
  void clearMaster();

  // Radio helpers
  bool radioInit();
  //void radioStartRx();
  void handlePacket(const uint8_t* buf, size_t len);
  //void sendPacket(const uint8_t* data, size_t len);

  static void on_rx_node(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx);
  static void on_tx_done_node(void* ctx);

  void sendAckTo(const uint8_t destLast3[3], uint8_t echoOpcode7, RaceLinkProto::AckStatus st);
  void sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac);
  void sendStatusReplyTo(const uint8_t destLast3[3]);

  // Build frames

/*   void buildCoreFromCurrent(GateCore& out) {
    out.deviceType = THIS_TYPE;
    out.groupId    = current.groupId;
    out.flags     = current.flags;
    out.presetId  = current.presetId;
    out.brightness = current.brightness;
  } */
  void buildCoreFromCurrent(GateCore& out) { out = current; }  // oder komplett entfernen TODO: noch nötig?

  // Apply CONTROL like original ESPNOW logic
  void applyControl(const GateCore& in); // legacy immediate apply (kept for compatibility)

  // New preset-sync handlers
  void handleControl(const GateCore& cfg);
  void handleSync(uint32_t ts24, uint8_t briFromPkt);

  bool handleStreamPacket(const uint8_t* buf, uint8_t len, const uint8_t senderLast3[3]);
};
