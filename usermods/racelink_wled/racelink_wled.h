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

// AdvancedFields: parsed body of an OPC_CONTROL packet (variable-length wire
// format described near P_Control in racelink_proto.h). fieldMask/extMask
// mirror the wire layout — a bit set means the value was explicitly carried
// by the most recent CONTROL. After a preset load both masks are set to
// "all known" because the segment now has a defined value for every field.
struct AdvancedFields {
  uint8_t fieldMask = 0;
  uint8_t extMask   = 0;
  uint8_t brightness = 0;
  uint8_t mode = 0;
  uint8_t speed = 0;
  uint8_t intensity = 0;
  uint8_t custom1 = 0;
  uint8_t custom2 = 0;
  uint8_t custom3 = 0;      // 0..31 (5 bits)
  bool    check1 = false;
  bool    check2 = false;
  bool    check3 = false;
  uint8_t palette = 0;
  uint32_t color1 = 0;      // RGBW (W=0)
  uint32_t color2 = 0;
  uint32_t color3 = 0;
};

// DeviceState: full snapshot of what this device looks like right now.
// Replaces the old P_Preset-aliased GateCore. Single source of truth for
// STATUS replies and any future OPC_STATUS_EXT.
//   - flags is written ONLY by OPC_CONTROL handling (single-writer rule).
//     OPC_PRESET still carries a flag byte on the wire but it is ignored.
//   - fields holds the last applied / observed effect parameters; refreshed
//     from the WLED segment after a preset load and via onStateChange.
struct DeviceState {
  uint8_t  groupId    = 0;
  uint8_t  flags      = 0;
  uint8_t  presetId   = 0;
  uint8_t  brightness = 0;
  AdvancedFields fields{};
  uint32_t lastUpdateMs = 0;
};

using GateCore = DeviceState;  // alias kept for existing call sites

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

// Master-Quiet-Gate: physical-button gestures (color/brightness) only fire
// when the device has not received a packet from the paired master for at
// least this many ms. The triple-tap hotspot recovery ignores this gate.
#ifndef RACELINK_MASTER_QUIET_MS
  #define RACELINK_MASTER_QUIET_MS 60000
#endif

// Hold threshold (= WLED_LONG_PRESS) for the GPIO 0 button.
#ifndef RACELINK_BTN_LONG_MS
  #define RACELINK_BTN_LONG_MS 600
#endif

// Max gap between successive short clicks counted as a multi-tap burst.
#ifndef RACELINK_BTN_MULTI_WINDOW_MS
  #define RACELINK_BTN_MULTI_WINDOW_MS 500
#endif

// Brightness step per fade tick while the button is held.
#ifndef RACELINK_BTN_BRI_STEP
  #define RACELINK_BTN_BRI_STEP 4
#endif

// Minimum interval between brightness fade updates while held.
#ifndef RACELINK_BTN_FADE_TICK_MS
  #define RACELINK_BTN_FADE_TICK_MS 30
#endif

// Idle window for the single-click colour cycle. After this many ms with
// no short-click, the next click restarts the R→G→B→random sequence at R.
#ifndef RACELINK_BTN_COLOR_RESET_MS
  #define RACELINK_BTN_COLOR_RESET_MS 10000
#endif

// PRESET packet (OPC_PRESET, 4B P_Preset):
//   groupId    : group selector
//   flags      : IGNORED by WLED (still on the wire for host compatibility).
//                Flags are processed only from OPC_CONTROL — single writer rule.
//                Consequence: PRESET cannot be armed and cannot drive
//                FORCE_TT0 / FORCE_REAPPLY. Use OPC_CONTROL for those.
//   presetId   : presetId (1..250) — applied immediately on receipt.
//   brightness : brightness (0..255) — applied immediately on receipt.
//
// Bitfield carried by OPC_CONTROL.flags (and also by OPC_PRESET.flags on the
// wire, where it is ignored). Single host-side definition: flags.py.
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


// ------------ Radio pin defaults (overridable via build_flags or usermod config) ------------
// Defaults below match HT-CT62 (ESP32-C3 + SX1262). Per-target build profiles override
// via -D RACELINK_PIN_*=... so first boot picks the correct pins per hardware variant.
// The runtime usermod settings (UsermodRaceLink::pin*) read these as initial values
// and persist any UI override into cfg.json — see addToConfig/readFromConfig.
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

// ------------ ePaper pin defaults (only when -D RACELINK_EPAPER is set) ------------
// Mirror the defaults in racelink_epaper.cpp so the UsermodRaceLink members can use
// them as initializers. The .cpp keeps its own #ifndef guards in case this header
// is not included there (racelink_epaper.cpp only pulls in racelink_epaper.h).
#ifdef RACELINK_EPAPER
  #ifndef RACELINK_EPAPER_CS
    #define RACELINK_EPAPER_CS   10
  #endif
  #ifndef RACELINK_EPAPER_BUSY
    #define RACELINK_EPAPER_BUSY 3
  #endif
  #ifndef RACELINK_EPAPER_RST
    #define RACELINK_EPAPER_RST  46
  #endif
  #ifndef RACELINK_EPAPER_DC
    #define RACELINK_EPAPER_DC   9
  #endif
  #ifndef RACELINK_EPAPER_MOSI
    #define RACELINK_EPAPER_MOSI 11
  #endif
  #ifndef RACELINK_EPAPER_SCK
    #define RACELINK_EPAPER_SCK  12
  #endif
  #ifndef RACELINK_EPAPER_MISO
    #define RACELINK_EPAPER_MISO -1
  #endif
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

// ------------ Fleet-uniformity defaults (applied on every boot) ------------
// These values are re-applied by applyRaceLinkDefaults() in setup() after WLED
// has deserialised cfg.json — overriding any per-device drift introduced via
// the WLED UI. This is the runtime mechanism that previously was attempted via
// `-D WLED_FPS=...` build flag but doesn't work without modifying upstream
// FX.h (the source-level `#define WLED_FPS 42` would silently win over -D).
//
// Each macro is `#ifndef`-guarded so a build profile can override per-platform
// (e.g. to test a different FPS for a hardware variant) without editing this
// header. The defaults below match the operator-validated 2026-05-08 fleet
// settings that resolved the V3↔V4 Strobe drift (Issue 3).
//
// Future direction: OPC_CONFIG (option codes 0x05+) will let the master adjust
// these at runtime per device. For now they are pure boot-time enforcement —
// any UI change is reverted on next reboot, which is intentional.
#ifndef RACELINK_DEFAULT_FPS
  #define RACELINK_DEFAULT_FPS         75   // Target refresh rate (0..250). 0 = unlimited (NOT recommended for sync groups).
#endif
#ifndef RACELINK_DEFAULT_ABL_MAX_MA
  #define RACELINK_DEFAULT_ABL_MAX_MA  0    // Brightness limiter budget in mA. 0 = ABL disabled.
#endif
#ifndef RACELINK_DEFAULT_GAMMA_COL
  #define RACELINK_DEFAULT_GAMMA_COL   true // gammaCorrectCol — gamma on color
#endif
#ifndef RACELINK_DEFAULT_GAMMA_BRI
  #define RACELINK_DEFAULT_GAMMA_BRI   false // gammaCorrectBri — gamma on brightness
#endif
#ifndef RACELINK_DEFAULT_GAMMA_VAL
  #define RACELINK_DEFAULT_GAMMA_VAL   2.2f // gammaCorrectVal — gamma exponent (clamped to [0.1, 3.0])
#endif
#ifndef RACELINK_DEFAULT_AP_BEHAVIOR
  // 3 = AP_BEHAVIOR_BUTTON_ONLY = "Never (not recommended)" in WLED's WiFi
  // settings UI. Prevents auto-opening the AP on every boot, which is the
  // WLED post-factory-reset default (AP_BEHAVIOR_BOOT_NO_CONN = 0). The
  // RaceLink usermod's triple-tap gesture calls WLED::instance().initAP(true)
  // directly, bypassing apBehavior, so hotspot recovery still works.
  #define RACELINK_DEFAULT_AP_BEHAVIOR  AP_BEHAVIOR_BUTTON_ONLY
#endif

// "Reset to RaceLink defaults" target values for the override fields the
// usermod always enforces (briS, transition, segments). applyRaceLinkDefaults()
// applies these on every boot AND on every WebUI/OPC_CONFIG change — RaceLink
// is the sole source of truth for these settings. OPC_CONFIG 0x0F resets the
// override slots back to these compile-time values.
#ifndef RACELINK_DEFAULT_BRIS
  #define RACELINK_DEFAULT_BRIS         128  // Default boot brightness (briS, 0..255).
#endif
#ifndef RACELINK_DEFAULT_TRANSITION_MS
  #define RACELINK_DEFAULT_TRANSITION_MS 700 // Default transition duration in ms.
#endif

// Segment geometry defaults. seg0 spans LEDs 0..SEG0_STOP on a fresh install
// (or 0x0F reset). seg1 sentinel: seg1Start == seg1Stop means "seg[1] not
// configured" — applyRaceLinkDefaults() then skips appendSegment/setGeometry,
// matching WLED's own resetSegments() result of one segment over the strip.
// Adjust SEG0_STOP per build profile if the typical strip length differs.
#ifndef RACELINK_DEFAULT_SEG0_START
  #define RACELINK_DEFAULT_SEG0_START  0
#endif
#ifndef RACELINK_DEFAULT_SEG0_STOP
  #define RACELINK_DEFAULT_SEG0_STOP   200
#endif
#ifndef RACELINK_DEFAULT_SEG1_START
  #define RACELINK_DEFAULT_SEG1_START  0
#endif
#ifndef RACELINK_DEFAULT_SEG1_STOP
  #define RACELINK_DEFAULT_SEG1_STOP   0
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
  bool handleButton(uint8_t b) override;
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

  // Pin assignments (defaults from build flags above; runtime-overridable via
  // usermod config). See addToConfig/readFromConfig for the JSON schema.
  // A pin change in the UI sets the global doReboot flag; SPI is re-init'd at
  // boot using the saved values.
  int8_t pinSck  = RACELINK_PIN_SCK;
  int8_t pinMiso = RACELINK_PIN_MISO;
  int8_t pinMosi = RACELINK_PIN_MOSI;
  int8_t pinNss  = RACELINK_PIN_NSS;
  int8_t pinDio1 = RACELINK_PIN_DIO1;
  int8_t pinBusy = RACELINK_PIN_BUSY;
  int8_t pinRst  = RACELINK_PIN_RST;

  #ifdef RACELINK_EPAPER
    // ePaper SPI bus + control pins. CS/DC/RST/BUSY are written into the
    // GxEPD2 display object during epaperInit(); SCK/MISO/MOSI configure
    // the dedicated HSPI bus.
    int8_t epdSck  = RACELINK_EPAPER_SCK;
    int8_t epdMiso = RACELINK_EPAPER_MISO;
    int8_t epdMosi = RACELINK_EPAPER_MOSI;
    int8_t epdCs   = RACELINK_EPAPER_CS;
    int8_t epdDc   = RACELINK_EPAPER_DC;
    int8_t epdRst  = RACELINK_EPAPER_RST;
    int8_t epdBusy = RACELINK_EPAPER_BUSY;
  #endif

  // True until the first readFromConfig() call returns. Used to suppress
  // the reboot-on-pin-change behavior during the initial cfg.json deserialize
  // (which always runs before setup()), so a fresh boot doesn't reboot itself.
  bool firstReadFromConfig = true;

  // ===== RaceLink-authoritative overrides (always active) =====
  // RaceLink is the single source of truth for these six settings. Every
  // override slot is always populated (compile-time RACELINK_DEFAULT_* on
  // first boot, then whatever the host or operator most recently saved).
  // applyRaceLinkDefaults() unconditionally pushes the slot values into
  // the live WLED globals on every boot AND after every cfg.json reload
  // (WebUI Save → readFromConfig → applyRaceLinkDefaults). OPC_CONFIG
  // 0x05..0x0A write a single slot; 0x0F resets every slot to its
  // RACELINK_DEFAULT_*. Persisted as the "RaceLink.overrides" sub-object
  // by addToConfig/readFromConfig.
  struct RaceLinkOverrides {
    uint8_t  fps           = (uint8_t)RACELINK_DEFAULT_FPS;
    uint16_t ablMaxMa      = (uint16_t)RACELINK_DEFAULT_ABL_MAX_MA;
    uint16_t seg0Start     = (uint16_t)RACELINK_DEFAULT_SEG0_START;
    uint16_t seg0Stop      = (uint16_t)RACELINK_DEFAULT_SEG0_STOP;
    uint16_t seg1Start     = (uint16_t)RACELINK_DEFAULT_SEG1_START;
    uint16_t seg1Stop      = (uint16_t)RACELINK_DEFAULT_SEG1_STOP;
    uint8_t  briS          = (uint8_t)RACELINK_DEFAULT_BRIS;
    uint16_t transitionMs  = (uint16_t)RACELINK_DEFAULT_TRANSITION_MS;
  } overrides;

  //UsermodBattery* bat = nullptr;
  um_data_t* batteryUM = nullptr;

  // Node identity
  // now in RaceLink::rl struct
/*   uint8_t myMac6[6] = {0};
  uint8_t myLast3[3] = {0};
  bool macReadOK = false; */

  // Config / State
  GateCore current{};  // init in setup()

  // ===== Pending CONTROL (queued, awaiting SYNC) =====
  // Only OPC_CONTROL can arm. OPC_PRESET applies immediately and never
  // produces pending state. The "armed" condition is derived:
  //   armed == (pending.valid && (current.flags & RACELINK_FLAG_ARM_ON_SYNC))
  // Cleared by either: SYNC firing (apply + valid=false) or the next CONTROL
  // arriving with ARM_ON_SYNC=0 (immediate-apply branch sets valid=false).
  // When the arming CONTROL has OFFSET_MODE set, ``pending.offsetMs`` is
  // snapshotted from the active offset value at arm time so a later
  // OPC_OFFSET cannot retroactively shift this queued effect.
  struct PendingControl {
    AdvancedFields fields{};
    uint32_t rxAtMs   = 0;
    uint16_t offsetMs = 0;
    bool     valid    = false;
  } pending;

  // ===== Deferred apply (post-SYNC, awaiting offset deadline) =====
  // When a SYNC fires with a non-zero pending.offsetMs, the node copies the
  // queued state here and schedules ``millis() + offsetMs`` as the apply
  // deadline. ``serviceDeferredApply()`` (called from loop()) replays the
  // exact apply path the SYNC handler used to run inline.
  bool           pendingDeferred         = false;
  uint32_t       pendingDeferredAt       = 0;   // local millis() one-shot deadline
  uint8_t        pendingDeferredFlags    = 0;   // snapshot of current.flags at SYNC time
  uint8_t        pendingDeferredBri      = 0;   // briFromPkt captured at SYNC
  AdvancedFields pendingDeferredFields{};
  uint16_t       pendingDeferredOffsetMs = 0;   // captured pending.offsetMs; consumed by serviceDeferredApply

  // ===== Persistent per-device phase offset =====
  // Subtracted from strip.timebase after every SYNC so cyclic effects
  // (Breathe, Pacifica, anything using strip.now directly in a sin/beat
  // wave) maintain a stable phase difference vs other groups. Without
  // this, SYNC re-aligns strip.timebase identically on every device and
  // direct-time-based effects phase-lock — even when start times were
  // staggered via offset mode. Updated whenever an effect is applied via
  // the offset-mode path; reset to 0 when a non-offset effect is applied.
  int32_t activePhaseOffsetMs = 0;

  // ===== Offset acceptance gate + formula =====
  // The device runs in one of two states gating which OPC_CONTROL/OPC_PRESET
  // packets it accepts:
  //   * mode == NONE  — accept only packets with OFFSET_MODE flag = 0
  //   * mode != NONE  — accept only packets with OFFSET_MODE flag = 1
  //
  // OPC_OFFSET writes the requested formula into ``pendingChange``. The
  // pending change materialises into ``active`` at the next OPC_CONTROL /
  // OPC_PRESET that is accepted under the *effective* config (= pending if
  // valid, else active). For arm-on-sync packets, materialisation defers
  // to the SYNC that fires the queued effect — the runner exploits this
  // to send one broadcast OPC_CONTROL to all groups while only the
  // offset-configured ones accept it.
  //
  // ``computeOffsetMs`` evaluates the stored formula against this device's
  // current.groupId; the result is snapshotted into ``pending.offsetMs``
  // at arm time (handleControl) so a later OPC_OFFSET cannot retroactively
  // shift an already-armed effect. The Python mirror lives in
  // ``racelink/domain/offset_formula.py`` and the contract is locked down
  // by tests there.
  struct OffsetConfig {
    uint8_t  mode        = RaceLinkProto::OFFSET_MODE_NONE;  // OffsetMode
    int16_t  base_ms     = 0;
    int16_t  step_ms     = 0;
    uint8_t  center      = 0;     // VSHAPE
    uint8_t  cycle       = 1;     // MODULO (1..255; 0 treated as 1)
    uint16_t explicit_ms = 0;     // EXPLICIT
  };
  OffsetConfig active;
  OffsetConfig pendingChange;
  bool         pendingChangeValid = false;

  bool haveControl = false;   // set true after first PRESET or CONTROL packet

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

  // Session-only master-contact tracking (NOT persisted) — drives the
  // physical-button master-quiet gate. lastMasterRxMs is meaningful only
  // when anyMasterRxSinceBoot is true.
  //
  // Why not reuse rl.lastRxAtMs (transport core)? That field is ticked in
  // the transport before the senderAllowed() filter — it captures any
  // packet whose receiver matches us, including OPC_DEVICES broadcasts
  // from a stranger gateway in the next room (different paired master)
  // or from any gateway during pre-pairing discovery. Using it here would
  // close the button gate on those packets even though they have no
  // RaceLink effect on this node. lastMasterRxMs is set only in
  // learnMasterFromSender() and in handlePacket() guarded by
  //   masterKnown && same3(h.sender, masterLast3)
  // so it strictly tracks "our paired master is active right now".
  //
  // anyMasterRxSinceBoot disambiguates "never heard from master" from
  // "60 s elapsed since contact" — without it, lastMasterRxMs == 0 at
  // boot would falsely register as "master spoke 0 ms ago" for the first
  // 60 s and lock the button on a fresh power-up.
  uint32_t lastMasterRxMs = 0;
  bool     anyMasterRxSinceBoot = false;

  // Custom button state for GPIO 0 (replaces WLED's default short/long/double
  // mapping). Driven from our own loop() via pollPhysicalButton() so we sample
  // the pin at full loop rate — WLED's UsermodManager::handleButton() dispatch
  // is throttled to ~250 ms while strip.isUpdating(), which loses 3-click
  // bursts and adds noticeable lag to release detection.
  struct BtnState {
    bool     down              = false;
    bool     longHandled       = false; // hold passed RACELINK_BTN_LONG_MS, fade running
    bool     briDirUp          = true;  // current ping-pong direction; flips on hitting bri limits
    uint32_t pressedAtMs       = 0;
    uint32_t lastFadeTickMs    = 0;
    uint8_t  pendingShortClicks = 0;    // accumulates within RACELINK_BTN_MULTI_WINDOW_MS
    uint32_t lastShortReleaseMs = 0;
    // Single-click colour cycle. The fixed primary order is R(0)→G(1)→B(2)
    // wrapped as a ring buffer; boot picks a random starting index from
    // {0,1,2} and the click cycle advances `nextPrimaryIdx = (idx+1) % 3`
    // so all three primaries are guaranteed reachable regardless of where
    // the boot pick landed (e.g. boot=G → click 1=B → click 2=R → click 3+=
    // random). primariesShownCount tracks how many primaries have already
    // been displayed in the current cycle (including the boot colour); once
    // it reaches 3 the click action switches to random RGB. After
    // RACELINK_BTN_COLOR_RESET_MS without a short-click the cycle is
    // re-seeded with a fresh random starting index on the next click.
    uint8_t  nextPrimaryIdx       = 0;
    uint8_t  primariesShownCount  = 0;
    uint32_t lastColorClickMs     = 0;
  } btn;

  // Discovery reply target
  uint8_t targetForReplyLast3[3] = {0};
  uint8_t startupIdentifyStage = 0;
  uint32_t startupIdentifyAtMs[2] = {0, 0};

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

  // Push every RaceLink-authoritative override slot into the live WLED
  // globals (`_targetFps`, `BusManager::_gMilliAmpsMax`, `briS`,
  // `transitionDelayDefault`, segment geometry) and enforce the
  // RACELINK_DEFAULT_* values for the non-override settings (gamma,
  // apBehavior). Called from setup() after the boot-time cfg.json
  // deserialise AND from readFromConfig() on every subsequent WebUI
  // Save — so a UI change is live without a reboot. Idempotent. Logs
  // every value it actually had to write.
  void applyRaceLinkDefaults();

  // Direct-effect visualisations (replace the legacy preset-11 pair feedback
  // and provide a boot-time fallback when the operator did not configure a
  // boot preset).
  void showPairConfirmedEffect();
  void showBootRandomColor();

  // Colour-cycle helpers. applyCycleColor() writes colPri + the segment for
  // a given index (0=R, 1=G, 2=B, >=3=random); applyColorCycleStep() handles
  // the click-driven advance with idle-window reset.
  void applyCycleColor(uint8_t idx);
  void applyColorCycleStep(uint32_t now);

  // Master-contact tracking for the button-quiet gate.
  bool masterContactedRecently() const;
  void noteMasterRx();

  // Button (GPIO 0) — custom gesture state machine.
  void pollPhysicalButton(uint32_t now); // own poll, called from loop()
  void handleRaceLinkButton(uint8_t b, bool pressed, uint32_t now);
  void serviceButtonFade(uint32_t now);

  // Radio helpers
  bool radioInit();
  //void radioStartRx();
  void handlePacket(const uint8_t* buf, size_t len);
  //void sendPacket(const uint8_t* data, size_t len);

  static void on_rx_node(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx);

  void sendAckTo(const uint8_t destLast3[3], uint8_t echoOpcode7, RaceLinkProto::AckStatus st);
  bool sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac);
  void sendStatusReplyTo(const uint8_t destLast3[3]);
  // OPC_GET_CONFIG reply (5B body = P_Config). Sent in response to an
  // OPC_GET_CONFIG request; data0..3 carry the live device-side value of
  // ``option`` packed per-option per the matching OPC_CONFIG write
  // command's layout (uint8 / uint16 LE / uint16-pair).
  void sendGetConfigReplyTo(const uint8_t destLast3[3], uint8_t option,
                            uint8_t data0, uint8_t data1,
                            uint8_t data2, uint8_t data3);
  void serviceStartupIdentifyReplies();

  // Build frames

/*   void buildCoreFromCurrent(GateCore& out) {
    out.deviceType = THIS_TYPE;
    out.groupId    = current.groupId;
    out.flags     = current.flags;
    out.presetId  = current.presetId;
    out.brightness = current.brightness;
  } */
  void buildCoreFromCurrent(GateCore& out) { out = current; }  // oder komplett entfernen TODO: noch nötig?

  // current.flags is written ONLY by handleControl (single-writer rule), so
  // this reads back the most recently received OPC_CONTROL flag byte. Plumbed
  // for the upcoming scene editor (staggered group offsets via groupDelay /
  // groupQuotient); currently a no-op in handleSync().
  bool isOffsetMode() const { return (current.flags & RACELINK_FLAG_OFFSET_MODE) != 0; }

  // PRESET handler (OPC_PRESET). Always applies immediately — flags are
  // ignored (single-writer rule, see DeviceState comment).
  void handlePreset(const RaceLinkProto::P_Preset& p);

  // SYNC handler. ``syncFlags`` is the fifth body byte (or 0 when the
  // packet is the legacy 4 B form). Bit 0 = SYNC_FLAG_TRIGGER_ARMED:
  // when clear, only the timebase is adjusted; when set, any pending
  // arm-on-sync state materialises. See the P_Sync comment in
  // racelink_proto.h for the wire-format contract.
  void handleSync(uint32_t ts24, uint8_t briFromPkt, uint8_t syncFlags);

  // OFFSET handler (OPC_OFFSET, variable-length 2..7 B). Parses the body
  // (mode discriminator + mode-specific payload) and writes the result
  // into ``pendingChange``. Filters by groupId: only applies if
  // body.groupId == 255 or matches our own. Returns false on malformed
  // bodies (mode unknown / payload short).
  bool handleOffset(const uint8_t* body, uint8_t bodyLen);

  // Acceptance gate for OPC_CONTROL / OPC_PRESET. Returns true when the
  // packet's OFFSET_MODE flag matches whether the *effective* config has
  // mode != NONE (pendingChange if valid, else active). Caller drops the
  // packet on false.
  bool offsetGateAccepts(uint8_t packetFlags) const;

  // Materialise pendingChange into ``active``. Called immediately on
  // accepted, non-armed OPC_CONTROL/OPC_PRESET; deferred to the SYNC
  // handler when the accepted packet had ARM_ON_SYNC set.
  void materialisePendingChange();

  // Evaluate the supplied OffsetConfig against this device's groupId,
  // clamped to [0, 65535]. Used at arm time to snapshot the per-effect
  // offset so a later OPC_OFFSET cannot shift an already-armed effect.
  uint16_t computeOffsetMs(const OffsetConfig& cfg) const;

  // Per-loop tick that fires a deferred apply once its deadline elapses.
  void serviceDeferredApply();

  // Subtracts activePhaseOffsetMs from strip.timebase. ONLY for use right
  // after a HARD reset of strip.timebase (= desiredTb), where no offset
  // has yet been baked in. Soft-sync converges naturally and must NOT
  // call this (would double-subtract).
  void applyPhaseOffsetAfterHardSync();

  // Switches activePhaseOffsetMs to ``newOffsetMs`` and adjusts
  // strip.timebase by the delta so the device's effective phase offset
  // transitions cleanly without waiting for a SYNC. Used by apply paths
  // (immediate, inline-on-sync, deferred-apply).
  void setActivePhaseOffsetMs(int32_t newOffsetMs);

  // CONTROL handlers (OPC_CONTROL, variable-length direct parameters).
  // Returns false on malformed input (under-/overrun, reserved bits we
  // don't know how to consume, etc.).
  static bool parseControlBody(const uint8_t* body, uint8_t len,
                               uint8_t& groupId, uint8_t& flags,
                               AdvancedFields& out);
  void handleControl(uint8_t flags, const AdvancedFields& fields);
  void applyAdvancedFields(uint8_t flags, const AdvancedFields& fields);

  // Snapshot the WLED main segment into current.fields. Called after a preset
  // load (segment values are now whatever the preset defines, not what was
  // last commanded) and from onStateChange (UI / JSON API mutations).
  void refreshFieldsFromSegment();

  // Merge the present-fields of a parsed CONTROL into current.fields.
  // OR-merges fieldMask/extMask so the snapshot accumulates "everything
  // we've ever been told", instead of dropping previously-known values
  // when a partial CONTROL arrives.
  void mergeFieldsIntoSnapshot(const AdvancedFields& f);

  bool handleStreamPacket(const uint8_t* buf, uint8_t len, const uint8_t senderLast3[3]);
};
