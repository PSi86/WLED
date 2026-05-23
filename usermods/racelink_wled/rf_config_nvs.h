// rf_config_nvs.h -- WLED-Node-side persistent storage for the LoRa
// PHY config (P_RfConfig wire struct). Backs the LoRa-level
// OPC_RF_CONFIG / OPC_GET_RF_CONFIG opcodes and the compile-default
// fallback path used by radioInit() before radio.begin().
//
// Mirrors the gateway-side `rf_config_nvs.h` (same NVS namespace +
// schema, same validator, same boot-loop recovery) so a cross-flashed
// image keeps the NVS slot interpretable. The only differences from
// the gateway copy are the compile-default define names (RACELINK_CR
// vs RACELINK_CR_DEN, RACELINK_SYNC_WORD vs RACELINK_SYNCWORD) -- the
// compile-default helper lives at the call site (racelink_wled.cpp)
// to keep this header WLED-/Gateway-agnostic.
//
// Header-only / inline. NVS access goes through ESP32 Preferences. A
// schema CRC16 over the P_RfConfig bytes guards against a slot that
// survives Preferences' own CRC but no longer matches the struct
// layout (sizeof-equal field shuffle on a P_RfConfig revision). A
// boot-loop counter wipes the slot after 3 consecutive failed boots
// so an unflashable persisted config can be recovered by power-
// cycling.
//
// Lives in usermods/racelink_wled/ -- WLED-fork-only, NOT one of the
// four shared headers. No drift-test coverage required.
//
// Added 2026-05-20 for Stage 1 PR-3 of the multi-gateway plan.

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "racelink_proto.h"

namespace RfConfigNvs {

static const uint16_t NVS_MAGIC = 0xA5C3;
static const uint8_t BOOT_COUNTER_MAX = 3;

static const char* NVS_NAMESPACE = "rl_rf";
static const char* KEY_MAGIC     = "magic";
static const char* KEY_CONFIG    = "cfg";
static const char* KEY_CRC       = "crc";
static const char* KEY_BOOTCNT   = "bootcnt";

inline uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline RaceLinkProto::RfChangeReason validate(const RaceLinkProto::P_RfConfig& c) {
  if (c.freq_hz < 863000000UL || c.freq_hz > 928000000UL) {
    return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  }
  switch (c.bw_khz_x10) {
    case 78: case 104: case 156: case 208: case 313:
    case 417: case 625: case 1250: case 2500: case 5000:
      break;
    default:
      return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  }
  if (c.sf < 5 || c.sf > 12) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.cr_den < 5 || c.cr_den > 8) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.tx_power_dbm < -9 || c.tx_power_dbm > 22) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.preamble < 6) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  return RaceLinkProto::RF_CHANGE_OK;
}

inline RaceLinkProto::RfChangeReason store(const RaceLinkProto::P_RfConfig& c) {
  auto r = validate(c);
  if (r != RaceLinkProto::RF_CHANGE_OK) return r;
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    return RaceLinkProto::RF_CHANGE_NVS_FAIL;
  }
  bool ok = true;
  ok &= (prefs.putUShort(KEY_MAGIC, NVS_MAGIC) == sizeof(uint16_t));
  ok &= (prefs.putBytes(KEY_CONFIG, &c, sizeof(c)) == sizeof(c));
  const uint16_t crc = crc16(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
  ok &= (prefs.putUShort(KEY_CRC, crc) == sizeof(uint16_t));
  prefs.end();
  return ok ? RaceLinkProto::RF_CHANGE_OK : RaceLinkProto::RF_CHANGE_NVS_FAIL;
}

inline bool load(RaceLinkProto::P_RfConfig& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/true)) return false;
  bool good = false;
  do {
    if (!prefs.isKey(KEY_MAGIC) || !prefs.isKey(KEY_CONFIG) || !prefs.isKey(KEY_CRC)) break;
    if (prefs.getUShort(KEY_MAGIC, 0) != NVS_MAGIC) break;
    if (prefs.getBytesLength(KEY_CONFIG) != sizeof(RaceLinkProto::P_RfConfig)) break;
    RaceLinkProto::P_RfConfig tmp{};
    if (prefs.getBytes(KEY_CONFIG, &tmp, sizeof(tmp)) != sizeof(tmp)) break;
    const uint16_t expected = prefs.getUShort(KEY_CRC, 0);
    const uint16_t actual   = crc16(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
    if (expected != actual) break;
    if (validate(tmp) != RaceLinkProto::RF_CHANGE_OK) break;
    out = tmp;
    good = true;
  } while (false);
  prefs.end();
  return good;
}

inline uint8_t bootCounterIncrement() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return 0;
  uint8_t c = prefs.getUChar(KEY_BOOTCNT, 0);
  if (c < 255) ++c;  // saturate
  prefs.putUChar(KEY_BOOTCNT, c);
  prefs.end();
  return c;
}

inline void bootCounterClear() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return;
  prefs.putUChar(KEY_BOOTCNT, 0);
  prefs.end();
}

inline void wipe() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return;
  prefs.remove(KEY_MAGIC);
  prefs.remove(KEY_CONFIG);
  prefs.remove(KEY_CRC);
  prefs.putUChar(KEY_BOOTCNT, 0);
  prefs.end();
}

} // namespace RfConfigNvs
