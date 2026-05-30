// racelink_transport_eth.h -- RaceLink transport backend over W5500 / UDP (Ethernet)
// Header-only, Arduino-friendly. No heap allocations.
//
// This is the Ethernet sibling of racelink_transport_core.h (the LoRa/RadioLib
// backend). It exposes the SAME `RaceLinkTransport::` namespace surface that
// racelink_wled.{cpp,h} consumes, so the usermod compiles unchanged against
// either backend, selected at build time by -D RACELINK_ETH. Only the transport
// medium changes: UDP datagrams over a Wiznet W5500 instead of LoRa frames.
//
// Why the standalone Arduino Ethernet lib (not ETH.h): the WLED build is pinned
// to Tasmota Arduino-ESP32 2.0.18, whose ETH.h has no ETH_PHY_W5500 / SPI begin
// overload (only RMII PHYs). So the W5500 is driven by the standalone
// arduino-libraries/Ethernet driver over a dedicated SPIClass. See the project
// memory `ethernet_block_e_stage0_w5500_decision`.
//
// Wire framing (authoritative: RaceLink_Host ethernet_transport.py +
// mock_ethernet_node.py; memory `ethernet_block_e_wire_framing`). The firmware's
// internal frame is `Header7 + body` (Header7 = sender3+receiver3+type, type at
// out[6]). The host UDP datagrams differ, so this backend translates at the seam:
//   * N2M (node->host) scheduleSend(out,n): datagram = [out[6]] ++ out[0..n),
//     sent to the learned host endpoint (else broadcast:HOST_PORT).
//   * M2N (host->node) service(): inbound datagram = [type_full][recv3][body];
//     learn the host endpoint from the source (ip,port) and reconstruct a
//     Header7 frame [synthSender3][recv3][type_full][body] for onRxPacket().
//
// License: MIT
#pragma once

#include <Arduino.h>
#include <SPI.h>
// Self-contained W5500 UDP driver. No external library: on the pinned Tasmota
// 2.0.18 core neither ETH.h, the IDF esp_eth W5500 driver, nor the Arduino W5500
// socket libs are usable (see memory ethernet_block_e_stage0_w5500_decision), so
// we talk to the W5500 directly over SPI.
#include "racelink_w5500_udp.h"

#include "racelink_transport_common.h"  // helpers, StreamStatus, RX stream templates; pulls racelink_proto.h

// -------------------- Build-flag configuration (defaults from the plan) ------
// W5500 SPI pinout (ESP32-S3-ETH / Waveshare). Override via the build profile.
#ifndef RACELINK_ETH_MISO
  #define RACELINK_ETH_MISO 12
#endif
#ifndef RACELINK_ETH_MOSI
  #define RACELINK_ETH_MOSI 11
#endif
#ifndef RACELINK_ETH_SCLK
  #define RACELINK_ETH_SCLK 13
#endif
#ifndef RACELINK_ETH_CS
  #define RACELINK_ETH_CS 14
#endif
#ifndef RACELINK_ETH_RST
  #define RACELINK_ETH_RST 9
#endif
#ifndef RACELINK_ETH_INT
  #define RACELINK_ETH_INT 10
#endif
// UDP ports: node listens on NODE_PORT; host replies arrive on HOST_PORT. The
// node learns the real host endpoint from inbound datagrams, so HOST_PORT is
// only the fallback target for replies sent before any host is known.
#ifndef RACELINK_ETH_NODE_PORT
  #define RACELINK_ETH_NODE_PORT 5078
#endif
#ifndef RACELINK_ETH_HOST_PORT
  #define RACELINK_ETH_HOST_PORT 5079
#endif
// DHCP on by default; set -D RACELINK_ETH_DHCP=0 to use the static IP below.
#ifndef RACELINK_ETH_DHCP
  #define RACELINK_ETH_DHCP 1
#endif
// Static IP config: used when DHCP is disabled, or as a fallback if a lease
// fails. Override per build profile. Each macro expands to a comma-separated
// octet list, e.g. -D RACELINK_ETH_IP=192,168,1,177
#ifndef RACELINK_ETH_IP
  #define RACELINK_ETH_IP 192,168,1,177
#endif
#ifndef RACELINK_ETH_SUBNET
  #define RACELINK_ETH_SUBNET 255,255,255,0
#endif
#ifndef RACELINK_ETH_GATEWAY
  #define RACELINK_ETH_GATEWAY 192,168,1,1
#endif

namespace RaceLinkTransport {

// Synthetic 3-byte "host/master" address stamped as Header7.sender on inbound
// M2N datagrams (the host sends no sender3). Must not be broadcast (FF FF FF)
// nor collide with a node's own last-3 MAC; the firmware only uses it for
// master-tracking bookkeeping and echoes it back as the (host-ignored) N2M
// receiver3. 'E','T','H'.
static constexpr uint8_t ETH_HOST_SENDER3[3] = { 0x45, 0x54, 0x48 };

// Largest datagram we handle: Header7(7) + BODY_MAX(22) + 1 type byte, padded.
static constexpr uint8_t ETH_MAX_DGRAM = 64;

// -------------------- PHY config (stub for source-compat) --------------------
// The LoRa backend's PhyCfg carries radio parameters; Ethernet has none. Kept
// as an empty type so any transport-agnostic reference still compiles. The
// usermod only constructs PhyCfg inside the LoRa branch of transportInit().
struct PhyCfg {};

// -------------------- Callbacks (identical surface to the LoRa backend) ------
struct Core;
struct Callbacks {
  void (*onRxPacket)(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx) = nullptr;
  void (*onTxStart)(void* ctx) = nullptr;
  void (*onTxDone)(void* ctx) = nullptr;
  void (*onRxWindowOpen)(uint16_t ms, void* ctx) = nullptr;
  void (*onRxWindowClosed)(uint16_t rxCountDelta, void* ctx) = nullptr;
  void (*onIdle)(void* ctx) = nullptr;
  void* ctx = nullptr;
};

// -------------------- Core state --------------------
// Mirrors the field names the usermod (racelink_wled.{cpp,h}) reads from the
// LoRa Core, plus the stream-state fields the shared RX reassembly templates in
// racelink_transport_common.h require (handleStreamPacket needs the nested
// StreamMode + streamBuf/streamLen/...). Radio/LBT/RX-window concepts collapse
// to inert constants here.
struct Core {
  // --- identity ---
  uint8_t  myMac6[6]  = {0};
  uint8_t  myLast3[3] = {0};
  bool     macReadOK  = false;

  // --- Ethernet/UDP backend (self-contained W5500 driver) ---
  RaceLinkW5500::Driver w5500;
  uint16_t nodePort   = RACELINK_ETH_NODE_PORT;
  uint8_t  hostIp[4]  = {0};        // learned host endpoint (reply target)
  uint16_t hostPort   = RACELINK_ETH_HOST_PORT;
  bool     hostKnown  = false;
  bool     linkUp     = false;
  bool     netReady   = false;      // app UDP socket bound (DHCP done or static)
  bool     dhcpOk     = false;      // true if a DHCP lease was obtained
  uint8_t  ip[4]      = {0};        // own IP (DHCP lease or static)
  uint8_t  subnet[4]  = {0};
  uint8_t  gateway[4] = {0};

  // --- TX state (fire-and-forget: never actually pending) ---
  bool     txPending  = false;
  uint16_t txCount    = 0;
  uint32_t lastTxAtMs = 0;

  // --- RX telemetry (RF metrics are always 0 on Ethernet) ---
  int16_t  lastRssi      = 0;
  int8_t   lastSnr       = 0;
  uint16_t rxCountTotal  = 0;
  uint16_t rxCountFiltered = 0;
  uint32_t lastRxAtMs    = 0;

  // --- LoRa-only knobs kept as inert fields for source-compat ---
  bool     lbtEnable     = false;   // no LBT on a switched/wired medium
  uint32_t toaUsMaxPkt   = 0;       // no time-on-air concept
  uint16_t debug         = 0;

  // --- Stream state (shared RX reassembly; same layout as the LoRa Core) ---
  enum class StreamMode : uint8_t { None, Rx, Tx };
  StreamMode streamMode           = StreamMode::None;
  bool      streamActive          = false;
  bool      streamReady           = false;
  bool      streamLastScheduled   = false;
  uint8_t   streamBuf[128]        = {0};
  uint8_t   streamLen             = 0;
  uint8_t   streamOffset          = 0;
  uint8_t   streamLastPacketsLeft = 0;
  uint8_t   streamTotalPackets    = 0;
  uint8_t   streamIndex           = 0;
  uint16_t  streamPostRxMs        = 0;
  int8_t    streamPostRxNumWanted = -1;
  uint8_t   streamDst3[3]         = {0};
  uint8_t   streamSrc3[3]         = {0};
  uint8_t   streamType            = 0;
};

// -------------------- Config passed to beginCommon --------------------
struct EthCfg {
  int8_t   miso = RACELINK_ETH_MISO;
  int8_t   mosi = RACELINK_ETH_MOSI;
  int8_t   sclk = RACELINK_ETH_SCLK;
  int8_t   cs   = RACELINK_ETH_CS;
  int8_t   rst  = RACELINK_ETH_RST;
  int8_t   irq  = RACELINK_ETH_INT;   // currently unused (polled RX), reserved
  uint16_t nodePort = RACELINK_ETH_NODE_PORT;
};

// -------------------- Lifecycle --------------------
// Bring up the W5500 over the dedicated SPI bus, apply the node MAC + static
// network config, and open the UDP RX socket on nodePort (broadcast-capable for
// OPC_DEVICES discovery). Returns true once the chip is detected and the socket
// reaches the UDP state. (Static IP is a PoC choice; DHCP is a later add.)
inline bool beginCommon(Core& rl, const EthCfg& cfg = EthCfg{}) {
  rl.nodePort = cfg.nodePort;

  // Identity from EFUSE (same source as the LoRa backend).
  if (readEfuseMac6(rl.myMac6)) {
    rl.macReadOK = true;
    last3FromMac6(rl.myLast3, rl.myMac6);
  }

  // Derive a locally-administered unicast MAC for the NIC from the EFUSE MAC.
  uint8_t mac[6];
  if (rl.macReadOK) {
    for (int i = 0; i < 6; ++i) mac[i] = rl.myMac6[i];
  } else {
    static const uint8_t fallback[6] = {0x02, 0x52, 0x4C, 0x00, 0x00, 0x01};
    for (int i = 0; i < 6; ++i) mac[i] = fallback[i];
  }
  mac[0] = (uint8_t)((mac[0] & 0xFE) | 0x02);  // unicast + locally administered

  const uint8_t staticIp[4]     = { RACELINK_ETH_IP };
  const uint8_t staticSubnet[4] = { RACELINK_ETH_SUBNET };
  const uint8_t staticGw[4]     = { RACELINK_ETH_GATEWAY };

  // Chip bring-up (HW reset, SPI, MAC). No IP / socket yet.
  if (!rl.w5500.begin(SPI, cfg.sclk, cfg.miso, cfg.mosi, cfg.cs, cfg.rst, mac)) {
    rl.linkUp = false;
    return false;
  }
  rl.linkUp = rl.w5500.linkUp();   // may still be negotiating

  // Network config. DHCP runs non-blocking (kicked off here, driven to
  // completion in service()); static comes up immediately. radioReady becomes
  // true regardless -- service() gates real RX/TX on rl.netReady.
#if RACELINK_ETH_DHCP
  rl.w5500.dhcpBegin();              // socket 0 -> port 68; service() finishes it
  rl.netReady = false;
#else
  rl.w5500.setStaticIp(staticIp, staticSubnet, staticGw);
  for (int i = 0; i < 4; ++i) { rl.ip[i] = staticIp[i]; rl.subnet[i] = staticSubnet[i]; rl.gateway[i] = staticGw[i]; }
  if (!rl.w5500.udpOpen(rl.nodePort)) return false;
  rl.netReady = true;
#endif
  return true;
}

// LoRa ISR attach — no DIO1 on Ethernet. No-op for source-compat.
inline bool attachDio1(Core& /*rl*/) { return true; }

// LoRa "enter continuous RX" — Ethernet RX is always-on (polled in service()).
inline void setDefaultRxContinuous(Core& /*rl*/) {}

// -------------------- Send (N2M) --------------------
// Fire-and-forget. `buf` is the firmware-internal frame (Header7 + body) exactly
// as RaceLinkProto::build() produced it; the host expects [type_byte] prepended.
// jitterMaxMs is ignored (no LBT/CAD on a wired medium).
inline bool scheduleSend(Core& rl, const uint8_t* buf, uint8_t len, uint16_t /*jitterMaxMs*/ = 0) {
  if (!rl.netReady) return false;            // network not up yet (DHCP pending)
  if (!buf || len < (uint8_t)sizeof(RaceLinkProto::Header7)) return false;

  // N2M datagram = [type_byte] ++ (Header7 + body), where type_byte = Header7.type.
  uint8_t dg[1 + ETH_MAX_DGRAM];
  if ((uint16_t)len + 1u > sizeof(dg)) return false;
  dg[0] = buf[6];                            // Header7.type (bit7 set for N2M)
  for (uint8_t i = 0; i < len; ++i) dg[1 + i] = buf[i];

  static const uint8_t kBroadcast[4] = { 255, 255, 255, 255 };
  const uint8_t* dst   = rl.hostKnown ? rl.hostIp : kBroadcast;
  const uint16_t dport = rl.hostKnown ? rl.hostPort : (uint16_t)RACELINK_ETH_HOST_PORT;

  const bool ok = rl.w5500.sendTo(dst, dport, dg, (uint16_t)(len + 1));
  if (ok) { ++rl.txCount; rl.lastTxAtMs = millis(); }
  rl.txPending = false;  // never queued
  return ok;
}

// -------------------- Service (RX pump) --------------------
// Drains all pending inbound M2N datagrams, learns the host endpoint, and hands
// each one to cb.onRxPacket() as a reconstructed Header7 frame so the usermod's
// handlePacket()/receiverMatches() path runs unchanged.
inline void service(Core& rl, const Callbacks& cb) {
  // Until the network is up, drive the non-blocking DHCP state machine. Once a
  // lease binds (or DHCP times out -> static fallback), open the app UDP socket.
  if (!rl.netReady) {
    const RaceLinkW5500::Driver::DhcpState st = rl.w5500.dhcpPoll();
    if (st == RaceLinkW5500::Driver::DhcpState::Bound) {
      rl.w5500.dhcpResult(rl.ip, rl.subnet, rl.gateway);
      rl.dhcpOk = true;
    } else if (st == RaceLinkW5500::Driver::DhcpState::Failed) {
      const uint8_t sIp[4] = { RACELINK_ETH_IP };
      const uint8_t sSub[4] = { RACELINK_ETH_SUBNET };
      const uint8_t sGw[4] = { RACELINK_ETH_GATEWAY };
      rl.w5500.setStaticIp(sIp, sSub, sGw);
      for (int i = 0; i < 4; ++i) { rl.ip[i] = sIp[i]; rl.subnet[i] = sSub[i]; rl.gateway[i] = sGw[i]; }
      rl.dhcpOk = false;
    } else {
      return;                                  // still negotiating
    }
    if (!rl.w5500.udpOpen(rl.nodePort)) return; // try again next tick on failure
    rl.netReady = true;
  }

  // parsePacket() reports the next datagram's payload size; read() must follow to
  // advance the W5500 RX pointer, so read() is called once per iteration before
  // any `continue`.
  for (uint16_t sz = rl.w5500.parsePacket(); sz > 0; sz = rl.w5500.parsePacket()) {
    uint8_t dg[ETH_MAX_DGRAM];
    uint16_t n = rl.w5500.read(dg, sizeof(dg));
    if (n < 4) continue;                       // need at least type_full + recv3

    const uint8_t typeFull = dg[0];
    if ((typeFull & 0x80) != 0x00) continue;   // only M2N (DIR_M2N bit7 clear)

    // Learn the host endpoint for replies.
    rl.w5500.remoteIP(rl.hostIp);
    rl.hostPort = rl.w5500.remotePort();
    rl.hostKnown = true;

    const uint8_t bodyLen = (uint8_t)(n - 4);
    if ((uint8_t)(sizeof(RaceLinkProto::Header7) + bodyLen) > ETH_MAX_DGRAM) continue;

    // Reconstruct the firmware-internal Header7 frame.
    uint8_t frame[ETH_MAX_DGRAM];
    frame[0] = ETH_HOST_SENDER3[0];
    frame[1] = ETH_HOST_SENDER3[1];
    frame[2] = ETH_HOST_SENDER3[2];
    frame[3] = dg[1];                          // recv3
    frame[4] = dg[2];
    frame[5] = dg[3];
    frame[6] = typeFull;                       // Header7.type
    for (uint8_t i = 0; i < bodyLen; ++i) frame[7 + i] = dg[4 + i];
    const uint8_t flen = (uint8_t)(sizeof(RaceLinkProto::Header7) + bodyLen);

    ++rl.rxCountTotal;

    // Parity with the LoRa backend: drop datagrams not addressed to us before
    // the callback (handlePacket re-checks, but filtering here matches LoRa).
    RaceLinkProto::Header7 h{};
    if (!RaceLinkProto::parseHeader(frame, flen, h)) continue;
    if (!receiverMatches(h.receiver, rl.myLast3)) continue;

    rl.lastRssi = 0;
    rl.lastSnr  = 0;
    rl.lastRxAtMs = millis();
    ++rl.rxCountFiltered;

    if (cb.onRxPacket) cb.onRxPacket(frame, flen, 0, 0, cb.ctx);
  }
}

} // namespace RaceLinkTransport
