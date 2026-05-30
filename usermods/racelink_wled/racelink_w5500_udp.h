// racelink_w5500_udp.h -- minimal, self-contained Wiznet W5500 UDP driver (SPI)
// Header-only, Arduino-friendly. No heap allocations, no external library.
//
// Why this exists: on WLED's pinned Tasmota Arduino-ESP32 2.0.18 core there is no
// usable W5500 path -- ETH.h has no W5500, the IDF esp_eth W5500 driver is not in
// the precompiled SDK, and the Arduino W5500 socket libs (arduino-libraries/
// Ethernet, Ethernet_Generic) don't compile (ambiguous IPAddress ctor). So this
// file talks to the W5500 directly over SPI using the chip's documented register
// map and one UDP socket -- enough for the RaceLink node (discovery / status /
// preset over UDP). See project memory `ethernet_block_e_stage0_w5500_decision`.
//
// Scope: socket 0 in UDP mode, static IP (PoC; DHCP is a later add), unicast +
// broadcast RX, unicast/broadcast TX. The API mirrors the small EthernetUDP/
// WiFiUDP subset that racelink_transport_eth.h consumes.
//
// License: MIT
#pragma once

#include <Arduino.h>
#include <SPI.h>

namespace RaceLinkW5500 {

// ---- W5500 SPI framing: [addr_hi][addr_lo][control][data...] ----------------
// control = (BSB<<3) | (RW<<2) | OM ; OM=00 => variable-length data mode.
static constexpr uint8_t BSB_COMMON = 0x00;          // common register block
static constexpr uint8_t BSB_S0_REG = 0x01;          // socket 0 register block
static constexpr uint8_t BSB_S0_TX  = 0x02;          // socket 0 TX buffer
static constexpr uint8_t BSB_S0_RX  = 0x03;          // socket 0 RX buffer
static constexpr uint8_t RWB_WRITE  = 0x04;
static constexpr uint8_t RWB_READ   = 0x00;

// ---- Common registers -------------------------------------------------------
static constexpr uint16_t REG_MR       = 0x0000;
static constexpr uint16_t REG_GAR      = 0x0001;     // gateway IP (4)
static constexpr uint16_t REG_SUBR     = 0x0005;     // subnet mask (4)
static constexpr uint16_t REG_SHAR     = 0x0009;     // source MAC (6)
static constexpr uint16_t REG_SIPR     = 0x000F;     // source IP (4)
static constexpr uint16_t REG_PHYCFGR  = 0x002E;
static constexpr uint16_t REG_VERSIONR = 0x0039;     // reads 0x04 on W5500

// ---- Socket 0 registers -----------------------------------------------------
static constexpr uint16_t Sn_MR      = 0x0000;
static constexpr uint16_t Sn_CR      = 0x0001;
static constexpr uint16_t Sn_IR      = 0x0002;
static constexpr uint16_t Sn_SR      = 0x0003;
static constexpr uint16_t Sn_PORT    = 0x0004;       // (2)
static constexpr uint16_t Sn_DIPR    = 0x000C;       // dest IP (4)
static constexpr uint16_t Sn_DPORT   = 0x0010;       // dest port (2)
static constexpr uint16_t Sn_RXBUF_SIZE = 0x001E;
static constexpr uint16_t Sn_TXBUF_SIZE = 0x001F;
static constexpr uint16_t Sn_TX_FSR  = 0x0020;       // free size (2)
static constexpr uint16_t Sn_TX_WR   = 0x0024;       // write pointer (2)
static constexpr uint16_t Sn_RX_RSR  = 0x0026;       // received size (2)
static constexpr uint16_t Sn_RX_RD   = 0x0028;       // read pointer (2)

// ---- Values -----------------------------------------------------------------
static constexpr uint8_t MR_RST       = 0x80;        // software reset (MR)
static constexpr uint8_t Sn_MR_UDP    = 0x02;
static constexpr uint8_t Sn_CR_OPEN   = 0x01;
static constexpr uint8_t Sn_CR_CLOSE  = 0x10;
static constexpr uint8_t Sn_CR_SEND   = 0x20;
static constexpr uint8_t Sn_CR_RECV   = 0x40;
static constexpr uint8_t Sn_SR_UDP    = 0x22;
static constexpr uint8_t PHYCFGR_LINK = 0x01;        // 1 = link up

class Driver {
public:
  // Bring up the chip: HW reset, SPI bus, MAC + static network config, and open
  // socket 0 as a UDP listener on `port`. Returns true if the chip is detected
  // (VERSIONR==0x04) and the socket reaches the UDP state.
  bool begin(SPIClass& spi, int8_t sck, int8_t miso, int8_t mosi, int8_t cs,
             int8_t rst, const uint8_t mac6[6],
             const uint8_t ip[4], const uint8_t subnet[4], const uint8_t gw[4],
             uint16_t port) {
    _spi = &spi;
    _cs  = cs;

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _spi->begin(sck, miso, mosi, cs);

    if (rst >= 0) {
      pinMode(rst, OUTPUT);
      digitalWrite(rst, LOW);
      delayMicroseconds(500);          // >= 500us reset low (datasheet)
      digitalWrite(rst, HIGH);
      delay(2);                         // PLL lock
    }

    // Software reset, then wait for it to clear.
    writeReg8(BSB_COMMON, REG_MR, MR_RST);
    for (uint8_t i = 0; i < 20 && (readReg8(BSB_COMMON, REG_MR) & MR_RST); ++i) delay(1);

    if (readReg8(BSB_COMMON, REG_VERSIONR) != 0x04) return false;  // not a W5500

    // Static network identity.
    writeBuf(BSB_COMMON, REG_SHAR, mac6, 6);
    writeBuf(BSB_COMMON, REG_SIPR, ip, 4);
    writeBuf(BSB_COMMON, REG_SUBR, subnet, 4);
    writeBuf(BSB_COMMON, REG_GAR, gw, 4);

    // 2KB TX/RX buffers for socket 0 (default; other sockets unused).
    writeReg8(BSB_S0_REG, Sn_RXBUF_SIZE, 2);
    writeReg8(BSB_S0_REG, Sn_TXBUF_SIZE, 2);

    return udpOpen(port);
  }

  // (Re)open socket 0 in UDP mode on `port`.
  bool udpOpen(uint16_t port) {
    _port = port;
    writeReg8(BSB_S0_REG, Sn_MR, Sn_MR_UDP);           // UDP, broadcast RX enabled
    writeReg16(BSB_S0_REG, Sn_PORT, port);
    sockCmd(Sn_CR_OPEN);
    for (uint8_t i = 0; i < 20; ++i) {
      if (readReg8(BSB_S0_REG, Sn_SR) == Sn_SR_UDP) return true;
      delay(1);
    }
    return false;
  }

  bool linkUp() { return (readReg8(BSB_COMMON, REG_PHYCFGR) & PHYCFGR_LINK) != 0; }

  // If a datagram is waiting, consume its 8-byte W5500 UDP header (srcIP, srcPort,
  // payload length) and return the payload length; the payload is then read with
  // read(). Returns 0 when nothing is pending. Caller must read() the reported
  // length before the next parsePacket().
  uint16_t parsePacket() {
    if (_pendingLen) return _pendingLen;               // not yet read out
    uint16_t rsr = readReg16(BSB_S0_REG, Sn_RX_RSR);
    if (rsr < 8) return 0;                              // need at least the header

    uint16_t rd = readReg16(BSB_S0_REG, Sn_RX_RD);
    uint8_t hdr[8];
    readBuf(BSB_S0_RX, rd, hdr, 8);
    _srcIp[0]=hdr[0]; _srcIp[1]=hdr[1]; _srcIp[2]=hdr[2]; _srcIp[3]=hdr[3];
    _srcPort = (uint16_t)(hdr[4] << 8) | hdr[5];
    uint16_t dlen = (uint16_t)(hdr[6] << 8) | hdr[7];

    _rxRd = (uint16_t)(rd + 8);                         // payload starts after header
    _pendingLen = dlen;
    return dlen;
  }

  // Copy up to maxLen payload bytes of the datagram parsed by parsePacket() and
  // advance the RX read pointer past the whole datagram (header + payload).
  uint16_t read(uint8_t* buf, uint16_t maxLen) {
    if (!_pendingLen) return 0;
    uint16_t n = _pendingLen < maxLen ? _pendingLen : maxLen;
    if (n) readBuf(BSB_S0_RX, _rxRd, buf, n);
    // Advance past the whole payload regardless of n (_rxRd already points past
    // the 8-byte header consumed in parsePacket()).
    writeReg16(BSB_S0_REG, Sn_RX_RD, (uint16_t)(_rxRd + _pendingLen));
    sockCmd(Sn_CR_RECV);
    _pendingLen = 0;
    return n;
  }

  void remoteIP(uint8_t out[4]) const { out[0]=_srcIp[0]; out[1]=_srcIp[1]; out[2]=_srcIp[2]; out[3]=_srcIp[3]; }
  uint16_t remotePort() const { return _srcPort; }

  // Fire-and-forget UDP send to (ip,port). Returns true once the SEND command is
  // accepted (does not wait for SEND_OK).
  bool sendTo(const uint8_t ip[4], uint16_t port, const uint8_t* data, uint16_t len) {
    if (!data || len == 0) return false;
    // Wait briefly for enough TX free space.
    for (uint8_t i = 0; i < 50; ++i) {
      if (readReg16(BSB_S0_REG, Sn_TX_FSR) >= len) break;
      delayMicroseconds(100);
    }
    if (readReg16(BSB_S0_REG, Sn_TX_FSR) < len) return false;

    writeBuf(BSB_S0_REG, Sn_DIPR, ip, 4);
    writeReg16(BSB_S0_REG, Sn_DPORT, port);

    uint16_t wr = readReg16(BSB_S0_REG, Sn_TX_WR);
    writeBuf(BSB_S0_TX, wr, data, len);
    writeReg16(BSB_S0_REG, Sn_TX_WR, (uint16_t)(wr + len));
    sockCmd(Sn_CR_SEND);
    return true;
  }

private:
  SPIClass* _spi = nullptr;
  int8_t    _cs  = -1;
  uint16_t  _port = 0;
  uint8_t   _srcIp[4] = {0};
  uint16_t  _srcPort = 0;
  uint16_t  _pendingLen = 0;   // payload bytes of the parsed-but-unread datagram
  uint16_t  _rxRd = 0;         // RX read pointer at the payload start

  // SPI clock for the W5500 (datasheet allows up to ~80MHz; 16MHz is safe).
  static constexpr uint32_t kSpiHz = 16000000UL;

  void beginTxn() {
    _spi->beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
  }
  void endTxn() {
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
  }
  void sendFrameHeader(uint8_t bsb, uint16_t addr, uint8_t rwb) {
    _spi->transfer((uint8_t)(addr >> 8));
    _spi->transfer((uint8_t)(addr & 0xFF));
    _spi->transfer((uint8_t)((bsb << 3) | rwb));        // OM = 00 (variable length)
  }

  void writeReg8(uint8_t bsb, uint16_t addr, uint8_t val) {
    beginTxn(); sendFrameHeader(bsb, addr, RWB_WRITE); _spi->transfer(val); endTxn();
  }
  uint8_t readReg8(uint8_t bsb, uint16_t addr) {
    beginTxn(); sendFrameHeader(bsb, addr, RWB_READ); uint8_t v = _spi->transfer(0x00); endTxn();
    return v;
  }
  void writeReg16(uint8_t bsb, uint16_t addr, uint16_t val) {
    beginTxn(); sendFrameHeader(bsb, addr, RWB_WRITE);
    _spi->transfer((uint8_t)(val >> 8)); _spi->transfer((uint8_t)(val & 0xFF)); endTxn();
  }
  // 16-bit pointer registers can change between reads; read twice until stable.
  uint16_t readReg16(uint8_t bsb, uint16_t addr) {
    uint16_t a, b = readReg16Once(bsb, addr);
    for (uint8_t i = 0; i < 4; ++i) { a = b; b = readReg16Once(bsb, addr); if (a == b) break; }
    return b;
  }
  uint16_t readReg16Once(uint8_t bsb, uint16_t addr) {
    beginTxn(); sendFrameHeader(bsb, addr, RWB_READ);
    uint8_t hi = _spi->transfer(0x00); uint8_t lo = _spi->transfer(0x00); endTxn();
    return (uint16_t)(hi << 8) | lo;
  }
  void writeBuf(uint8_t bsb, uint16_t addr, const uint8_t* data, uint16_t len) {
    if (len == 0) return;
    beginTxn(); sendFrameHeader(bsb, addr, RWB_WRITE);
    for (uint16_t i = 0; i < len; ++i) _spi->transfer(data[i]);
    endTxn();
  }
  void readBuf(uint8_t bsb, uint16_t addr, uint8_t* data, uint16_t len) {
    if (len == 0) return;
    beginTxn(); sendFrameHeader(bsb, addr, RWB_READ);
    for (uint16_t i = 0; i < len; ++i) data[i] = _spi->transfer(0x00);
    endTxn();
  }
  void sockCmd(uint8_t cmd) {
    writeReg8(BSB_S0_REG, Sn_CR, cmd);
    while (readReg8(BSB_S0_REG, Sn_CR) != 0) { /* W5500 clears CR when accepted */ }
  }
};

} // namespace RaceLinkW5500
