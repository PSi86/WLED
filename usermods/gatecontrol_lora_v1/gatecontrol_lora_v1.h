#pragma once
#include "wled.h"

#include "lora_link_core.h"
//#include "lora_proto.h"

// ===================== LoRa / RadioLib =====================
//#include <RadioLib.h>
extern "C" {
  #include "esp_system.h"
  //#include "esp_wifi.h"
  #include "esp_mac.h"   // for esp_read_mac / ESP_MAC_WIFI_STA
}

#define FW_VERSION 2 // Version of the GateControlLoRa firmware
using GateCore = LoraProto::P_WledControl;  // 4B: groupId, state, effect, brightness // Define control packet type for this device

// README WLAN Behaviour in WLED
// wled.h (~Line 380):  WLED_GLOBAL byte apBehavior      _INIT(AP_BEHAVIOR_BUTTON_ONLY); // modified for WLED LoRa Gates
// wled.h (~Line 380):  WLED_GLOBAL byte apBehavior      _INIT(AP_BEHAVIOR_BOOT_NO_CONN); // Original for ESP32
// Button Only: Start AP only on button press (5s)
// Short Button press: toogle on/off
// Long Button press (600ms): Toggle Preset


// ------------ HT-CT62 (ESP32-C3 + SX1262) fixed pins ------------
#ifndef LORA_PIN_MOSI
  #define LORA_PIN_MOSI   7
#endif
#ifndef LORA_PIN_MISO
  #define LORA_PIN_MISO   6
#endif
#ifndef LORA_PIN_SCK
  #define LORA_PIN_SCK    10
#endif
#ifndef LORA_PIN_NSS
  #define LORA_PIN_NSS    8   // strapping pin on HT-CT62 (already wired to CS)
#endif
#ifndef LORA_PIN_DIO1
  #define LORA_PIN_DIO1   3
#endif
#ifndef LORA_PIN_BUSY
  #define LORA_PIN_BUSY   4
#endif
#ifndef LORA_PIN_RST
  #define LORA_PIN_RST    5
#endif

// ------------ LoRa defaults (EU868) ------------
#ifndef LORA_FREQ_HZ
  #define LORA_FREQ_HZ    868000000UL
#endif
#ifndef LORA_BW_KHZ
  #define LORA_BW_KHZ     125.0
#endif
#ifndef LORA_SF
  #define LORA_SF         7
#endif
#ifndef LORA_CR
  #define LORA_CR         5      // 4/5
#endif
#ifndef LORA_SYNC_WORD
  #define LORA_SYNC_WORD  0x12
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER   14     // dBm
#endif
#ifndef LORA_PREAMBLE
  #define LORA_PREAMBLE   8
#endif


class UsermodGateControlLoRa : public Usermod {
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
  // Radio / SPI
  SPIClass* spi = &SPI;
  SX1262* radio = nullptr;

  //UsermodBattery* bat = nullptr;
  um_data_t* batteryUM = nullptr;

  // Node identity
  // now in LoraLink::ll struct
/*   uint8_t myMac6[6] = {0};
  uint8_t myLast3[3] = {0};
  bool macReadOK = false; */

  // Config / State
  GateCore current{};  // init in setup()

  // Master filter options
  bool macFilterEnabled = true;   // default: ON
  bool macFilterPersist = false;  // default: OFF
  bool masterKnown = false;
  uint8_t masterLast3[3] = {0};
  uint8_t masterFull6[6] = {0};
  bool   masterFull6Known = false;

  // Discovery reply target
  uint8_t targetForReplyLast3[3] = {0};

  // ===== Debug/Diag =====
  bool    radioReady = false;
  int16_t radioInitCode = 0;
  //uint32_t rxTotal = 0;      // all packets successfully read
  uint32_t rxAccepted = 0;   // packets that passed filters AND triggered actions (difers from rxTotal and rxCountFiltered in lora_link_core)
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

  void sendAckTo(const uint8_t destLast3[3], uint8_t echoOpcode7, LoraProto::AckStatus st);
  void sendIdentifyReplyTo(const uint8_t destLast3[3], bool includeFullMac);
  void sendStatusReplyTo(const uint8_t destLast3[3]);

  // Build frames

/*   void buildCoreFromCurrent(GateCore& out) {
    out.deviceType = THIS_TYPE;
    out.groupId    = current.groupId;
    out.state      = current.state;
    out.effect     = current.effect;
    out.brightness = current.brightness;
  } */
  void buildCoreFromCurrent(GateCore& out) { out = current; }  // oder komplett entfernen TODO: noch nötig?

  // Apply CONTROL like original ESPNOW logic
  void applyControl(const GateCore& in);
};
