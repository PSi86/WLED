// normal Wizmote (remote.cpp) implementation needs to be disabled in my_config:
// #define WLED_DISABLE_ESPNOW
// #define RH_GateControl_V2

// if this is not done the receive callback will be overwritten by this plugin but the logic to initialize 
// and deinitialize espnow on every iteration will continue.

// ideas
// control wifi behaviour from here: set to station but do not connect to any wifi.
// have special command to activate wifi ap (to be able to do ota-update or web configuration)

#pragma once
#include "wled.h"

#ifdef ESP8266
    #include <espnow.h>
#else // ESP32
    #include <esp_now.h>
#endif

#define ESP_OK 0
#define ESP_NOW_STATE_UNINIT       0
#define ESP_NOW_STATE_ON           1
#define ESP_NOW_STATE_ERROR        2

String gc_remote_str = "94b97e8252e8"; // used for setting storage
char gc_remote[13]; // gets updated when loading config from json storage 
char gc_last_src[13]; // updates when a message is received - used for comparison with gc_remote

#define THIS_TYPE 24 // Must match one of the device types below

// Defines for Device Types and commands
#define ESPNOW_GATE 20 // unified message structure - groups only work with this type
#define BASIC_IR_GATE 21 // IR Area Controller will identify with this code
#define CUSTOM_IR_GATE 22 // not used currently
#define WIZMOTE_GATE 23 // standard WLED type(does not support self identification)
#define WLED_CUSTOM 24 // once custom WLED fw is built this will be the identifier
#define GET_DEVICES 30 // only devices with groupId != 0 should respond here 
#define SET_GROUP 31 // send this command to make a device store the received groupId

byte gc_rcvAddress[6];
typedef struct struct_unified_gate_message {
  uint8_t deviceType=THIS_TYPE;
  uint8_t groupId=0;
  uint8_t state;
  uint8_t effect;
  uint8_t brightness;
} struct_unified_gate_message;

// create struct instance for incoming data
struct_unified_gate_message gc_newData;
const uint8_t numStructBytes = sizeof(gc_newData);

// create struct instance for current State of this gate controller
struct_unified_gate_message gc_currentData;

// Command Enum
enum gc_command {NONE, CONTROL, IDENTIFY, SETGROUP};
gc_command gc_activeCmd;

static int esp_now_state = ESP_NOW_STATE_UNINIT;
static uint32_t last_seq = UINT32_MAX;
uint32_t cur_seq = 0;

unsigned long gc_now=0; // store current millis value once per main loop iteration
unsigned long gc_target_time=0; // delayed execution timer

    // Callback function that will be executed when data is received
    #ifdef ESP8266
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
    #else
    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    #endif

        sprintf (gc_last_src, "%02x%02x%02x%02x%02x%02x",
            mac [0], mac [1], mac [2], mac [3], mac [4], mac [5]);

        if (strcmp(gc_last_src, gc_remote) != 0) {
            DEBUG_PRINT(F("ESP Now Message Received from Unlinked Sender: "));
            DEBUG_PRINTLN(gc_last_src);
            return;
        }

        if (len != numStructBytes) {
            DEBUG_PRINT(F("Unknown incoming ESP Now message received of length "));
            DEBUG_PRINTLN(len);
            return;
        }

        if(gc_activeCmd==NONE) {
            // insert received data into gc_newData struct
            memcpy(&gc_newData, incomingData, numStructBytes);
            cur_seq++;

            if (cur_seq == last_seq) {
                return;
            }
            if((gc_newData.deviceType==THIS_TYPE || gc_newData.deviceType==ESPNOW_GATE) && (gc_newData.groupId==gc_currentData.groupId || gc_newData.groupId==255)) {
                gc_activeCmd=CONTROL;
                DEBUG_PRINT(F("Control Command accepted"));
            }
            else if(gc_newData.deviceType==GET_DEVICES && (gc_currentData.groupId==gc_newData.groupId || gc_newData.groupId==255)) {
                gc_activeCmd=IDENTIFY;
                bri=128;
                applyPreset(11, CALL_MODE_DIRECT_CHANGE); // applies preset but not brightness
                //stateUpdated(CALL_MODE_DIRECT_CHANGE); // apply brightness - could also use: CALL_MODE_NO_NOTIFY
                memcpy(&gc_rcvAddress, mac, 6); // store senders MAC for sending answer
                DEBUG_PRINT(F("Identify Command accepted"));
            }
            // Set groupId works only once while groupId has not been changed during runtime or if the command is received with a state of 255
            else if(gc_newData.deviceType==SET_GROUP && (gc_currentData.groupId==0 || gc_newData.state==255)) { 
                gc_activeCmd=SETGROUP;
                bri=128;
                applyPreset(2, CALL_MODE_DIRECT_CHANGE); // applies preset but not brightness
                DEBUG_PRINT(F("SetGroup Command accepted"));
            }
        }

        DEBUG_PRINT(F("Incoming ESP Now Packet["));
        DEBUG_PRINT(cur_seq);
        DEBUG_PRINT(F("] from sender:"));
        DEBUG_PRINT(gc_last_src);

        last_seq = cur_seq;

    }


class rh_gatecontrol_v2 : public Usermod {
  private:

      

  public:

    void espnow_control(bool espnow_activate) {
        if (espnow_activate==true) {
            //if ((esp_now_state == ESP_NOW_STATE_UNINIT) && (interfacesInited || apActive)) { // ESPNOW requires Wifi to be initialized (either STA, or AP Mode) 
            if ((esp_now_state == ESP_NOW_STATE_UNINIT)) { // ESPNOW requires Wifi to be initialized (either STA, or AP Mode) 
                DEBUG_PRINTLN(F("Initializing ESP_NOW listener"));
                // Init ESP-NOW
                if (esp_now_init() != 0) {
                    DEBUG_PRINTLN(F("Error initializing ESP-NOW"));
                    esp_now_state = ESP_NOW_STATE_ERROR;
                    // Error while init - try to set everything for better chance next iteration
                    if(!apActive) {
                        //dnsServer.stop(); // TESTING
                        //WiFi.softAPdisconnect(true); // TESTING Try this to force espnow working
                        //WiFi.mode(WIFI_STA); // TESTING Try this to force espnow working
                    }
                    return;
                }

                #ifdef ESP8266
                esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
                #endif
                
                esp_now_register_recv_cb(OnDataRecv);
                esp_now_state = ESP_NOW_STATE_ON;
            }
        } 
        else {
            if (esp_now_state == ESP_NOW_STATE_ON) {
                DEBUG_PRINTLN(F("Disabling ESP-NOW Remote Listener"));
                if (esp_now_deinit() != 0) {
                    DEBUG_PRINTLN(F("Error de-initializing ESP-NOW"));
                }
                esp_now_state = ESP_NOW_STATE_UNINIT;
            } 
            else if (esp_now_state == ESP_NOW_STATE_ERROR) {
                //Clear any error states (allows retrying by cycling)
                esp_now_state = ESP_NOW_STATE_UNINIT;
            }
        }
    }    

    void setup() {
    }

    void loop() {
        gc_now=millis();
        
        if(esp_now_state != ESP_NOW_STATE_ON) {
            espnow_control(true); // If esp_now is not successfully initialized we keep trying
        }
        
        if(gc_activeCmd==NONE) {
            return;
        }

        if(gc_activeCmd==CONTROL) {
            if(gc_newData.state>0) {
                bri=gc_newData.brightness;
            }
            else {
                bri=0;
            }
            //applyBri(); // not needed? works without
            applyPreset(gc_newData.effect); // applies preset but not brightness
            stateUpdated(CALL_MODE_DIRECT_CHANGE); // apply brightness - could also use: CALL_MODE_NO_NOTIFY

            //memcpy(&gc_currentData, &gc_newData, numStructBytes); // as groupID and deviceType should not be overwritten we better do individual updates
            gc_currentData.state=gc_newData.state;
            gc_currentData.effect=gc_newData.effect;
            gc_currentData.brightness=gc_newData.brightness;
            
            gc_activeCmd=NONE; // if new data packet is received while we are still busy with the last packet the new one will be ignored.
        }

        if(gc_activeCmd==IDENTIFY) {
            if(gc_target_time>0 && gc_now>gc_target_time) { //time to send identification back to sender
                DEBUG_PRINT(F("Target Time for identification reply reached."));
                // Set peer
                //memcpy(peerInfo.peer_addr, gc_rcvAddress, 6);
                if (esp_now_add_peer(gc_rcvAddress,ESP_NOW_ROLE_SLAVE,1,NULL,0) != ESP_OK) {
                    DEBUG_PRINT(F("Failed to add peer"));
                    return;
                }
                // Execute Identification response
                int result = esp_now_send(gc_rcvAddress, (uint8_t *) &gc_currentData, numStructBytes);

                if (result == ESP_OK) {
                    DEBUG_PRINT(F("Sent identification with success"));
                } 
                else {
                    DEBUG_PRINT(F("Error sending the identification"));
                }

                gc_target_time=0; // delayed execution triggered, disable timer
                gc_activeCmd=NONE; // IDENTIFY command will be active until here (after sending state back)
            }
            else if (gc_target_time == 0) { // Command is fresh, no target time was set previously, so set it
                gc_target_time=gc_now+random(100, 2000); // calculate a time between 0.1 and 2 seconds in the future
                DEBUG_PRINT(F("Target Time set:"));
                DEBUG_PRINT(gc_target_time);
            }
        }
        if(gc_activeCmd==SETGROUP) {
            gc_currentData.groupId=gc_newData.groupId;
            DEBUG_PRINT(F("GroupId has been set: "));
            DEBUG_PRINT(gc_currentData.groupId);
            gc_activeCmd=NONE; // if new data packet is received while we are still busy with the last packet the new one will be ignored.
        }

    }

    void addToConfig(JsonObject& root) {
        JsonObject top = root.createNestedObject("RH_GateControl");
        gc_remote_str.toLowerCase();
        top["GC_MAC"] = gc_remote_str;
    }

    bool readFromConfig(JsonObject& root) {
        // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
        // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)

        JsonObject top = root["RH_GateControl"];

        bool configComplete = !top.isNull();

        //configComplete &= getJsonValue(top["GC_MAC"], gc_remote_str);

        // A 3-argument getJsonValue() assigns the 3rd argument as a default value if the Json value is missing
        configComplete &= getJsonValue(top["GC_MAC"], gc_remote_str, "94b97e8252e8");
        gc_remote_str.toLowerCase();
        strcpy(gc_remote, gc_remote_str.c_str());
        return configComplete;
    }

    void appendConfigData() {
        //oappend(SET_F("addInfo('")); oappend(String("RotorHazard GateControl Remote").c_str()); oappend(SET_F(":gc_remote_str")); oappend(SET_F("',1,'<i>(MAC of RotorHazard GateControl USB Communicator)</i>');"));
    }

};
