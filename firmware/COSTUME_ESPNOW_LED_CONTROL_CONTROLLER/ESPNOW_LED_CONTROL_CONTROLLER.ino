/**
   ESPNOW - CONNECTED TO THE PC
   SENDS DATA TO A REMOTE ACTUATOR NODE

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/

#define COORDINATOR_AP_MODE // uncomment to run coordinator in AP-mode / broadcasting a network ssid
#define ESPNOW_CHANNEL 1 // ESPNow channel, must match nodes firmware
#define ESPNOW_ENCRYPTION 0       // no encryption
#define PC_SERIAL_BAUD 115200 // Serial to PC baud

#if defined(COORDINATOR_AP_MODE)
const char* coordinator_ap_ssid = "rcv_cordcordcord"; // Set this to your AP coordinator's SSID, with prefix
#endif

//uint8_t actuator_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // ESP32 v2 a
//uint8_t actuator_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x54}; // ESP32 v2 b
uint8_t actuator_node_mac[6] = {0x0C, 0x8B, 0x95, 0x95, 0x81, 0xD0}; // ESP32 v2 d

#include <esp_now.h>
#include <WiFi.h>
#include <Trill.h>

Trill trillFlex; // for Trill Flex
boolean flexTouchActive = false;
int flexRet;
bool flexOK=false;
Trill::Mode trillMode;


// Actuator Input Data package, must match data package format in Coordinator firmware
// ESPNow spec limits this to around 250 bytes
typedef struct actuator_data_package {
  uint8_t light_position = 0; // simplified control, set a led strip position from 0-255
  //uint32_t led_colors[neopixel_led_count]; // careful, this can fill up the max 250 byte data package very quickly!
  char msg[8] = "ALLGOOD";
} actuator_data_package;
actuator_data_package dataOUT;

esp_now_peer_info_t actuator_node_info;

//timer for transmission rate
unsigned long timer = 0;
//const int transmissionPeriod_ms = 15;   // desired transmission period in milliseconds
const int transmissionDelay_ms = 30;   // desired transmission period in milliseconds
//const bool verbose = false;         // verbose serial output
const bool verbose = true;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(verbose) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  delay(1200);

  trillMode=Trill::CENTROID;
  Serial.println("Attempting to discover and configure Trill Flex");
  flexRet = trillFlex.setup(Trill::TRILL_FLEX);
  if(flexRet != 0)
  {
    Serial.print("failed to initialise trill flex, errorcode:"); Serial.println(flexRet);
    Serial.println(decodeTrillError(flexRet));
    flexOK=false;
  } else {
    Serial.println("Initialising trill flex");
    delay(10);
    trillFlex.setPrescaler(4);
    delay(10);
    trillFlex.updateBaseline();
    delay(10);
    flexOK=true;
    delay(200);
    trillFlex.setMode(trillMode);
    printTrillInfo(trillFlex);
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(actuator_node_info.peer_addr, actuator_node_mac, 6);
  actuator_node_info.channel = ESPNOW_CHANNEL;  
  actuator_node_info.encrypt = ESPNOW_ENCRYPTION;
  
  // Add peer        
  if (esp_now_add_peer(&actuator_node_info) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

  if(flexOK) { // Read touch centroids if available...
    trillFlex.read();
    if(trillFlex.getNumTouches() > 0) {
      Serial.print("FLEX: ");
      for(int i = 0; i < trillFlex.getNumTouches(); i++) {
        Serial.print(trillFlex.touchLocation(i));
        Serial.print(" ");
        Serial.print(trillFlex.touchSize(i));
        Serial.print(" ");
      }
      Serial.println("");
      // Set values to send

      dataOUT.light_position = (trillFlex.touchLocation(0) * 255) / 4000;
      Serial.print("lightpos:"); Serial.println(dataOUT.light_position);

      flexTouchActive = true;
    } else if(flexTouchActive) {
      Serial.print("FLEX: ");
      // Print a single line when touch goes off
      Serial.println("0 0");
      flexTouchActive = false;
    }
  }



  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(actuator_node_mac, (uint8_t *) &dataOUT, sizeof(dataOUT));
  if(verbose) {
    Serial.print("Sending: "); Serial.print(dataOUT.light_position);
    Serial.print(" "); Serial.print(dataOUT.msg);
    Serial.println();
  }

  if(verbose) {
    if (result == ESP_OK)
      Serial.println("Sent with success");
    else
      Serial.println("Error sending the data");
  }

  delay(transmissionDelay_ms);
}



void printTrillInfo(Trill & trill) {
  int address = trill.getAddress();

  Serial.println("Detected Trill Device: ");
  Serial.print("\t- I2C address: ");
  Serial.print("#");
  Serial.print(address, HEX);
  Serial.print(" (");
  Serial.print(address);
  Serial.println(")");

  int deviceType = trill.deviceType();
  Serial.print("\t- Trill device type: ");
  switch(deviceType) {
    case Trill::TRILL_BAR:
      Serial.println("bar");
      break;
    case Trill::TRILL_SQUARE:
      Serial.println("square");
      break;
    case Trill::TRILL_HEX:
      Serial.println("hex");
      break;
    case Trill::TRILL_RING:
      Serial.println("ring");
      break;
    case Trill::TRILL_CRAFT:
      Serial.println("craft");
      break;
  	case Trill::TRILL_FLEX:
      Serial.print("flex");
      break;
    case Trill::TRILL_UNKNOWN:
      Serial.println("unknown");
      break;
    case Trill::TRILL_NONE:
      Serial.println("none");
      break;
  }
    int firmwareRev = trill.firmwareVersion();
    Serial.print("\t- Firmware version: ");
    Serial.println(firmwareRev);

    int mode = trill.getMode();
    Serial.print("\t- Sensor mode: ");
    switch(mode) {
      case Trill::CENTROID:
        Serial.println("centroid");
        break;
      case Trill::RAW:
        Serial.println("raw");
        break;
      case Trill::BASELINE:
        Serial.println("baseline");
        break;
      case Trill::DIFF:
        Serial.println("differential");
        break;
      case Trill::AUTO:
        Serial.println("auto");
        break;
    }

    Serial.print("\t- Number of available centroid dimensions: ");
    if(trill.is1D()) {
      Serial.println(1);
    } else if(trill.is2D()) {
      Serial.println(2);
    } else {
      Serial.println(0);
    }

    int numChannels = trill.getNumChannels();
    Serial.print("\t- Number of capacitive channels: ");
    Serial.println(numChannels);

    int numButtons = trill.getNumButtons();
    Serial.print("\t- Number of button channels: ");
    Serial.println(numButtons);
}

// See: https://github.com/BelaPlatform/Trill-Arduino/blob/master/Trill.cpp function begin()
String decodeTrillError(int errcode) {
  String res;
  switch(errcode) {
    case -2:
      res = "i2c address does not match known i2c addresses for Trill boards";
      break;
    case 2:
      res = "Unable to identify device";
      break;
    case -3:
      res = "Unknown device type";
      break;
    case -1:
      res = "Unknown device mode";
      break;
    case 0:
      res = "Successful intiialization";
      break;
    default:
      res = "Unknown error code";
      break;
  }
  return res;
}








// Init ESP Now with fallback
// void InitESPNow() {
//   WiFi.disconnect();

//   if (esp_now_init() == ESP_OK) {
//     Serial.println("ESPNow Init Success");
//   }
//   else {

//     Serial.println("ESPNow Init Failed");
//     // Retry InitESPNow, add a counte and then restart?
//     // InitESPNow();
//     // or Simply Restart
//     ESP.restart();
//   }

// }

// #if defined(COORDINATOR_AP_MODE)

// // config AP SSID
// void configDeviceAP() {
//   bool result = WiFi.softAP(coordinator_ap_ssid, "rcv_1_Password", ESPNOW_CHANNEL, 0);
//   if (!result) {
//     Serial.println("AP Config failed.");
//   } else {
//     Serial.println("AP Config Success. Broadcasting with AP: " + String(coordinator_ap_ssid));
//     Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());

//   }
// }

// #endif


// // send data to actuator node
// void sendData() {

//   const uint8_t *peer_addr = actuator_node.peer_addr;
//   esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &dataOUT, sizeof(dataOUT));

//   Serial.print("Send Status: ");
// /*
// Here's what the error codes generally mean in ESP-NOW:

// ESP_OK (0): Operation completed successfully.
// -1: Generic error.
// -2: ESP-NOW is not initialized properly.
// -3: Peer is not found during send operation.
// -4: Cannot allocate memory for internal data structures.
// -5: Operation not allowed in the current state.

// If you are encountering an error code of -3, 
// it suggests that the specified peer (the one you are trying to send 
// data to) is not currently in the list of known peers. Make sure that 
// you have successfully added the peer using esp_now_add_peer before 
// attempting to send data to it.
// */


//   if (result == ESP_OK) {
//     Serial.println("Success");
//   } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
//     // How did we get so far!!
//     Serial.println("ESPNOW not Init.");
//   } else if (result == ESP_ERR_ESPNOW_ARG) {
//     Serial.println("Invalid Argument");
//   } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
//     Serial.println("Internal Error");
//   } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
//     Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//   } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
//     Serial.println("Peer not found.");
//   } else {
//     Serial.println("Not sure what happened");
//   }

// }


// // callback when data is sent to node
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   char macStr[18];
//   if(verbose) {
//     snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//     Serial.print("Last Packet Sent to: "); 
//     Serial.println(macStr);
//     Serial.print("Last Packet Send Status: ");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//   }
// }



// // Check if the nodes are paired. If not, make a pairing
// bool manageNodes() {
//   if (actuator_node.channel == ESPNOW_CHANNEL) {
//     //if (DELETEBEFOREPAIR) deleteActuatorNode();
//     Serial.print("Reciever Status: ");
    
//     // check if the node is registered as a peer
//     bool exists = esp_now_is_peer_exist(actuator_node.peer_addr);

//     if ( exists) {
//       Serial.println("Already Paired");
//       return true;
//     } else {
//       char macStr[18];

//       // attempt pairing with node ESP32
//       esp_err_t addStatus = esp_now_add_peer(&actuator_node);
//       if (addStatus == ESP_OK) {
//         // Pair success
//         uint8_t mac_addr[6] = {actuator_node_mac[0], actuator_node_mac[1], actuator_node_mac[2], actuator_node_mac[3], actuator_node_mac[4], actuator_node_mac[5] };
//         snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//         Serial.print("Actuator Node added to peer list with mac ");
//         Serial.println(macStr);
//         return true;
//       } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//         // How did we get so far!!
//         Serial.println("ESPNOW Not Initialized!");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
//         Serial.println("Invalid Argument");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
//         Serial.println("Peer list full");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
//         Serial.println("Out of memory");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
//         Serial.println("Peer Exists");
//         return true;
//       } else {
//         Serial.println("Not sure what happened");
//         return false;
//       }      

//     }
//   } else {
//     Serial.println("No Actuator Node found to process");
//     return false;
//   }
// }


// ////////////////////////////////////////////////////////////////////////

// void setup() {

//   Serial.begin(PC_SERIAL_BAUD);
//   delay(1500);
//   Serial.println("Booting ESPNow Reciever");

//   Serial.print("USING ESP32 CHIP IN");

//   #if defined(COORDINATOR_AP_MODE)
//   // Set up coordinator in AP mode, with its own network SSID
//   // that is discoverable by nodes. 
//   Serial.println("AP MODE");
//   WiFi.mode(WIFI_AP);
//   configDeviceAP();
//   Serial.print("AP Config Successful with SSID "); Serial.println(coordinator_ap_ssid);
//   Serial.print("AP MAC ADDRESS: "); Serial.println(WiFi.softAPmacAddress());
//   #else
//   Serial.println(" STA MODE");
//   // Set device in STA mode.
//   // Add peer nodes directly by MAC address.
//   WiFi.mode(WIFI_STA);
//   delay(100);
//   Serial.print("Station MAC Address ");
//   Serial.println(WiFi.macAddress());
//   #endif

//   // Init ESPNow with a fallback logic
//   InitESPNow();
//   delay(100);
//   manageNodes();

//   Serial.print("dataOUT to actuator node has length "); Serial.println(sizeof(dataOUT));
//   esp_now_register_send_cb(OnDataSent);

// }


// void loop() {

//   // 1. Read any incoming messages from Python/PC into dataOUT

//   if((millis() - timer) > transmissionPeriod_ms) { // transfer rate timer
//     if (actuator_node.channel == ESPNOW_CHANNEL) { // check if actuator_node channel is defined
//       // Add actuator node as a peer if it has not been added already
//       bool isPaired = manageNodes();
//       if (isPaired) {     

        
//         // 2. For now just create some random data....
//         dataOUT.light_position += 1; // 0 - 255
//         if(dataOUT.light_position > 255)
//           dataOUT.light_position = 0;
//         Serial.print("Set light_position to: "); Serial.println(dataOUT.light_position);
//         strncpy(dataOUT.msg, "SENTLPS", 7);
//         dataOUT.msg[7] = '\0';
//         Serial.print("Set msg to: "); Serial.println(dataOUT.msg);
  
//         // 3. Transmit data to actuator node
//         sendData();

//       } else {
//         Serial.println("Unable to pair with actuator node!");
//       }
//     }
//     else {
//       Serial.println("No actuator node configuration available!");
//     }
//     timer=millis();
//   }


//   // 1. read any serial messages available from python
//   if (Serial.available() > 0) {
//     String cmd = Serial.readString();
// 		Serial.println(cmd);
//     // Populate dataOUT & send
//     //esp_err_t result = esp_now_send(nodeAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings)); // Send message via ESP-NOW
//     // if (result == ESP_OK) {
//     //   Serial.println("Sent with success");
//     // }
//     // else {
//     //   Serial.println("Error sending the data");
//     // }
//   }

// }
