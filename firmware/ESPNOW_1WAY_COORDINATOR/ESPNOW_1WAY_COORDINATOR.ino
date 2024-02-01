/**
   ESPNOW - GENERAL FIRMWARE FOR A COORDINATOR COMMUNICATING WITH
   1-way sensor or actuator nodes.

   Depending on configuration of preprocessor flags, the coordinator will:
    * recieve sensor data from one node 
    * send actuator signals to one node

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/

//#define ENABLE_SENSOR_NODE
#define ENABLE_ACTUATOR_NODE
#define COORDINATOR_AP_MODE // uncomment to run coordinator in AP-mode / broadcasting a network ssid
#define ESPNOW_CHANNEL 11 // ESPNow channel, must match nodes firmware

// PC Communication
#define PC_SERIAL_BAUD 115200 // Serial to PC baud


#if defined(COORDINATOR_AP_MODE)
const char* coordinator_ap_ssid = "rcv_cordcordcord"; // Set this to your AP coordinator's SSID, with prefix
#endif

#if defined(ENABLE_ACTUATOR_NODE)
//uint8_t actuator_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // ESP32 v2 a
uint8_t actuator_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x54}; // ESP32 v2 b
#endif

#if defined(ENABLE_SENSOR_NODE)
//uint8_t sensor_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // ESP32 v2 a
uint8_t sensor_node_mac[6] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x54}; // ESP32 v2 b
#endif


// Sensors used by node
//#define USE_ADXL313 // using an ADXL313 3-axis accelerometer
//#define USE_TRILL       // uncomment if using a Trill sensor
//#define TRILL_SENSOR_MODE 1    // 0 for DIFF, 1 for CENTROID

// Actuators used by node
//#define USE_SERVO180
#define USE_NEOPIXEL_STRIP

#if defined(ESP32)
#include <esp_now.h>
#include <WiFi.h>
#endif

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

#if defined(ENABLE_SENSOR_NODE)
// Sensor Output Data package, must match data package format in node firmware
// ESPNow spec limits this to around 250 bytes
typedef struct sensor_data_package {
#if defined(USE_TRILL)
  #if TRILL_SENSOR_MODE == 0
  // Trill DIFF parameters
  uint8_t numSensors = 30;
  int sensorValues[30];     // raw sensor values minus baseline
  #else
  // Trill CENTROID parameters
  int numTouches = 0;       // number of activate touches
  int touchLocations[5];    // touch location points
  int touchSizes[5];        // touch centroid sizes
  bool touchActive = false; // touch active
  #endif
#endif
#if defined(USE_ADXL313)
  int ax = 0;
  int ay = 0;
  int az = 0;
#endif
  char msg[8];
} sensor_data_package;
sensor_data_package dataIN;

#endif


#if defined(ENABLE_ACTUATOR_NODE)
// Actuator Input Data package, must match data package format in Coordinator firmware
// ESPNow spec limits this to around 250 bytes
typedef struct actuator_data_package {
  #if defined(USE_NEOPIXEL_STRIP)
  uint8_t light_position = 0; // simplified control, set a led strip position from 0-255
  //uint32_t led_colors[neopixel_led_count]; // careful, this can fill up the max 250 byte data package very quickly!
  #endif
  char msg[8];
} actuator_data_package;
actuator_data_package dataOUT;

#if defined(ESP32)
esp_now_peer_info_t actuator_node;
#else if defined(ESP8266)
uint8_t actuator_node_role = ESP_NOW_ROLE_SLAVE;
uint8_t actuator_node_key[0] = {};/////////no key
//uint8_t actuator_node_key[16] = {1,3,3,4,j,9,8,6,t,v,l,2,5,d,b,1};
uint8_t actuator_node_key_len = sizeof(actuator_node_key);

#endif

//timer for transmission rate
unsigned long timer = 0;
//const int transmissionPeriod_ms = 15;   // desired transmission period in milliseconds
const int transmissionPeriod_ms = 3500;   // desired transmission period in milliseconds
const bool verbose = false;         // verbose serial output

#endif





// Init ESP Now with fallback
void InitESPNow() {
  //WiFi.disconnect();

#ifdef ESP32
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {

    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
#endif

#ifdef ESP8266
  // Init ESP-NOW
  //if (esp_now_init() == 0) {
  if (esp_now_init() == ERR_OK) {
    Serial.println("ESPNOW INIT SUCCESS");
  } else {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    //return;
    delay(100);
  }
  
  // Serial.println("Set role to ESP_NOW_ROLE_CONTROLLER");
  // esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  Serial.print("Add peer on chanel "); Serial.println(ESPNOW_CHANNEL);
  int add = esp_now_add_peer(actuator_node_mac, actuator_node_role, ESPNOW_CHANNEL, NULL, 0);
  Serial.print("Add actuator node as peer: ");
  if(add == 0)
    Serial.println("Pairing Success " + String(add));
  else
    Serial.println("Pairing Failed " + String(add));

#endif


}

#if defined(COORDINATOR_AP_MODE)

// config AP SSID
void configDeviceAP() {
  bool result = WiFi.softAP(coordinator_ap_ssid, "rcv_1_Password", ESPNOW_CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(coordinator_ap_ssid));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());

  }
}

#endif


#if defined(ENABLE_SENSOR_NODE)

// TODO: Test this on an ESP8266
#if defined(ESP8266)
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t data_len) {
#elseif defined(ESP32)
//void OnDataRecv(const esp_now_recv_info *mac_addr, const uint8_t *incomingData, int data_len) {
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
#endif
  char macStr[18];
  memcpy(&dataIN, incomingData, sizeof(dataIN));
  uint8_t mac_str[6] = {actuator_node_mac[0], actuator_node_mac[1], actuator_node_mac[2], actuator_node_mac[3], actuator_node_mac[4], actuator_node_mac[5]}
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); 
  Serial.println(macStr);

  // Package sensor data for Python client

#if defined(USE_TRILL)

  #if TRILL_SENSOR_MODE == 0
  // Recieved trill data in RAW/DIFF mode
  Serial.print("ns:"); Serial.print(dataIN.numSensors);
  for(uint8_t i=0; i < dataIN.numSensors; i++) {
    int data = dataIN.sensorValues[i];
    Serial.print(",t"); Serial.print(i); Serial.print(":");
    Serial.print(dataIN.sensorValues[i]);
  }
  Serial.println();

  #else
  // Recieved trill data in TOUCH/CENTROID mode

  Serial.print("nt:"); Serial.print(dataIN.numTouches);
  Serial.print(",ta:"); Serial.print((int)dataIN.touchActive);
  for(int i=0; i<dataIN.numTouches; i++) {
    Serial.print(",tl"); Serial.print(i); Serial.print(":"); Serial.print(dataIN.touchLocations[i]);
    Serial.print(",ts"); Serial.print(i); Serial.print(":"); Serial.print(dataIN.touchSizes[i]);
  }
  Serial.println();
  #endif

#endif
  
  Serial.print("Bytes recieved: "); Serial.println(data_len);
}

#endif


#if defined(ENABLE_ACTUATOR_NODE)

// send data to actuator node
void sendData() {


  #if defined(ESP32)
  const uint8_t *peer_addr = actuator_node.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &dataOUT, sizeof(dataOUT));
  #endif
  
  #if defined(ESP8266)
  int result = esp_now_send(actuator_node_mac, (uint8_t *) &dataOUT, sizeof(dataOUT));
  #endif

  Serial.print("Send Status: ");
/*
Here's what the error codes generally mean in ESP-NOW:

ESP_OK (0): Operation completed successfully.
-1: Generic error.
-2: ESP-NOW is not initialized properly.
-3: Peer is not found during send operation.
-4: Cannot allocate memory for internal data structures.
-5: Operation not allowed in the current state.

If you are encountering an error code of -3, 
it suggests that the specified peer (the one you are trying to send 
data to) is not currently in the list of known peers. Make sure that 
you have successfully added the peer using esp_now_add_peer before 
attempting to send data to it.
*/


  #if defined(ESP32)
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
  #endif
  
  #if defined(ESP8266)
  // See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
  if (result == 0) {
    Serial.println("Success");
  } else if (result == 1) {
    Serial.println("Error 1: Send Failed!");
  } else {
    Serial.println("Error Code "); Serial.print(result); Serial.println(" ... Not sure what happened");
  }
  #endif

}

// callback when data is sent to node
#ifdef ESP8266
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
#endif
#ifdef ESP32
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  char macStr[18];
  if(verbose) {
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Last Packet Sent to: "); 
    Serial.println(macStr);
    Serial.print("Last Packet Send Status: ");
    #if defined(ESP32)
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif 
    
    #if defined(ESP8266)
    Serial.println(status == 0 ? "Delivery Success" : "Delivery Fail");
    #endif
  }
}

#endif


// Check if the nodes are paired. If not, make a pairing
bool manageNodes() {
  #if defined(ESP32)
  if (actuator_node.channel == ESPNOW_CHANNEL) {
  #else
  if (true) { // TODO: Some other way to test is actuator node exists..
  #endif
    //if (DELETEBEFOREPAIR) deleteActuatorNode();
    Serial.print("Reciever Status: ");
    
    // check if the node is registered as a peer
    #if defined(ESP32)
    bool exists = esp_now_is_peer_exist(actuator_node.peer_addr);
    #endif 
    
    #if defined(ESP8266)
    bool exists = esp_now_is_peer_exist(actuator_node_mac);
    #endif

    if ( exists) {
      Serial.println("Already Paired");
      return true;
    } else {
      char macStr[18];

      #if defined(ESP32)
      // attempt pairing with node ESP32
      esp_err_t addStatus = esp_now_add_peer(&actuator_node);
      if (addStatus == ESP_OK) {
        // Pair success
        uint8_t mac_addr[6] = {actuator_node_mac[0], actuator_node_mac[1], actuator_node_mac[2], actuator_node_mac[3], actuator_node_mac[4], actuator_node_mac[5] };
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.print("Actuator Node added to peer list with mac ");
        Serial.println(macStr);
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Initialized!");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }      
      #else if defined(ESP8266)
      // For ESP8266 you need to use a different add peer function
      // See: https://github.com/esp8266/Arduino/blob/master/tools/sdk/include/espnow.h
      int addStatus = esp_now_add_peer(actuator_node_mac, actuator_node_role, ESPNOW_CHANNEL, actuator_node_key, actuator_node_key_len);
      if (addStatus == 0) {
        // Pair success
        uint8_t mac_addr[6] = {actuator_node_mac[0], actuator_node_mac[1], actuator_node_mac[2], actuator_node_mac[3], actuator_node_mac[4], actuator_node_mac[5] };
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.print("Actuator Node added to peer list ");
        Serial.println(macStr);
        return true;
      } else {
        Serial.print("Error code "); Serial.print(addStatus);
        Serial.println(" ... Not sure what happened");
        return false;
      }
      #endif


    }
  } else {
    Serial.println("No Actuator Node found to process");
    return false;
  }
}


////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(PC_SERIAL_BAUD);
  delay(1500);
  Serial.println("Booting ESPNow Reciever");

  #if defined(ESP32)
  Serial.print("USING ESP32 CHIP IN");
  #endif

  #if defined(ESP8266)
  Serial.print("USING ESP8266 CHIP");
  #endif

  #if !defined(ESP32) && !defined(ESP8266)
  Serial.println("UNKNOWN CHIP! PROBABLY VERY LITTLE WILL WORK...");
  #endif

  #if defined(COORDINATOR_AP_MODE)
  // Set up coordinator in AP mode, with its own network SSID
  // that is discoverable by nodes. 
  Serial.println("AP MODE");
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP Config Successful with SSID "); Serial.println(coordinator_ap_ssid);
  Serial.print("AP MAC ADDRESS: "); Serial.println(WiFi.softAPmacAddress());
  #else
  Serial.println(" STA MODE");
  // Set device in STA mode.
  // Add peer nodes directly by MAC address.
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print("Station MAC Address ");
  Serial.println(WiFi.macAddress());
  #endif

  // Init ESPNow with a fallback logic
  InitESPNow();
  delay(100);
  manageNodes();


  #if defined(ENABLE_SENSOR_NODE)
  Serial.print("dataIN from sensor node has length "); Serial.println(sizeof(dataIN));
  esp_now_register_recv_cb(OnDataRecv);
  #endif
  #if defined(ENABLE_ACTUATOR_NODE)
  Serial.print("dataOUT to actuator node has length "); Serial.println(sizeof(dataOUT));
  esp_now_register_send_cb(OnDataSent);
  #endif


}


void loop() {

#if defined(ENABLE_ACTUATOR_NODE)

  // 1. Read any incoming messages from Python/PC into dataOUT

  if((millis() - timer) > transmissionPeriod_ms) { // transfer rate timer
    if (actuator_node.channel == ESPNOW_CHANNEL) { // check if actuator_node channel is defined
      // Add actuator node as a peer if it has not been added already
      bool isPaired = manageNodes();
      if (isPaired) {     

        
        // 2. For now just create some random data....
        #if defined(USE_NEOPIXEL_STRIP)
        dataOUT.light_position += 1; // 0 - 255
        if(dataOUT.light_position > 255)
          dataOUT.light_position = 0;
        Serial.print("Set light_position to: "); Serial.println(dataOUT.light_position);
        #endif
        strncpy(dataOUT.msg, "SENTLPS", 7);
        dataOUT.msg[7] = '\0';
        Serial.print("Set msg to: "); Serial.println(dataOUT.msg);
  
        // 3. Transmit data to actuator node
        sendData();

      } else {
        Serial.println("Unable to pair with actuator node!");
      }
    }
    else {
      Serial.println("No actuator node configuration available!");
    }
    timer=millis();
  }


  // 1. read any serial messages available from python
  if (Serial.available() > 0) {
    String cmd = Serial.readString();
		Serial.println(cmd);
    // Populate dataOUT & send
    //esp_err_t result = esp_now_send(nodeAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings)); // Send message via ESP-NOW
    // if (result == ESP_OK) {
    //   Serial.println("Sent with success");
    // }
    // else {
    //   Serial.println("Error sending the data");
    // }
  }

#endif

}
