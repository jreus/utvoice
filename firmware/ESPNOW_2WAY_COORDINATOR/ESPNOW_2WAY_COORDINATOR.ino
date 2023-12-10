/**
ESPNOW 2-way Reciever Module
Allows host PC to recieve sensor data from nodes
and send actuator messages to nodes.

This sketch is meant to be paired with the serial2osc.py simple serial to OSC script 
on the PC side.

See: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

<< This Device is Reciever >>

   Codeflow: Sender (Primary)
   Step 1 : ESPNow Init on Sender and set it in STA mode
   Step 2 : Start scanning for Receiver ESP32 (we have added a prefix of `rcv` to the SSID of the receiver for an easy setup)
   Step 3 : Once found, add the Reciever as a peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Sender to Reciever

   Codeflow: Reciever (Secondary)
   Step 1 : ESPNow Init on Receiver
   Step 2 : Update the SSID of Reciever with a prefix of `rcv` so it can be found easily by the sender.
   Step 3 : Set Receiver in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Sender and Reciever have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Sender/Reciever.
         Any devices can act as either one or both simultaneously.
*/

#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif


#define USE_TRILL // uncomment this line if any NODES are using a Trill sensor
//#define USE_ACCEL // uncomment this line if any NODES are using an Accelerometer
#define USE_SERVO // uncomment this line if any NODES are controlling a Servo/Linear Actuator
#define ESPNOW_CHANNEL 1 // ESPNow channel
#define ESPNOW_ENCRYPTED false
#define BAUD_PC 115200 // Serial to PC baud

// Set reciever SSID (prefix should match SENDER's SCAN_FOR_PREFIX)
const char* reciever_SSID = "rcv_thatonelaptop"; 

// THE MAC Address of the NODE DEVICE
uint8_t nodeAddress[] = {0xE8, 0x9F, 0x6D, 0x32, 0xFF, 0x50}; // ESP32 V2 c)
esp_now_peer_info_t nodeInfo;
// Variable to store if sending data was successful
String send_success;

// Incoming Data package format, must match data package in NODE firmware
// ESPNow spec limits this to around 250 bytes
typedef struct incoming_data_package {
#ifdef USE_TRILL
  uint8_t numTouches = 0;
  uint16_t touchLocations[5];
  uint16_t touchSizes[5];
  bool touchActive = false;
#endif
#ifdef USE_ACCEL
  uint16_t xa=0;
  uint16_t ya=0;
  uint16_t za=0;
#endif
  char msg[8];
} incoming_data_package;
incoming_data_package dataIN;


// Outgoing Data package format, must match data package in NODE firmware
typedef struct outgoing_data_package {
#ifdef USE_SERVO
  uint8_t targetPos = 0; // angle from 0-180
#endif
  char msg[8];
} outgoing_data_package;
outgoing_data_package dataOUT;


// Init ESP Now with fallback logic
int InitESPNow() {
  WiFi.disconnect();
  #if defined(ESP32)
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

  #if defined(ESP8266)
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return 1;
  }
  #endif
  return 0;
}

// config AP SSID
void configDeviceAP() {
  bool result = WiFi.softAP(reciever_SSID, "rcv_1_Password", ESPNOW_CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(reciever_SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());

  }
}

// Callback when data is received
// HANDLE INCOMING DATA FROM CLIENTS & SEND TO PYTHON
// TODO: Test this on ESP8266
#if defined(ESP8266)
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t data_len) {
#endif
#if defined(ESP32)
void OnDataRecv( const uint8_t * mac_addr, const uint8_t *incomingData, int data_len ) {
#endif
 char macStr[18];
 bool firstPrinted=false;
  memcpy(&dataIN, incomingData, sizeof(dataIN));
  
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);

  // Package incoming data for Python client
#if defined(USE_TRILL)
  if(!firstPrinted)
    firstPrinted=true;
  else
    Serial.print(",");
  Serial.print("nt:"); Serial.print(dataIN.numTouches);
  Serial.print(",ta:"); Serial.print((int)dataIN.touchActive);
  for(int i=0; i<dataIN.numTouches; i++) {
    Serial.print(",tl"); Serial.print(i); Serial.print(":"); Serial.print(dataIN.touchLocations[i]);
    Serial.print(",ts"); Serial.print(i); Serial.print(":"); Serial.print(dataIN.touchSizes[i]);
  }
#endif

#if defined(USE_ACCEL)
  if(!firstPrinted)
    firstPrinted=true;
  else
    Serial.print(",");
  Serial.print("ax:"); Serial.print(dataIN.ax);
  Serial.print("ay:"); Serial.print(dataIN.ay);
  Serial.print("az:"); Serial.print(dataIN.az);
#endif

#if defined(USE_TRILL) || defined(USE_ACCEL)
  Serial.println();
#endif
  
  Serial.print("Bytes recieved: "); Serial.println(data_len);
}



// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0) {
    send_success = "Delivery Success :)";
  }
  else{
    send_success = "Delivery Fail :(";
  }
}


////////////////////////////////////////////////////////////////////////

void setup() {
  uint8_t res;
  Serial.begin(BAUD_PC);
  delay(1000);
  Serial.println();
  Serial.println("ESPNow Reciever");

  #if defined(ESP32)
  Serial.println("USING ESP32 CHIP");
  #endif

  #if defined(ESP8266)
  Serial.println("USING ESP8266 CHIP");
  #endif

  //WiFi.mode(WIFI_STA);

  // This sets up a local network where this device operates as a coordinator (AP)
  // and give it a SSID that can be autodiscovered
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  Serial.println("Init ESPNOW");
  delay(1000);
  if(InitESPNow() != 0)
    return;

  // Once ESPNow is successfully Init, register Send CB to
  // get the status of Trasnmitted packet
  Serial.println("Register Data Sent Callback");
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer NODE(s) TODO: enable multiple nodes
  Serial.println("Register Peers");
  memcpy(nodeInfo.peer_addr, nodeAddress, 6);
  nodeInfo.channel = ESPNOW_CHANNEL;  
  nodeInfo.encrypt = ESPNOW_ENCRYPTED;
  
  if (esp_now_add_peer(&nodeInfo) != ESP_OK){
    Serial.println("Failed to add NODE as peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  Serial.println("Register Data Recieved Callback");
  esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
  // HANDLE INCOMING DATA FROM PYTHON & SEND TO CLIENTS

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
  delay(10);
}

