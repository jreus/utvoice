/**
   ESPNOW - Basic communication - Reciever

  See: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

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


// Uncomment the define statement here that matches the ESP chip you're programming.
//#define USING_ESP32
//#define USING_ESP8266

#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif


#define CHANNEL 1
#define BAUD 115200

#define PLOT_SERIAL false

// Stupid simple serial communication with host PC

// Data package, must match data package in Reciever firmware
// ESPNow spec limits this to around 250 bytes
typedef struct data_package {
  uint16_t A2;
  uint16_t A4;
  uint16_t touch1;
} data_package;
data_package myData;


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();

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
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
#endif


}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "rcv_1";
  bool result = WiFi.softAP(SSID, "rcv_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());

  }
}


// callback when data is recv from Sender

// #ifdef USING_ESP32
// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
//   OnDataRecvCommon((uint8_t *)mac_addr, (uint8_t *)incomingData, (uint8_t) data_len);
// }
// #endif

// #ifdef USING_ESP8266
// void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t data_len) {
//   OnDataRecvCommon(mac_addr, incomingData, data_len);
// }
// #endif

#ifdef ESP8266
void OnDataRecvCommon(uint8_t *mac_addr, uint8_t *incomingData, uint8_t data_len) {
#endif
#ifdef ESP32
void OnDataRecv(const esp_now_recv_info *mac_addr, const uint8_t *incomingData, int data_len) {
#endif
 char macStr[18];
  memcpy(&myData, incomingData, sizeof(myData));
  
  #if (PLOT_SERIAL == true)
  // Print verbose serial info and plot incoming data
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); 
  Serial.println(macStr);

  // Package for Arduino serial plotter
  Serial.print("a:"); Serial.print(myData.A2); // alpha datapoint code followed by value in string format
  Serial.print(",b:"); Serial.print(myData.A4); // alpha datapoint code followed by value in string format
  Serial.print(",c:"); Serial.println(myData.touch1); // alpha datapoint code followed by value in string format

  Serial.print("Bytes recieved: "); Serial.println(data_len);
  #else
  // Package data for Python client
  Serial.print("a:"); Serial.print(myData.A2); // alpha datapoint code followed by value in string format
  Serial.print(",b:"); Serial.print(myData.A4); // alpha datapoint code followed by value in string format
  Serial.print(",c:"); Serial.println(myData.touch1); // alpha datapoint code followed by value in string format
  #endif

  
}




////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(BAUD);
  delay(10);
  Serial.println();
  Serial.println("ESPNow Reciever");

  #ifdef ESP32
  Serial.println("USING ESP32 CHIP");
  #endif

  #ifdef ESP8266
  Serial.println("USING ESP8266 CHIP");
  #endif

  // Set device as a Wifi Station, so both Sender and Reciever operate as two Wifi Stations
  // and connect directly by Mac address?
  // TODO: This doesn't seem to work... use AP mode for now and see if a direct connection is possible.
  //WiFi.mode(WIFI_STA);

  // Another option is to set the device in AP mode and give it a SSID that can be autodiscovered
  WiFi.mode(WIFI_AP);
  configDeviceAP();

  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  // Init ESPNow with a fallback logic
  InitESPNow();

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

}


void loop() {
  // Chill
}
