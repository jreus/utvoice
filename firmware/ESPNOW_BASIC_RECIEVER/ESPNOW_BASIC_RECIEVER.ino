/**
   ESPNOW - Basic communication - Reciever
  Stupid simple serial communication with a host PC.
  This sketch is meant to be paired with the serial2osc.py simple serial to OSC script
  on the PC side.


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

#define USE_TRILL // uncomment this line if transmitter is using a Trill sensor

#define CHANNEL 1 // ESPNow channel
#define BAUD 115200 // Serial to PC baud

#define PLOT_SERIAL false // Format Serial for Arduino plotter

// Set reciever SSID (prefix should match SENDER's SCAN_FOR_PREFIX)
const char* reciever_SSID = "rcv_thatonelaptop"; 


#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif


// Data package, must match data package in Reciever firmware
// ESPNow spec limits this to around 250 bytes
typedef struct data_package {
#ifdef USE_TRILL
  int numTouches = 0;
  int touchLocations[5];
  int touchSizes[5];
  bool touchActive = false;
#endif
  char msg[8];
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
  bool result = WiFi.softAP(reciever_SSID, "rcv_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(reciever_SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());

  }
}


// TODO: Test this on an ESP8266
#ifdef ESP8266
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t data_len) {
#endif
#ifdef ESP32
void OnDataRecv(const esp_now_recv_info *mac_addr, const uint8_t *incomingData, int data_len) {
#endif
 char macStr[18];
  memcpy(&myData, incomingData, sizeof(myData));
  
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); 
  Serial.println(macStr);

  // Package data for Python client

#ifdef USE_TRILL
  Serial.print("nt:"); Serial.print(myData.numTouches);
  Serial.print(",ta:"); Serial.print((int)myData.touchActive);
  for(int i=0; i<myData.numTouches; i++) {
    Serial.print(",tl"); Serial.print(i); Serial.print(":"); Serial.print(myData.touchLocations[i]);
    Serial.print(",ts"); Serial.print(i); Serial.print(":"); Serial.print(myData.touchSizes[i]);
  }
  Serial.println();
#endif
  
  // // Package for Arduino serial plotter
  // Serial.print("a:"); Serial.print(myData.A2); // alpha datapoint code followed by value in string format
  // Serial.print(",b:"); Serial.print(myData.A4); // alpha datapoint code followed by value in string format
  // Serial.print(",c:"); Serial.println(myData.touch1); // alpha datapoint code followed by value in string format

  Serial.print("Bytes recieved: "); Serial.println(data_len);

  
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
