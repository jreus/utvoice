/**
   ESPNOW - FIRMWARE FOR 1-WAY SENSOR NODE sending to this COORDINATOR 
   which communicates with python serial-2-osc client on the host PC.

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/

#define USE_TRILL       // uncomment if using a Trill sensor
#define TRILL_DEVICE_TYPE 1  // 0 for CRAFT, 1 for FLEX
#define TRILL_SENSOR_MODE 0    // 0 for DIFF, 1 for CENTROID
//#define USE_ADXL313 // using an ADXL313 3-axis accelerometer

#define ESPNOW_CHANNEL 11          // pick a channel (good to choose one not being used by ambient wifi)
#define ESPNOW_ENCRYPTION 0       // no encryption
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// PC Communication rate
#define PC_SERIAL_BAUD 115200 // Serial to PC baud

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()


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




// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  char macStr[18];
  memcpy(&dataIN, incomingData, sizeof(dataIN));
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Packet Recv from: "); 
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

  // TODO: package accelerometer data
  
  Serial.print("Bytes recieved: "); Serial.println(data_len);
}


void setup() {

  Serial.begin(PC_SERIAL_BAUD);
  delay(1500);
  Serial.println("Booting ESPNow Reciever");

  Serial.println(" STA MODE");
  // Set device in STA mode.
  // Add peer nodes directly by MAC address.
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);
  Serial.print("Station MAC Address ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}


void loop() {
  // chill
}
