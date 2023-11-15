// Get MAC Address ESP32
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

/*
Known MAC addresses

Angelika's Feather ESP32 v1 dev boards.
A: 30:AE:A4:20:94:28
B: 30:AE:A4:23:74:AC

*/

#include "WiFi.h"
 
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}