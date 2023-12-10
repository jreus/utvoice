// Get MAC Address ESP board
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
// For ESP 8266 see: https://randomnerdtutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
// And: https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/overview
/*
Known MAC addresses

Angelika's Feather ESP32 v1 dev boards.
A: 30:AE:A4:20:94:28
B: 30:AE:A4:23:74:AC

Jon's Feather ESP32 v2 boards.
a) E8:9F:6D:2F:64:10
b) E8:9F:6D:2F:48:54
c) E8:9F:6D:32:FF:50
d) 0C:8B:95:95:81:D0

Jon's ESP8266 boards.
q) 5C:CF:7F:EF:BB:08
r) 5C:CF:7F:EF:BD:81


*/

#ifdef ESP32
#include "WiFi.h"
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.println();
#ifdef ESP32
  WiFi.mode(WIFI_MODE_STA);
#endif
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}