/**
   ESPNOW Remote LED Control

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/

#define COORDINATOR_AP_MODE // uncomment if coordinator is running in AP-mode and broadcasting a network ssid
#define ESPNOW_CHANNEL 1          // pick a channel (good to choose one not being used by ambient wifi)
#define ESPNOW_ENCRYPTION 0       // no encryption
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_NeoPixel.h>
// On ESP32 pins 6-11 are connected to SPI flash.
const uint8_t neopixel_dpin = 12; // use a pix appropriate to your ESP board
const uint8_t neopixel_led_count = 24; // how many neopixels are in the strip?
//const uint8_t neopixel_led_count = 36; // how many neopixels are in the strip?
Adafruit_NeoPixel strip(neopixel_led_count, neopixel_dpin, NEO_GRB + NEO_KHZ800);
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// Actuator Input Data package, must match data package format in Coordinator firmware
// ESPNow spec limits this to around 250 bytes
typedef struct actuator_data_package {
  uint8_t light_position = 0; // simplified control, set a led strip position from 0-255
  //uint32_t led_colors[neopixel_led_count]; // careful, this can fill up the max 250 byte data package very quickly!
  char msg[8] = "ALLGOOD";
} actuator_data_package;
actuator_data_package dataIN;



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Data received: ");
  Serial.println(len);
  memcpy(&dataIN, incomingData, sizeof(dataIN));
  Serial.print("lp:"); Serial.print(dataIN.light_position);
  Serial.print(",msg:"); Serial.print(dataIN.msg);
  Serial.println();

  // Set LED display state
  colorFill(dataIN.light_position, strip.Color(random(100,255), random(100,255), random(100,255)));


}



// Init ESP Now with fallback
void InitESPNow() {
  //WiFi.disconnect();
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

}

 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("BEGIN ESPNOW ACTUATOR NODE");



  //====== LED Strip ========
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  delay(1000);

  // Set device as a Wi-Fi Station
  Serial.println("INIT WIFI MODE STA");
  WiFi.mode(WIFI_STA);

  Serial.print("MY MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("MY CHANNEL: "); Serial.println(WiFi.channel());
  Serial.print("dataIN has length "); Serial.println(sizeof(dataIN));


  // Init ESP-NOW
  InitESPNow();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  //Serial.print(".ping.");
  delay(2000);
}



////////////////////////// LED EFFECTS ////////////////////


// Instantaneously fill strip pixels up to a fraction from 0-100%
// the percentage is a number from 0-255, so 128 is halfway
// Pass in color as a single 'packed' 32-bit value, which you can get 
// by calling strip.Color(red, green, blue)
uint32_t blank_color = strip.Color(0,0,0);
void colorFill(uint8_t position, uint32_t color) {
  uint8_t maxpixel = (strip.numPixels() * position) / 255;
  Serial.print("Max Pixel:"); Serial.println(maxpixel);
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    if (i < maxpixel) {
      strip.setPixelColor(i, color); //  Set pixel's color (in RAM)
    } else {
      strip.setPixelColor(i, blank_color); //  Set pixel's color (in RAM)
    }
  }
  strip.show();                          //  Update strip to match
}



// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}



// void setup() {


//   // Setup ESPNOW...
//   WiFi.mode(WIFI_STA);
//   esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
//   Serial.println("Configuring ESPNow Node WIFI_STA");

//   // Init ESPNow with a fallback logic
//   Serial.println("Initializing ESPNow");
//   InitESPNow();

//   // Register recieve callback
//   Serial.println("Register OnDataRecv Callback");
//   esp_now_register_recv_cb(OnDataRecv);

//   // Register reciever as a peer
//   #if defined(COORDINATOR_AP_MODE)
//   ScanForCoordinator();
//   #else
//   memcpy(coordinator.peer_addr, coordinator_sta_mac, 6);
//   coordinator.channel = ESPNOW_CHANNEL;
//   coordinator.encrypt = ESPNOW_ENCRYPTION;
//   #endif

//   // Register coordinator as peer
//   while( ! manageCoordinator() ) {
//     delay(3000);
//     Serial.println("Retry connection to coordinator...");
//   }

// }




// // Set coordinator SSID prefix, or coordinator MAC address
// #if defined(COORDINATOR_AP_MODE)
// const char* coordinator_ap_ssid_prefix = "rcv_"; // Set this to your AP coordinator's SSID prefix
// #else
// // Coordinator is STA mode. Set this to your coordinator's MAC address so that this node will be able to find it.
// //uint8_t coordinator_sta_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // ESP32 v2 a
// //uint8_t coordinator_sta_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x54}; // ESP32 v2 b
// uint8_t coordinator_sta_mac[] = {0xE8, 0x9F, 0x6D, 0x32, 0xFF, 0x50}; // ESP32 v2 c
// //uint8_t coordinator_sta_mac[] = {0x5C, 0xCF, 0x7F, 0xEF, 0xBD, 0x81}; // ESP8266 r
// #endif

// #include <esp_now.h>
// #include <WiFi.h>
// #include <esp_wifi.h> // only for esp_wifi_set_channel()


// esp_now_peer_info_t coordinator;
// const bool verbose = false;         // verbose serial output


// /////////////////////////////////////////////////////////////////////////////////


// #if defined(COORDINATOR_AP_MODE)

// // Scan for coordinator AP network with SSID including coordinator_ap_ssid_prefix
// void ScanForCoordinator() {
//   int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, ESPNOW_CHANNEL); // Scan for networks on ESPNOW_CHANNEL
//   // reset on each scan
//   bool coordinatorFound = 0;
//   memset(&coordinator, 0, sizeof(coordinator));
//   Serial.println("Scanning for coordinator AP networks on channel "); Serial.println(ESPNOW_CHANNEL);
//   if (scanResults == 0) {
//     Serial.println("No WiFi devices in AP Mode found on channel");
//   } else {
//     Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices on channel");
//     for (int i = 0; i < scanResults; ++i) {
//       // Print SSID and RSSI for each device found
//       String SSID = WiFi.SSID(i);
//       int32_t RSSI = WiFi.RSSI(i);
//       String BSSIDstr = WiFi.BSSIDstr(i);

//       if (PRINTSCANRESULTS) {
//         Serial.print(i + 1);
//         Serial.print(": ");
//         Serial.print(SSID);
//         Serial.print(" (");
//         Serial.print(RSSI);
//         Serial.print(")");
//         Serial.println("");
//       }
//       delay(10);
//       // Check if the current device starts with RCV_PREFIX
//       if (SSID.indexOf(coordinator_ap_ssid_prefix) == 0) {
//         // SSID of interest
//         Serial.println("Found the Coordinator.");
//         Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
//         // Get BSSID => Mac Address of the Reciever
//         int mac[6];
//         if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
//           for (int ii = 0; ii < 6; ++ii ) {
//             coordinator.peer_addr[ii] = (uint8_t) mac[ii];
//           }
//         }

//         coordinator.channel = ESPNOW_CHANNEL;
//         coordinator.encrypt = ESPNOW_ENCRYPTION;

//         coordinatorFound = 1;
//         // Break after we find one matching AP network / coordinator
//         break;
//       }
//     }
//   }

//   if (coordinatorFound) {
//     Serial.println("Coordinator Found, processing..");
//   } else {
//     Serial.println("Coordinator Not Found, trying again.");
//   }

//   WiFi.scanDelete(); // clean up memory
// }
// #endif

// // Check if the coordinator is already paired with this node.
// // If not, make a pairing
// bool manageCoordinator() {

//   if (coordinator.channel == ESPNOW_CHANNEL) {
//     if (DELETEBEFOREPAIR) deleteCoordinator();

    
//     // check if the coordinator is registered as a peer
//     bool exists = esp_now_is_peer_exist(coordinator.peer_addr);
//     if ( exists) {
//       if(verbose) {
//         Serial.print("Coordinator Status: ");
//         Serial.println("Already Paired");
//       }
//       return true;
//     } else {
//       // attempt pairing with coordinator
//       esp_err_t addStatus = esp_now_add_peer(&coordinator);
//       if (addStatus == ESP_OK) {
//         // Pair success
//         Serial.println("Coordinator added to peer list");
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
//     if(verbose) Serial.println("No Reciever found to process");
//     return false;
//   }
// }

// void deleteCoordinator() {
//   esp_err_t delStatus = esp_now_del_peer(coordinator.peer_addr);
//   Serial.print("Coordinator Delete Status: ");
//   if (delStatus == ESP_OK) {
//     // Delete success
//     Serial.println("Success");
//   } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//     // How did we get so far!!
//     Serial.println("ESPNOW Not Init");
//   } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
//     Serial.println("Invalid Argument");
//   } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
//     Serial.println("Peer not found.");
//   } else {
//     Serial.println("Not sure what happened");
//   }
// }




// // INCOMING ACTUATOR DATA FUNCTIONS...





// ////////////////////////////////////////////////////////////////////////

