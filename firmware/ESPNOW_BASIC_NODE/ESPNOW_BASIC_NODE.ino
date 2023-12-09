/**
   ESPNOW - Basic communication system
   NODE (sends sensor data to reciever, accepts actuator signals from reciever)

  See: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

   << This Device is a NODE >>

   Codeflow: NODE (Primary)
   Step 1 : ESPNow Init on NODE and set it in STA mode
   Step 2 : Start scanning for Receiver ESP32 (we have added a prefix of `rcv` to the SSID of the receiver for an easy setup)
   Step 3 : Once found, add the Reciever as a peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from NODE to Reciever

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

#define USE_TRILL // uncomment this line if using a Trill sensor
//#define VERBOSE // Uncomment to get verbose serial output
//#define SCAN_FOR_PREFIX // uncomment to find reciever by prefix, otherwise find by mac address



#ifdef SCAN_FOR_PREFIX
const char* reciever_prefix = "rcv_"; // Set this to your reciever SSID prefix
#else
// Set this to your reciever MAC address
//uint8_t recieverAddress[] = {0x30, 0xAE, 0xA4, 0x20, 0x94, 0x29};
uint8_t recieverAddress[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x55}; // b
#endif




#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#endif

#ifdef ESP8266
// TODO: Build an ESP8266 compatible version of this firmware.
#endif


#ifdef USE_TRILL
#include <Trill.h>

Trill trillSensor;
boolean touchActive = false;
const Trill::Device TRILL_DEVICE = Trill::TRILL_CRAFT;
const Trill::Mode TRILL_MODE = Trill::CENTROID;
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
  //uint16_t A2;
  //uint16_t A4;
  //uint16_t touch1;
  //uint8_t hall;
} data_package;
data_package myData;

// Global copy of the reciever node
esp_now_peer_info_t reciever;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0


//timer for transmission freq
unsigned long timer = 0;
const int samplingPeriod_ms = 15; // desired transmission period in milliseconds
const bool verbose = false; // verbose serial output


//////////////////////////////////////////////


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
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


#ifdef SCAN_FOR_PREFIX

void ScanForReciever() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool recieverFound = 0;
  memset(&reciever, 0, sizeof(reciever));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with RCV_PREFIX
      if (SSID.indexOf(reciever_prefix) == 0) {
        // SSID of interest
        Serial.println("Found the Reciever.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Reciever
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            reciever.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        reciever.channel = CHANNEL; // pick a channel (good to choose one not being used by ambient wifi)
        reciever.encrypt = 0; // no encryption

        recieverFound = 1;
        // we are planning to have only one reciever in this sketch;
        // Hence, break after we find one
        break;
      }
    }
  }

  if (recieverFound) {
    Serial.println("Reciever Found, processing..");
  } else {
    Serial.println("Reciever Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}
#endif

// Check if the reciever is already paired with the sender.
// If not, pair the reciever with sender
bool manageReciever() {

  if (reciever.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    #ifdef VERBOSE
    Serial.print("Reciever Status: ");
    #endif
    
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(reciever.peer_addr);
    if ( exists) {
      #ifdef VERBOSE
      Serial.println("Already Paired");
      #endif
      return true;
    } else {
      // attempt pair
      esp_err_t addStatus = esp_now_add_peer(&reciever);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Reciever address added to peer list");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
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
    }
  } else {
    #ifdef VERBOSE
    Serial.println("No Reciever found to process");
    #endif
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(reciever.peer_addr);
  Serial.print("Reciever Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

uint8_t data = 0;
// send data
void sendData() {
  const uint8_t *peer_addr = reciever.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &myData, sizeof(myData));

  #ifdef VERBOSE
  #ifdef USE_TRILL
  Serial.print(myData.numTouches); Serial.print(" touches: ");
  for(int i=0; i < myData.numTouches; i++) {
    Serial.print(myData.touchLocations[i]); Serial.print(" ");
    Serial.print(myData.touchSizes[i]); Serial.print("  ");
  }
  Serial.println();
  #endif
  Serial.print("Send Status: ");
  #endif

  if (result == ESP_OK) {
    #ifdef VERBOSE
    Serial.println("Success");
    #endif
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
}

// read sensor data & load into myData
void readSensors() {
#ifdef USE_TRILL
  delay(50); // delay for stability
  trillSensor.read();
  myData.numTouches = trillSensor.getNumTouches();
  if(myData.numTouches > 0) {
    for(int i = 0; i < myData.numTouches; i++) {
      myData.touchLocations[i] = trillSensor.touchLocation(i);
      myData.touchSizes[i] = trillSensor.touchSize(i);
    }
    myData.touchActive = true;
  }
  else if(myData.touchActive) {
    // Print a single line when touch goes off
    myData.touchActive = false;
  }
#endif
  //myData.A2 = analogRead(A2); delay(1);
  //myData.A4 = analogRead(A4); delay(1);
  //myData.touch1 = touchRead(T0); delay(1); // T0=4=A5
  //myData.touch2 = touchRead(T4); // T4=13=D13
  //myData.hall = hallRead();
}

// callback when data is sent from Sender to Reciever
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  if(verbose) {
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Last Packet Sent to: "); 
    Serial.println(macStr);
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}



////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(50);

  // Setup sensors...
#ifdef USE_TRILL
  int ret = trillSensor.setup(TRILL_DEVICE);
  delay(50);
  trillSensor.setMode(TRILL_MODE);
  if(ret != 0) {
    Serial.println("failed to initialise trillSensor");
    Serial.print("Error code: ");
    Serial.println(ret);
  } else {
    Serial.println("Trill was detected!");
  }

  printTrillInfo();
#endif

  // Setup ESPNow...
  //Set this device, the Sender, in STA mode (Wifi Station) to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow Sender");
  // This is the mac address of the Sender in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());

  // Init ESPNow with a fallback logic
  InitESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register reciever as a peer
#ifdef SCAN_FOR_PREFIX
  ScanForReciever();
#else
  memcpy(reciever.peer_addr, recieverAddress, 6);
  reciever.channel = CHANNEL;
  reciever.encrypt = false;
#endif

  // Add reciever as peer
  while( ! manageReciever() ) {
    delay(3000);
    Serial.println("Retry connection to reciever...");
  }


}

void loop() {
  //ScanForReciever();
  
  if((millis() - timer) > samplingPeriod_ms) {
  // We will check if `reciever` is defined and then we proceed further
  if (reciever.channel == CHANNEL) { // check if reciever channel is defined
    // `reciever` is defined
    // Add reciever as peer if it has not been added already
    bool isPaired = manageReciever();
    if (isPaired) {
      // pair success or already paired
      // Sample datarecieverAddress
      //strcpy(myData.str, "THIS IS A CHAR ARRAY");
      // See pinout ESP32 V1: https://cdn-learn.adafruit.com/assets/assets/000/111/179/original/wireless_Adafruit_HUZZAH32_ESP32_Feather_Pinout.png
      // See pinout ESP32 V2: https://cdn-learn.adafruit.com/assets/assets/000/123/406/original/adafruit_products_Adafruit_ESP32_Feather_V2_Pinout.png
      
      // Read sensor data into data packet struct
      readSensors();

      // Send data to device
      sendData();
    } else {
      // reciever pair failed
      Serial.println("Reciever pair failed!");
    }
  }
  else {
    // No reciever found to process
  }
  timer=millis();
}
}


#ifdef USE_TRILL
void printTrillInfo() {
  int address = trillSensor.getAddress();

  Serial.println("Trill Device Details: ");
  Serial.print("\t- I2C address: ");
  Serial.print("#");
  Serial.print(address, HEX);
  Serial.print(" (");
  Serial.print(address);
  Serial.println(")");

  int deviceType = trillSensor.deviceType();
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
    int firmwareRev = trillSensor.firmwareVersion();
    Serial.print("\t- Firmware version: ");
    Serial.println(firmwareRev);

    int mode = trillSensor.getMode();
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
    if(trillSensor.is1D()) {
      Serial.println(1);
    } else if(trillSensor.is2D()) {
      Serial.println(2);
    } else {
      Serial.println(0);
    }

    int numChannels = trillSensor.getNumChannels();
    Serial.print("\t- Number of capacitive channels: ");
    Serial.println(numChannels);

    int numButtons = trillSensor.getNumButtons();
    Serial.print("\t- Number of button channels: ");
    Serial.println(numButtons);
}
#endif

