/**
   ESPNOW - GENERAL FIRMWARE FOR A 1WAY SENSOR/ACTUATOR NODE
   This firmware sketch is meant as a starting point for creating custom sensor or actuator nodes.
   Depending on configuration of preprocessor flags:
    * this firmware sends sensor data to a coordinator (NODE_MODE=0) 
    * or recieves actuator signals from a coordinator (NODE_MODE=1)

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/

#define NODE_MODE 0 // 0 == SENSOR (SENDER) NODE
//#define NODE_MODE 1 // 1 == ACTUATOR (RECIEVER) NODE
//#define COORDINATOR_AP // uncomment if coordinator is running in AP-mode and broadcasting a network ssid

// Enable sensors
//#define USE_ADXL313 // using an ADXL313 3-axis accelerometer

// Enable actuators
//#define USE_SERVO180 // use a standard servo/linear servo with operational values in degrees from 0-180

// Set coordinator SSID prefix, or coordinator MAC address
#if defined(COORDINATOR_AP)
const char* coordinator_ap_ssid_prefix = "rcv_"; // Set this to your AP coordinator's SSID prefix
#else
// Coordinator is STA mode. Set this to your coordinator's MAC address so that this node will be able to find it.
uint8_t coordinator_sta_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // a
//uint8_t coordinator_sta_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x55}; // b
#endif

#if defined(ESP32)
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#endif
#if defined(ESP8266)
// TODO: ESP8266 compatible version of this firmware
#endif

#if defined(USE_ADXL313)
#include <Wire.h>
#include <SparkFunADXL313.h>
ADXL313 accel3x;
#endif


// ENABLE & CONFIGURE TRILL SENSOR
#define USE_TRILL       // uncomment if using a Trill sensor
#define TRILL_DEVICE_TYPE 1  // 0 for CRAFT, 1 for FLEX
#define TRILL_SENSOR_MODE 1    // 0 for DIFF, 1 for CENTROID

#if defined(USE_TRILL)
#include <Trill.h>
Trill trillSensor;
boolean trillTouchActive = false;
bool trillOK = false;
#if TRILL_DEVICE_TYPE == 0
const Trill::Device trillDevice = Trill::TRILL_CRAFT;
#else
const Trill::Device trillDevice = Trill::TRILL_FLEX;
#endif
#if TRILL_SENSOR_MODE == 0
const Trill::Mode trillMode = Trill::DIFF;
#else
const Trill::Mode trillMode = Trill::CENTROID;
#endif
#endif


// Data package, must match data package format in Reciever firmware
// ESPNow spec limits this to around 250 bytes
typedef struct outgoing_data_package {
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
} outgoing_data_package;
outgoing_data_package dataOUT;

// Configure ESPNOW
#define ESPNOW_CHANNEL 1          // pick a channel (good to choose one not being used by ambient wifi)
#define ESPNOW_ENCRYPTION 0       // no encryption
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
esp_now_peer_info_t coordinator;

//timer for transmission rate
unsigned long timer = 0;
const int samplingPeriod_ms = 15;   // desired transmission period in milliseconds
const bool verbose = false;         // verbose serial output


/////////////////////////////////////////////////////////////////////////////////

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


#if defined(COORDINATOR_AP)

// Scan for coordinator AP network with SSID including coordinator_ap_ssid_prefix
void ScanForCoordinator() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, ESPNOW_CHANNEL); // Scan for networks on ESPNOW_CHANNEL
  // reset on each scan
  bool coordinatorFound = 0;
  memset(&coordinator, 0, sizeof(coordinator));
  Serial.println("Scanning for coordinator AP networks on channel "); Serial.println(ESPNOW_CHANNEL);
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found on channel");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices on channel");
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
      if (SSID.indexOf(coordinator_ap_ssid_prefix) == 0) {
        // SSID of interest
        Serial.println("Found the Coordinator.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Reciever
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            coordinator.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        coordinator.channel = ESPNOW_CHANNEL;
        coordinator.encrypt = ESPNOW_ENCRYPTION;

        coordinatorFound = 1;
        // Break after we find one matching AP network / coordinator
        break;
      }
    }
  }

  if (coordinatorFound) {
    Serial.println("Coordinator Found, processing..");
  } else {
    Serial.println("Coordinator Not Found, trying again.");
  }

  WiFi.scanDelete(); // clean up memory
}
#endif

// Check if the coordinator is already paired with this node.
// If not, make a pairing
bool manageCoordinator() {

  if (coordinator.channel == ESPNOW_CHANNEL) {
    if (DELETEBEFOREPAIR) deleteCoordinator();

    if(verbose) Serial.print("Reciever Status: ");
    
    // check if the coordinator is registered as a peer
    bool exists = esp_now_is_peer_exist(coordinator.peer_addr);
    if ( exists) {
      if(verbose) Serial.println("Already Paired");
      return true;
    } else {
      // attempt pairing with coordinator
      esp_err_t addStatus = esp_now_add_peer(&coordinator);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Coordinator added to peer list");
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
    }
  } else {
    if(verbose) Serial.println("No Reciever found to process");
    return false;
  }
}

void deleteCoordinator() {
  esp_err_t delStatus = esp_now_del_peer(coordinator.peer_addr);
  Serial.print("Coordinator Delete Status: ");
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
// send data to coordinator
void sendData() {
  const uint8_t *peer_addr = coordinator.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &dataOUT, sizeof(dataOUT));

  #if defined(USE_TRILL)

  #if TRILL_SENSOR_MODE == 0
  // Trill RAW/DIFF data
  for(uint8_t i=0; i < dataOUT.numSensors; i++) {
    int data = dataOUT.sensorValues[i];
    if(data < 1000)
      Serial.print(0);
    if(data < 100)
      Serial.print(0);
    if(data < 10)
      Serial.print(0);
    Serial.print(data);
    Serial.print(" ");
  }
  Serial.println("");  
  #else
  // Trill TOUCH/CENTROID data
  Serial.print(dataOUT.numTouches); Serial.print(" touches: ");
  for(int i=0; i < dataOUT.numTouches; i++) {
    Serial.print(dataOUT.touchLocations[i]); Serial.print(" ");
    Serial.print(dataOUT.touchSizes[i]); Serial.print("  ");
  }
  Serial.println();
  #endif

  #endif

  #if defined(USE_ADXL313)
  // ADXL x/y/z values
  Serial.print("x:"); Serial.print(dataOUT.ax);
  Serial.print("y:"); Serial.print(dataOUT.ay);
  Serial.print("z:"); Serial.print(dataOUT.az);
  Serial.println();
  #endif

  Serial.print("Send Status: ");

  if (result == ESP_OK) {
    if(verbose) Serial.println("Success");
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

// read sensors & load readings into dataOUT
void readSensors() {
  delay(50); // delay for stability
  
  #if defined(USE_TRILL)
  
  #if TRILL_SENSOR_MODE == 0
  if(trillOK) {
    trillSensor.requestRawData();
    if(trillSensor.rawDataAvailable() > 0) {
      readRawTrillData(trillSensor);
    }
  }
  #else
  // Read TOUCH/CENTROID data
  trillSensor.read();
  dataOUT.numTouches = trillSensor.getNumTouches();
  if(dataOUT.numTouches > 0) {
    for(uint8_t i = 0; i < dataOUT.numTouches; i++) {
      dataOUT.touchLocations[i] = trillSensor.touchLocation(i);
      dataOUT.touchSizes[i] = trillSensor.touchSize(i);
    }
    dataOUT.touchActive = true;
  }
  else if(dataOUT.touchActive) {
    // One reading with no touches, set touchActive to 0
    dataOUT.touchActive = false;
  }
  #endif

  #endif

  #if defined(USE_ADXL313)
  if(accel3x.dataReady()) // check data ready interrupt, note, this clears all other int bits in INT_SOURCE reg
  {
    accel3x.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
    dataOUT.ax = accel3x.x;
    dataOUT.ay = accel3x.y;
    dataOUT.az = accel3x.z;
  }
  #endif

  // Set sensor read info message, can be 7 characters max
  dataOUT.msg = "ALLGOOD";
  

}

// callback when data is sent to coordinator
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
  #if defined(USE_TRILL)
  
  int ret = trillSensor.setup(trillDevice);
  delay(50);
  trillSensor.setMode(trillMode);
  if(ret != 0) {
    Serial.println("failed to initialise trillSensor");
    Serial.print("Error code: ");
    Serial.println(ret);
    trillOK=false;
  } else {
    Serial.println("Trill was detected!");
    trillOK=true;
    printTrillInfo();
  }

  #endif


  #if defined(USE_ADXL313)
  
  #if !defined(USE_TRILL)
  Wire.begin(); // this gets called in trillSensor.setup()
  #endif
  
  if (accel3x.begin() == false) //Begin communication over I2C
  {
    Serial.println("The ADXL313 sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  Serial.print("The ADXL313 Sensor was detected and is connected properly.");  
  accel3x.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode

  #endif


  // Setup ESPNow...
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("Configuring ESPNow Node");
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());

  // Init ESPNow with a fallback logic
  Serial.println("Initializing ESPNow");
  InitESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register reciever as a peer
  #ifdef COORDINATOR_AP
  ScanForCoordinator();
  #else
  memcpy(coordinator.peer_addr, coordinator_mac, 6);
  reciever.channel = ESPNOW_CHANNEL;
  reciever.encrypt = ESPNOW_ENCRYPTION;
  #endif

  // Register coordinator as peer
  while( ! manageCoordinator() ) {
    delay(3000);
    Serial.println("Retry connection to coordinator...");
  }

}

void loop() {
  
  if((millis() - timer) > samplingPeriod_ms) { // transfer rate timer
    if (coordinator.channel == ESPNOW_CHANNEL) { // check if reciever channel is defined
      // Add coordinator as a peer if it has not been added already
      bool isPaired = manageCoordinator();
      if (isPaired) {        
        readSensors();
        sendData();
      } else {
        Serial.println("Unable to pair with coordinator!");
      }
    }
    else {
      Serial.println("No coordinator configuration available!")
    }
    timer=millis();
  }

}


#if defined(USE_TRILL)

/////// TRILL HELPER FUNCTIONS ///////
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

void readRawTrillData(Trill & trill) {
  uint8_t i = 0;
  while(trill.rawDataAvailable() > 0) {
    dataOUT.sensorValues[i] = trill.rawDataRead();
    i++;
  }
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
#endif

