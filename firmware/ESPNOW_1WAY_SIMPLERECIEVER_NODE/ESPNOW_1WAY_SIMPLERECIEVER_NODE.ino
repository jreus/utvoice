/**
   ESPNOW - GENERAL FIRMWARE FOR A SIMPLE 1WAY SENSOR NODE communicating with a COORDINATOR attached to a PC.

  For the basis of ESPNow Arduino programming see: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

  2023 (C) Jonathan Reus
  https://github.com/jreus
*/



// REPLACE WITH YOUR COORDINATOR MAC ADDRESS 
//uint8_t coordinator_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x64, 0x10}; // ESP32 v2 a
//uint8_t coordinator_mac[] = {0xE8, 0x9F, 0x6D, 0x2F, 0x48, 0x54}; // ESP32 v2 b
uint8_t coordinator_mac[] = {0xE8, 0x9F, 0x6D, 0x32, 0xFF, 0x50}; // ESP32 v2 c
//uint8_t coordinator_mac[] = {0x5C, 0xCF, 0x7F, 0xEF, 0xBD, 0x81}; // ESP8266 r

#define USE_TRILL       // uncomment if using a Trill sensor
#define TRILL_DEVICE_TYPE 1  // 0 for CRAFT, 1 for FLEX
#define TRILL_SENSOR_MODE 1    // 0 for DIFF, 1 for CENTROID
//#define USE_ADXL313 // using an ADXL313 3-axis accelerometer

#define ESPNOW_CHANNEL 11          // pick a channel (good to choose one not being used by ambient wifi)
#define ESPNOW_ENCRYPTION 0       // no encryption
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0



#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

#if defined(USE_ADXL313)
#include <Wire.h>
#include <SparkFunADXL313.h>
ADXL313 accel3x;
#endif

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

esp_now_peer_info_t coordinator;
//timer for transmission rate
unsigned long timer = 0;
//const int samplingPeriod_ms = 15;   // desired transmission period in milliseconds
const int samplingPeriod_ms = 2000;   // desired transmission period in milliseconds
const bool verbose = true;         // verbose serial output



// Sensor Output Data package, must match data package format in the Reciever/Coordinator firmware
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
sensor_data_package dataOUT;



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
  strcpy(dataOUT.msg, "ALLGOOD");
  //dataOUT.msg = "ALLGOOD";
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

  Serial.print("Send result: ");

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


// Check if the coordinator is already paired with this node.
// If not, make a pairing
bool manageCoordinator() {

  if (coordinator.channel == ESPNOW_CHANNEL) {
    if (DELETEBEFOREPAIR) deleteCoordinator();

    
    // check if the coordinator is registered as a peer
    bool exists = esp_now_is_peer_exist(coordinator.peer_addr);
    if ( exists) {
      if(verbose) {
        Serial.print("Coordinator Status: ");
        Serial.println("Already Paired");
      }
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






////////////////////////////////////////////////////////////////////////



void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("BEGIN ESPNOW SIMPLERECIEVER SENSOR NODE");
  Serial.print("dataOUT has length "); Serial.println(sizeof(dataOUT));

  setupSensors();

  // Setup ESPNOW...
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
  Serial.println("Register OnDataSent Callback");
  esp_now_register_send_cb(OnDataSent);

  // Register reciever as a peer
  memcpy(coordinator.peer_addr, coordinator_mac, 6);
  coordinator.channel = ESPNOW_CHANNEL;
  coordinator.encrypt = ESPNOW_ENCRYPTION;

  // Register coordinator as peer
  // Add peer (try this first to see if it works, if not, use the code block below)
  if (esp_now_add_peer(&coordinator) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // while( ! manageCoordinator() ) {
  //   delay(3000);
  //   Serial.println("Retry connection to coordinator...");
  // }

}

void setupSensors() {

  #if defined(USE_TRILL)
  
  int ret = trillSensor.setup(trillDevice);
  delay(50);

    #if TRILL_DEVICE_TYPE == 1
    // Trill Flex specific setup
    Serial.println("Initialising trill flex");
    delay(10);
    trillSensor.setPrescaler(4);
    delay(10);
    trillSensor.updateBaseline();
    delay(10);
    delay(200);
    #endif

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
}




void loop() {
//SENSOR SAMPLING

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
      Serial.println("No coordinator configuration available!");
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


#if TRILL_SENSOR_MODE == 0

void readRawTrillData(Trill & trill) {
  uint8_t i = 0;
  while(trill.rawDataAvailable() > 0) {
    dataOUT.sensorValues[i] = trill.rawDataRead();
    i++;
  }
}

#endif


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

