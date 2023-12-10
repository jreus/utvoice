/*
Trill Test Utility for ESP Boards

This utility is intended for testing Trill CRAFT and FLEX boards.
You can have one of each, up to two boards, connected.

See: https://github.com/BelaPlatform/Trill-Arduino

*/

#include <Trill.h>

//#define CENTROID_MODE // Uncomment to read touch point data instead of raw differential data
//#define USE_CRAFT // Uncomment if you have a Trill Craft connected
#define USE_FLEX  // Uncomment if you have a Trill Flex connected

const int delaytime_ms = 200; // set sensor sampling delay time

#if defined(USE_CRAFT)
Trill trillCraft; // for Trill Craft
boolean craftTouchActive = false;
int craftRet;
#endif

#if defined(USE_FLEX)
Trill trillFlex; // for Trill Flex
boolean flexTouchActive = false;
int flexRet;
#endif

bool craftOK=false, flexOK=false;
Trill::Mode trillMode;

void setup() {

  Serial.begin(115200);
  delay(1200);

#if defined(CENTROID_MODE)
  trillMode=Trill::CENTROID;
#else
  trillMode=Trill::DIFF;
#endif


#if defined(USE_CRAFT)
  Serial.println("Attempting to discover and configure Trill Craft");
  craftRet = trillCraft.setup(Trill::TRILL_CRAFT);
  if(craftRet != 0) {
    Serial.print("failed to initialise trill craft, errorcode:"); Serial.println(craftRet);
    Serial.println(decodeTrillError(craftRet));
    craftOK=false;
  } else {
    Serial.println("Initialising trill craft");
    // TODO: Probably also a good idea to set prescaler & update baseline based on craft setup
    craftOK=true;
    delay(200);
    trillCraft.setMode(trillMode);
    printTrillInfo(trillCraft);
  }
#endif

#if defined(USE_FLEX)
  Serial.println("Attempting to discover and configure Trill Flex");
  flexRet = trillFlex.setup(Trill::TRILL_FLEX);
  if(flexRet != 0)
  {
    Serial.print("failed to initialise trill flex, errorcode:"); Serial.println(flexRet);
    Serial.println(decodeTrillError(flexRet));
    flexOK=false;
  } else {
    Serial.println("Initialising trill flex");
    delay(10);
    trillFlex.setPrescaler(4);
    delay(10);
    trillFlex.updateBaseline();
    delay(10);
    flexOK=true;
    delay(200);
    trillFlex.setMode(trillMode);
    printTrillInfo(trillFlex);
  }
#endif

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

void printTrillInfo(Trill & trill) {
  int address = trill.getAddress();

  Serial.println("Detected Trill Device: ");
  Serial.print("\t- I2C address: ");
  Serial.print("#");
  Serial.print(address, HEX);
  Serial.print(" (");
  Serial.print(address);
  Serial.println(")");

  int deviceType = trill.deviceType();
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
    int firmwareRev = trill.firmwareVersion();
    Serial.print("\t- Firmware version: ");
    Serial.println(firmwareRev);

    int mode = trill.getMode();
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
    if(trill.is1D()) {
      Serial.println(1);
    } else if(trill.is2D()) {
      Serial.println(2);
    } else {
      Serial.println(0);
    }

    int numChannels = trill.getNumChannels();
    Serial.print("\t- Number of capacitive channels: ");
    Serial.println(numChannels);

    int numButtons = trill.getNumButtons();
    Serial.print("\t- Number of button channels: ");
    Serial.println(numButtons);
}

void printLine(unsigned int n, char character = '_') {
  for(int i = 0; i < n; i++) {
    Serial.print(character);
  }
  Serial.println(".");
}

void printRawData(Trill & trill) {
  while(trill.rawDataAvailable() > 0) {
    int data = trill.rawDataRead();
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
}

void loop() {
  delay(delaytime_ms);

  #if defined(CENTROID_MODE)
  // Read touches...

#if defined(USE_CRAFT)
  if(craftOK) { // Read touch centroids...
    trillCraft.read();
    if(trillCraft.getNumTouches() > 0) {
      Serial.print("CRAFT: ");
      for(int i = 0; i < trillCraft.getNumTouches(); i++) {
        Serial.print(trillCraft.touchLocation(i));
        Serial.print(" ");
        Serial.print(trillCraft.touchSize(i));
        Serial.print(" ");
      }
      Serial.println("");
      craftTouchActive = true;
    } else if(craftTouchActive) {
      Serial.print("CRAFT: ");
      // Print a single line when touch goes off
      Serial.println("0 0");
      craftTouchActive = false;
    }
  }
#endif

#if defined(USE_FLEX)
  if(flexOK) { // Read touch centroids...
    trillFlex.read();
    if(trillFlex.getNumTouches() > 0) {
      Serial.print("FLEX: ");
      for(int i = 0; i < trillFlex.getNumTouches(); i++) {
        Serial.print(trillFlex.touchLocation(i));
        Serial.print(" ");
        Serial.print(trillFlex.touchSize(i));
        Serial.print(" ");
      }
      Serial.println("");
      flexTouchActive = true;
    } else if(flexTouchActive) {
      Serial.print("FLEX: ");
      // Print a single line when touch goes off
      Serial.println("0 0");
      flexTouchActive = false;
    }
  }
#endif

  #else
  // Read RAW diff data...

#if defined(USE_CRAFT)
  if(craftOK) {
    trillCraft.requestRawData();
    if(trillCraft.rawDataAvailable() > 0) {
      Serial.print("CRAFT  ");
      printRawData(trillCraft);
    }
  }
#endif

#if defined(USE_FLEX)
  if(flexOK) {
    trillFlex.requestRawData();
    if(trillFlex.rawDataAvailable() > 0) {
      Serial.print("FLEX    ");
      printRawData(trillFlex);
    }
  }
#endif

  // if(craftOK || flexOK) 
  //   printLine(156, '_');

  #endif


}



