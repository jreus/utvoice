/*
 

Trill Print Multiple
====================

This is a example of using multiple trill sensors with a single Arduino board.
The I2C pins of all the sensors are connected together (SCL with SCL, SDA with SDA).
Each sensor has a unique address. Here is the table of address for each sensor type,
the first column is the default address:

| Type:  | Addresses (trillHex)                              |
|--------|----------------------------------------------|
| BAR    | 0x20 0x21 0x22 0x23 0x24 0x25 0x26 0x27 0x28 |
| SQUARE | 0x28 0x29 0x2A 0x2B 0x2C 0x2D 0x2E 0x2F 0x30 |
| CRAFT  | 0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37 0x38 |
| RING   | 0x38 0x39 0x3A 0x3B 0x3C 0x3D 0x3E 0x3F 0x40 |
| HEX    | 0x40 0x41 0x42 0x43 0x44 0x45 0x46 0x47 0x48 |
| FLEX   | 0x48 0x49 0x4A 0x4B 0x4C 0x4D 0x4E 0x4F 0x50 |

In this example the sensor readings are printed to the console as raw values using the
`rawDataRead()` method.
*/

#include <Trill.h>

#define RAW_DATA // Uncomment to show raw data instead of touches

Trill trillCraft; // for Trill Craft
Trill trillFlex; // for Trill Flex
int craftRet, flexRet;

boolean craftTouchActive = false;
boolean flexTouchActive = false;
const int delaytime_ms = 500;

void setup() {

  Serial.begin(115200);

  craftRet = trillCraft.setup(Trill::TRILL_CRAFT);
  if(craftRet != 0) {
    Serial.println("failed to initialise trill craft");
  } else {
    Serial.println("Initialising trill craft");
  }

  flexRet = trillFlex.setup(Trill::TRILL_FLEX);
  if(flexRet != 0)
  {
    Serial.println("failed to initialise trill flex");
  } else {
    Serial.println("Initialising trill flex");
    delay(10);
    trillFlex.setPrescaler(4);
    delay(10);
    trillFlex.updateBaseline();
    delay(10);
  }

  #if defined(RAW_DATA)
  trillCraft.setMode(Trill::DIFF);
  trillFlex.setMode(Trill::DIFF);
  #else
  trillCraft.setMode(Trill::CENTROID);
  trillFlex.setMode(Trill::CENTROID);
  #endif

  if(craftRet != 0)
    printTrillInfo(trillCraft);
  if(flexRet != 0)
    printTrillInfo(trillFlex);

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

  Serial.println("");

  #if defined(RAW_MODE)
  if(craftRet != 0) {
  trillCraft.requestRawData();
  if(trillCraft.rawDataAvailable() > 0) {
    Serial.print("CRAFT  ");
    printRawData(trillCraft);
  }
  }

  if(flexRet != 0) {
  trillFlex.requestRawData();
  if(trillFlex.rawDataAvailable() > 0) {
    Serial.print("FLEX    ");
    printRawData(trillFlex);
  }
  }

  if(craftRet != 0 || flexRet != 0) 
    printLine(156, '_');
  #else
  if(craftRet != 0) {
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
  }
  else if(craftTouchActive) {
    Serial.print("CRAFT: ");
    // Print a single line when touch goes off
    Serial.println("0 0");
    craftTouchActive = false;
  }
  }

  if(flexRet != 0) {
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
  }
  else if(flexTouchActive) {
    Serial.print("FLEX: ");
    // Print a single line when touch goes off
    Serial.println("0 0");
    flexTouchActive = false;
  }
  }
  #endif


}



