#include <Trill.h>

Trill trillSensor;
boolean touchActive = false;

// Uncomment the line matching the type of Trill device you are using - CRAFT or FLEX
const Trill::Device TRILL_DEVICE = Trill::TRILL_CRAFT;
//const Trill::Device TRILL_DEVICE = Trill::TRILL_FLEX;


void setup() {
  // Initialise serial and touch sensor
  Serial.begin(115200);
  delay(500);
  int ret = trillSensor.setup(TRILL_DEVICE);
  delay(500);
  trillSensor.setMode(Trill::CENTROID); // There are four mode options CENTROID, DIFF, RAW and BASELINE
  
  if(ret != 0) {
    Serial.println("failed to initialise trillSensor");
    Serial.print("Error code: ");
    Serial.println(ret);
  } else {
    Serial.println("Trill was detected!");
  }

  printTrillInfo();
}

void loop() {
  // Read 20 times per second
  delay(50);
  trillSensor.read();

  if(trillSensor.getNumTouches() > 0) {
    for(int i = 0; i < trillSensor.getNumTouches(); i++) {
        Serial.print(trillSensor.touchLocation(i));
        Serial.print(" ");
        Serial.print(trillSensor.touchSize(i));
        Serial.print(" ");
    }
    Serial.println("");
    touchActive = true;
  }
  else if(touchActive) {
    // Print a single line when touch goes off
    Serial.println("0 0");
    touchActive = false;
  }
}


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