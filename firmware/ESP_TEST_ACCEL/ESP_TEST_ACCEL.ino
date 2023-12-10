/*
Print out incoming Accelerometer data
*/

// See: https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library
// Click here to get the library: http://librarymanager/All#SparkFun_ADXL313
#define USE_ADXL313 // Uncomment if using ADXL313 low power 3-axis accelerometer

//#define USE_BNO08X // Uncomment if using BNO08X 9-DOF movement sensor (NOTE: support for BNO08X is not yet implemented)


#if defined(USE_ADXL313)
#include <Wire.h>
#include <SparkFunADXL313.h>
ADXL313 accel3x;
#endif

#if defined(USE_BNO08X)
#include <Adafruit_BNO08x.h>
// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t bnoSensorValue;
#endif

const int delaytime_ms = 50;


//=============================================

void setup() {

  Serial.begin(115200);
  delay(500);
  Wire.begin();
  delay(500);

#if defined(USE_ADXL313)
  if (accel3x.begin() == false) //Begin communication over I2C
  {
    Serial.println("The ADXL313 sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  Serial.print("The ADXL313 Sensor was detected and is connected properly.");
  
  accel3x.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode
#endif

#if defined(USE_BNO08X)
  Serial.print("BNO08X enabled in sketch. This doesn't mean it was discovered.");
#endif

}

//===========================

void loop() {

#if defined(USE_ADXL313)
  if(accel3x.dataReady()) // check data ready interrupt, note, this clears all other int bits in INT_SOURCE reg
  {
    accel3x.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
    Serial.print("x: ");
    Serial.print(accel3x.x);
    Serial.print("\ty: ");
    Serial.print(accel3x.y);
    Serial.print("\tz: ");
    Serial.print(accel3x.z);
    Serial.println();
  }
#endif

  delay(delaytime_ms);
}



