/*
REQUIREMENT: ESP32Servo library by Kevin Harrington


 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 the ground wire is typically black or brown, and the signal wire is typically yellow,
 orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
considerable power, it's good to connect the servo power either to the direct 5V USB
  signal, or to a separate power supply.
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32

This example assumes using a ACTUONIX L12-50-100-6-R micro linear actuator
See: https://www.actuonix.com/datasheets
According to the datasheet: A 1.0 ms pulse commands the controller to fully retract the actuator, 
and a 2.0 ms pulse signals it to fully extend. So the default values of 1000 and 2000 are fine.

If the motion of the actuator, or of other servos in your system, seems erratic, 
place a 1–4Ω resistor in series with the actuator’s red V+ lead wire.

 */

#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
const byte SERVO_PIN = 14;
const byte DELAY_ms = 250;
const byte INC_steps = 10;

void setup() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  // Serial.begin(115200);
  // Serial.println("Begin Servo Test");
}

void loop() {

	for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(20);             // waits for the servo to reach the position
	}
  delay(1000);
	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		myservo.write(pos);    // tell servo to go to position in variable 'pos'
		delay(20);             // waits for the servo to reach the position
	}
  delay(1000);
}

