// Libraries for Driving Mecanum Mobile Robot Motors
#include "CustomMobileRobot.h"
#include "Arduino.h"                                // Main Arduino library

#include <utility/motor_pins.h>

uint8_t Motor1_ID = 1;
uint8_t Motor2_ID = 2;

short revs_desired;        
PWM_type speed_desired;    
uint8_t wheel_dir = 0;

debug_error_type error_returned;
// Array of Wheel objects
/*
MecanumEncoderWheel Wheels[2] = {
              MecanumEncoderWheel(Motor1_ID, motor1_pin_array),
              MecanumEncoderWheel(Motor2_ID, motor2_pin_array)};
              */
// Single object
MecanumEncoderWheel Wheel1(Motor1_ID, MOT1_IN1, MOT1_IN2, MOT1_EN, MOT1_ENCOD1, MOT1_ENCOD2);


void setup() {
  Serial.begin(9600);
  while (!Serial){}
  revs_desired  = 2;
  speed_desired = 50;
  Serial.println("rev:1 speed 50");
  Wheel1.runFor_FixedRevsSpeed(revs_desired, speed_desired, wheel_dir, &error_returned);
  
  revs_desired  = 2;
  speed_desired = 100;
  Serial.println("rev:2 speed 100");
  Wheel1.runFor_FixedRevsSpeed(revs_desired, speed_desired, wheel_dir, &error_returned);
}

void loop() {
  // put your main code here, to run repeatedly:

}
