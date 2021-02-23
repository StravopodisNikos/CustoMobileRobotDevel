/*
 *  Here Class MecanumEncoderWheel defined in CustomMobileRobot.h is tested
 */

// Libraries for Driving Mecanum Mobile Robot Motors
#include "CustomMobileRobot.h"
#include "Arduino.h"                                // Main Arduino library

uint8_t Motor1_ID = 1;
uint8_t Motor2_ID = 2;
double Kp = 0.01, Ki = 0.0001, Kd = 0.085;

// Single object of MecanumEncoderWheel Class, inherits from L298N,Encoder base classes
MobileWheel::MecanumEncoderWheel Wheel1(Motor1_ID,MOT1_EN,MOT1_IN1,MOT1_IN2,MOT1_ENCOD1,MOT1_ENCOD2,Kp,Ki,Kd,PID::Direct);

// Array of wheel objects
MobileWheel::MecanumEncoderWheel RobotWheels[2] = {
  MobileWheel::MecanumEncoderWheel(Motor1_ID,MOT1_EN,MOT1_IN1,MOT1_IN2,MOT1_ENCOD1,MOT1_ENCOD2,Kp,Ki,Kd,PID::Direct),
  MobileWheel::MecanumEncoderWheel(Motor2_ID,MOT2_EN,MOT2_IN1,MOT2_IN2,MOT2_ENCOD1,MOT2_ENCOD2,Kp,Ki,Kd,PID::Direct),
};

MecanumMobileRobot::CustomMobileRobot MECANUM_ROBOT;

// globals for simple demo
double desired_revs[] = {1.0, 1.0};
unsigned short desired_speed = 100;
MobileWheel::wheel_rot_dir desired_wheel_dir[] = {1, 0};
volatile bool KILL_MOTION_TRIGGERED = false;
debug_error_type wheel_error;
debug_error_type robot_error;

MobileWheel::wheel_motion_states WHEEL_STATE[2];

bool fn_state = false;
bool TERMINATE_MOTION = false;

MecanumMobileRobot::robot_dir desired_robot_dir = 5;// RIGHT

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  delay(2000);

/*
  fn_state = RobotWheels[0].runFor_FixedRevsPID(desired_revs, MOTOR_DIR, &KILL_MOTION_TRIGGERED, &wheel_error);
  if (fn_state)
  {
    Serial.println("PID SUCCESS");
  }
  else
  {
    Serial.println("PID FAILED");
    Serial.print("ERROR CODE= ");Serial.println(wheel_error);
  }
*/

// TEST STATE MACHINE WITHOUT CLASS
/*
//Initialize
RobotWheels[0].initialize_FixedRevsPID(desired_revs[0], desired_wheel_dir[0], &KILL_MOTION_TRIGGERED, &wheel_error);
RobotWheels[1].initialize_FixedRevsPID(desired_revs[1], desired_wheel_dir[1], &KILL_MOTION_TRIGGERED, &wheel_error);
// Start
RobotWheels[0].start_FixedRevsPID( &wheel_error);
RobotWheels[1].start_FixedRevsPID( &wheel_error);

// Update
do
{
  RobotWheels[0].update_FixedRevsPID(&KILL_MOTION_TRIGGERED, WHEEL_STATE, &wheel_error);
  RobotWheels[1].update_FixedRevsPID(&KILL_MOTION_TRIGGERED, (WHEEL_STATE+1), &wheel_error);

  Serial.print("WHEEL_STATE[0] ="); Serial.println(WHEEL_STATE[0]);
  Serial.print("WHEEL_STATE[1] ="); Serial.println(WHEEL_STATE[1]);
  // TERMINATE
  if( ( WHEEL_STATE[0] == MobileWheel::wheel_motion_states::success ) && ( WHEEL_STATE[1] == MobileWheel::wheel_motion_states::success ) )
  {
    TERMINATE_MOTION = true;
  }
}while(!TERMINATE_MOTION);
*/

// TEST STATE MACHINE USING CustomMobileRobot
MECANUM_ROBOT.setRobotDir(desired_robot_dir, desired_wheel_dir, &robot_error);
Serial.print("desired_wheel_dir[0] ="); Serial.println(desired_wheel_dir[0]);
Serial.print("desired_wheel_dir[1] ="); Serial.println(desired_wheel_dir[1]);
Serial.print("ROBOT ERROR          ="); Serial.println(robot_error);
delay(5000);

fn_state = MECANUM_ROBOT.driveFor_FixedRevsPID(RobotWheels, desired_revs, desired_wheel_dir, WHEEL_STATE, &KILL_MOTION_TRIGGERED, &robot_error);
if (fn_state)
{
  Serial.print("SUCCES - ROBOT ERROR ="); Serial.println(robot_error);
  Serial.print("WHEEL_STATE[0]       ="); Serial.println(WHEEL_STATE[0]);
  Serial.print("WHEEL_STATE[1]       ="); Serial.println(WHEEL_STATE[1]);
}
else
{
  Serial.print("FAILED - ROBOT ERROR          ="); Serial.println(robot_error);
}


} // end setup

void loop() 
{
  // put your main code here, to run repeatedly:

}
