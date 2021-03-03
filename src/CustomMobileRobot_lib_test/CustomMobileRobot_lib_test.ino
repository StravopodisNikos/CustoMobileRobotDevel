/*
 *  Here Class MecanumEncoderWheel defined in CustomMobileRobot.h is tested
 */

// Libraries for Driving Mecanum Mobile Robot Motors
#include "CustomMobileRobot.h"
#include "Arduino.h"                                // Main Arduino library
#include <Wire.h>

/* 
 *  PID controller gains -> Must be tuned when system is complete
 *  Here the same gains are given to each wheel. This will change!
 */
uint8_t Motor1_ID = 1;
uint8_t Motor2_ID = 2;
double Kp = 0.01, Ki = 0.0001, Kd = 0.085;

/*
 * Define the Objects used
 */

// WHEELS
// Single object of MecanumEncoderWheel Class, inherits from L298N,Encoder base classes
MobileWheel::MecanumEncoderWheel Wheel1(Motor1_ID,MOT1_EN,MOT1_IN1,MOT1_IN2,MOT1_ENCOD1,MOT1_ENCOD2,Kp,Ki,Kd,PID::Direct);

// Array of wheel objects
MobileWheel::MecanumEncoderWheel RobotWheels[2] = {
  MobileWheel::MecanumEncoderWheel(Motor1_ID,MOT1_EN,MOT1_IN1,MOT1_IN2,MOT1_ENCOD1,MOT1_ENCOD2,Kp,Ki,Kd,PID::Direct),
  MobileWheel::MecanumEncoderWheel(Motor2_ID,MOT2_EN,MOT2_IN1,MOT2_IN2,MOT2_ENCOD1,MOT2_ENCOD2,Kp,Ki,Kd,PID::Direct),
};

// CHASSIS
// Single object of CustomMobileRobot class
MecanumMobileRobot::CustomMobileRobot MECANUM_ROBOT, *PTR2ROBOT;
/* HERE SET DESIRED ROBOT DIR FOR DEMO */
MecanumMobileRobot::robot_dir desired_robot_dir = MecanumMobileRobot::robot_dir::FWD;  

// SONAR SENSOR
NewPing MOBILE_SONAR1(SONAR_TRIG, SONAR_ECHO, SONAR_MAX_DISTANCE_CM), *PTR2SONAR;

// IMU
MPU6050 MOBILE_MPU, * PTR2MPU;

/*
 * ADAFRUIT IMU sensor 
 */
sensors::imu9dof SingleIMUSensor(GYRO_RANGE_2000DPS, ACCEL_RANGE_8G, FILTER_UPDATE_RATE_HZ, 0x0021002C, 0x8700A, 0x8700B), *IMUSensor;
Adafruit_FXAS21002C gyro;
Adafruit_FXOS8700 accelmag;
SF fusion;
sensors_event_t gyro_event, accel_event, magnet_event;
float pitch, roll, yaw;
sensors::imu_packet IMU_DATA_PKG, *PTR_2_imu_packet;
sensors::imu_sensor_states ImuCurrentState;


/*
 *  Vars that control sensor feedback events
 */
bool TERMINATE_MOTION = false;                // emergency

/*
 * Vars for debugging and system monitor
 */
bool fn_state = false;
debug_error_type wheel_error;
debug_error_type robot_error;

/* 
 *  Set globals for simple demo
 */
double desired_revs[] = {1.0, 1.0};
unsigned short desired_speed = 100;
MobileWheel::wheel_rot_dir desired_wheel_dir[] = {1, 0};
volatile bool KILL_MOTION_TRIGGERED = false;
MobileWheel::wheel_motion_states WHEEL_STATE[2];
float yaw_goal = 90.000;
float current_yaw = 0.00;
float timeStep = 0.01;
// SONAR DEMO
unsigned long DESIRED_DIST_UNTIL_NEST = 5; // [cm]

void setup() 
{
  // INITIALIZE SERIAL COMMUNICATION -> WILL BE REMOVED - ONLY FOR DEBUG
  Serial.begin(115200);
  
  while(!Serial);
  // INITIALIZE INTEGRATED IMU SENSOR
  /*
  while(!MOBILE_MPU.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  MOBILE_MPU.calibrateGyro();
  MOBILE_MPU.setThreshold(3);
    Vector norm = MOBILE_MPU.readNormalizeGyro();
  current_yaw = current_yaw + norm.ZAxis * timeStep;
    Serial.print(" CURRENT YAW = "); Serial.println(current_yaw);
    */
  // START SETUP
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
/*
MECANUM_ROBOT.setRobotDir(desired_robot_dir, desired_wheel_dir, &robot_error);
Serial.print("desired_wheel_dir[0] ="); Serial.println(desired_wheel_dir[0]);
Serial.print("desired_wheel_dir[1] ="); Serial.println(desired_wheel_dir[1]);
Serial.print("ROBOT ERROR          ="); Serial.println(robot_error);
delay(1000);

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
*/

/* TESTING REAL-TIME MPU
MECANUM_ROBOT.initializeMPU(PTR2MPU, &robot_error);

fn_state = MECANUM_ROBOT.rotateFor_FixedYawPID(PTR2ROBOT, PTR2MPU, RobotWheels, yaw_goal, desired_wheel_dir, WHEEL_STATE, &KILL_MOTION_TRIGGERED, &robot_error);
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
*/

/* TESTING REAL-TIME ADAFRUIT FUNCTIONS
*/

/* TESTING REAL-TIME SONAR FUNCTIONS */

MECANUM_ROBOT.setRobotDir(desired_robot_dir, desired_wheel_dir, &robot_error);
Serial.print("desired_wheel_dir[0] ="); Serial.println(desired_wheel_dir[0]);
Serial.print("desired_wheel_dir[1] ="); Serial.println(desired_wheel_dir[1]);
Serial.print("ROBOT ERROR          ="); Serial.println(robot_error);
delay(1000);

MECANUM_ROBOT.driveUntil_FixedDistPID(PTR2SONAR ,DESIRED_DIST_UNTIL_NEST, RobotWheels, desired_wheel_dir, WHEEL_STATE, &KILL_MOTION_TRIGGERED, &robot_error);
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

delay(2000);
/* TESTING NEST CORNERING */
MECANUM_ROBOT.attachNestLeft(RobotWheels, &robot_error);
Serial.print("DEBUG CODE RETURNED ="); Serial.println(robot_error);



} // end setup

void loop() 
{

}
