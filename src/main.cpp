#include "Arduino.h"
#include "IMUdata.h"
#include "I2C_SCANNER.h"
#include "PID.h"
#include "Wire.h"
#include "Motor_control.h"

/*Defines*/
#define system_runtime 20

void setup(){
  Serial.begin(115200);
  while (!Serial){delay(10); }
  init_IMU();
  PID_setProportional(1);
  PID_setDerivate(1);
  PID_setIntegral(1);
}

void loop(){
  readIMU();
  float yaw = IMU_angle_get_yaw();
  float pitch = IMU_angle_get_pitch();
  float roll = IMU_angle_get_roll();
  float speed = PID_angle(yaw);
  //float speed = PID_angle(pitch);
  //float speed = PID_angle(roll);

  if (abs(speed)>10)
    motor_speed(speed);
  else
    motor_break();
  
}
  