#include <Arduino.h>
#include <IMU.h>
#include <Motor.h>
#include <PID.h>


void setup() {
  Serial.begin(115200);
  motor_init();
  IMU_init();
  Serial.println("Setup done...");
  delay(1000);
  Serial.println("Starting loop...");
}

void loop() {
  float angle = IMU_get_angle();
  motor_speed(pid_calculate_speed(angle));
  delay(10);
}




