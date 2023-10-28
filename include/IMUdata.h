#ifndef IMUDATA_H
#define IMUDATA_H
#include <Arduino.h>

void init_IMU();

void readBuffer();

void readIMU();

float IMU_angle_get_yaw();

float IMU_angle_get_pitch();

float IMU_angle_get_roll();

#endif