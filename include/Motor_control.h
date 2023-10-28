#ifndef MOTOR_H
#define MOTOR_H

//#include <SparkFun_TB6612.h>


#define AIN1 33
#define AIN2 25
#define PWMA 32
#define STBY 4

void motor_init();

void motor_break();

void motor_speed(int);
#endif