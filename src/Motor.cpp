#include "Motor.h"

const int offsetA = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

void motor_init(){

}
void motor_break(){
    motor1.brake();
}

void motor_speed(int speed){
    motor1.drive(speed);    
}