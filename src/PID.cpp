#include "Arduino.h"
#include "PID.h"

double kp = 160.0;
double kd = 10.5;
double ki = 0.01;
double output_k = 1.0;
double currentPIDTime = 0.0;
double lastTime = 0.0;
double lastError = 0.0;
double eTime = 0.0;
double cum = 0.0;

double der = 0.0;
double prop = 0.0;
float baseLineAngle = 0.;

void PID_setProportional(double P){
    kp = P;
}

void PID_setIntegral(double I){
    ki = I;
}

void PID_setDerivate(double D){
    kd = D;
}


float PID_angle(float angle){
    float speed;
    float error = baseLineAngle - angle;
    lastTime = currentPIDTime;
    currentPIDTime= millis();
    eTime = currentPIDTime - lastTime;
    cum += error*eTime;
    der = ((error-lastError)/eTime);
    speed = (kp * error + kd * der + ki * cum)*output_k;
    lastError = error;

    if (abs(angle) < 0.1){speed = 0;}

    return speed;
}