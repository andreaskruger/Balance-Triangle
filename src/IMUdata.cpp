#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <cmath>


// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);

//IMU 1
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro[3];          // Gyro daa x,y,z
float yaw,pitch,roll;

#define OUTPUT_READABLE_YAWPITCHROLL

#if i2CDEV_IMPLEMENTATION == I2C_ARDUINO_WIRE
    #include "Wire.h"
#endif

float IMU_angle_get_yaw(){
    return yaw;
}

float IMU_angle_get_pitch(){
    return pitch;
}

float IMU_angle_get_roll(){
    return roll;
}

void init_IMU(){
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(F("Initializing I2C device 1..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device 1 connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection IMU 1 successful") : F("MPU6050 connection IMU 1 failed"));
    Serial.println(F("Initializing DMP 1..."));

    devStatus = mpu.dmpInitialize();
    Serial.println("Dev status 1: "+devStatus);

    mpu.setXGyroOffset(85);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(-14);
    mpu.setZAccelOffset(1550); 

    mpu.CalibrateAccel(7);
    mpu.CalibrateGyro(7);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
}


void readBuffer(){
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize){fifoCount = mpu.getFIFOCount();}
    mpu.getFIFOBytes(fifoBuffer, packetSize);
}

void readIMU(){
    readBuffer();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/PI;
    roll = ypr[1] * 180/PI;
    pitch = ypr[2] * 180/PI;
}
