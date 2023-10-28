#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define gscale ((250./32768.0)*(PI/180.0))
float Kp = 30.0;
float Ki = 0.0;

float q[4] = {1.0, 0.0, 0.0, 0.0};
float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest

float Axyz_cal[3] = {0.0, 0.0, 0.0};
float Gxyz_cal[3] = {0.0, 0.0, 0.0};

float Axyz[3];
float Gxyz[3];
float ypr[3];

int calibration_nr = 1000;

unsigned long print_ms = 200; //print angles every "print_ms" milliseconds
unsigned long current_ms, last_ms = 0;


void IMU_init(){
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  Serial.println("Starting calibration...");
  for(int j = 0; j<calibration_nr; j++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Axyz_cal[0] += a.acceleration.x;
    Axyz_cal[1] += a.acceleration.y;
    Axyz_cal[2] += a.acceleration.z;

    Gxyz_cal[0] += g.gyro.x;
    Gxyz_cal[1] += g.gyro.y;
    Gxyz_cal[2] += g.gyro.z;

  }
  Axyz_cal[0] = Axyz_cal[0]/calibration_nr;
  Axyz_cal[1] = Axyz_cal[1]/calibration_nr;
  Axyz_cal[2] = Axyz_cal[2]/calibration_nr;

  Gxyz_cal[0] = Gxyz_cal[0]/calibration_nr;
  Gxyz_cal[1] = Gxyz_cal[0]/calibration_nr;
  Gxyz_cal[2] = Gxyz_cal[0]/calibration_nr;

  Serial.println("Calibration values for accelerometer:");
  Serial.print("Acc x: ");
  Serial.println(Axyz_cal[0]);
  Serial.print("Acc y: ");
  Serial.println(Axyz_cal[1]);
  Serial.print("Acc z: ");
  Serial.println(Axyz_cal[2]);

  Serial.println("Calibration values for gyroscope:");
  Serial.print("Gyro x: ");
  Serial.println(Gxyz_cal[0]);
  Serial.print("Gyro y: ");
  Serial.println(Gxyz_cal[1]);
  Serial.print("Gyro z: ");
  Serial.println(Gxyz_cal[2]);
  A_cal[0] = Axyz_cal[0]; A_cal[1] = Axyz_cal[1]; A_cal[2] = Axyz_cal[2];
  G_off[0] = Gxyz_cal[0]; G_off[1] = Gxyz_cal[1]; G_off[2] = Gxyz_cal[2];
}

float IMU_get_angle(){
    int i = 0;
    float dt = 0;
    unsigned long current = 0, last = 0;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float Axyz[3];
    float Gxyz[3];
    Axyz[0] = (float) a.acceleration.x;
    Axyz[1] = (float) a.acceleration.y;
    Axyz[2] = (float) a.acceleration.z;
    for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float) g.gyro.x - G_off[0]) * gscale;
    Gxyz[1] = ((float) g.gyro.y - G_off[1]) * gscale;
    Gxyz[2] = ((float) g.gyro.z - G_off[2]) * gscale;

    current = micros();
    dt = (current - last) * 1.0e-6;
    last = current;

    // Calculate angles
    
    // to degrees
    ypr[0]   *= 180.0 / PI;
    if (ypr[0] < 0) ypr[0] += 360.0; //compass circle
    ypr[1] *= 180.0 / PI;
    ypr[2] *= 180.0 / PI;

    current_ms = millis(); //time to print?
    if (current_ms - last_ms >= print_ms) {
        last_ms = current_ms;
        Serial.print("Yaw: ");
        Serial.print(ypr[0], 0);
        Serial.print(" || Pitch: ");
        Serial.print(ypr[1], 0);
        Serial.print(" || Roll: ");
        Serial.println(ypr[2], 0);
    }


    float angle;
    return angle;
}