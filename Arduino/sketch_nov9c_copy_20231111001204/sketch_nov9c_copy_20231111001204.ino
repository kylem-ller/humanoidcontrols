#include <Arduino.h>
#include <arduino-timer.h>
#include "IMU.h"
#include "SimplyAtomic.h"
#include "Kalman.h"
// #include "myLSM6DSO.h"

IMU imu;
//auto imuTimer = timer_create_default();
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float gyroYangle;
float gyroXangle;
float timer;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

void setup() {
  Serial.begin(115200);
  imu.init();
  //attachInterrupt(digitalPinToInterrupt(22),updateIMU,RISING);
  //imuTimer.every(100, updateIMU);
  //Wire.onRequest(updateIMU);
  
  accX = imu.myIMU.readFloatAccelX();
  accY = imu.myIMU.readFloatAccelY();
  accZ = imu.myIMU.readFloatAccelZ();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);

  accX = imu.myIMU.readFloatAccelX();
  accY = imu.myIMU.readFloatAccelY();
  accZ = imu.myIMU.readFloatAccelZ();
  gyroX = imu.myIMU.readFloatGyroX() / 131;
  gyroY = imu.myIMU.readFloatGyroY() / 131;
  gyroZ = imu.myIMU.readFloatGyroZ() / 131;
  
  float dt = (millis() - timer) / 1000;
  timer = millis();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  if ((pitch < -90 && gyroYangle > 90) || (pitch > 90 && gyroXangle < -90)) {
    kalmanY.setAngle(pitch);
    gyroYangle = pitch;
  } else
    gyroYangle = kalmanY.getAngle(pitch, gyroY, dt); // Calculate the angle using a Kalman filter

  if (abs(gyroYangle) > 90)
    gyroX = -gyroX; // Invert rate, so it fits the restriced accelerometer reading
  gyroXangle = kalmanX.getAngle(roll, gyroX, dt);  

  Serial.println(gyroX, 3);
  Serial.println(gyroXangle, 3);
  
}

void updateIMU() {
  imu.update();
}


/* bool updateIMU(void *) {
  static bool IAmRunning = false;
  if(IAmRunning) return false;
  IAmRunning = true;
  interrupts();
  imu.update();
  return true;
}*/

/*#include <Wire.h>

#define I2C_SDA 23
#define I2C_SCL 22
 
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}*/