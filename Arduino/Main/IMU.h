#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "Kalman.h"

class IMU {
  private:
    float accY, accZ;
    float gyroX, gyroXangle;
    float angRef;
    float timer;

    Kalman kalman;
  
  public:
    LSM6DSO myIMU;

    IMU() {
    }

    void init() {
      Wire.begin(23, 22);
      delay(10);
      myIMU.begin();
      myIMU.initialize(BASIC_SETTINGS);
      setRef();
    }

    void update() {
      accY = myIMU.readFloatAccelY();
      accZ = myIMU.readFloatAccelZ();
      gyroX = myIMU.readFloatGyroX() / 131;
      
      float dt = (millis() - timer) / 1000;
      timer = millis();

      double roll  = atan2(accY, accZ) * RAD_TO_DEG;

      if ((roll < -90 && gyroXangle > 90) || (roll > 90 && gyroXangle < -90)) {
        kalman.setAngle(roll);
        gyroXangle = roll;
      } else
        gyroXangle = kalman.getAngle(roll, gyroX, dt);
    }

    float getPos() {
      return (gyroXangle - angRef) * DEG_TO_RAD;
    }

    float getVel() {
      return gyroX  * DEG_TO_RAD;
    }

    void setRef() {
      accY = myIMU.readFloatAccelY();
      accZ = myIMU.readFloatAccelZ();

      double roll  = atan2(accY, accZ) * RAD_TO_DEG;
      kalman.setAngle(roll);

      gyroXangle = roll;
      angRef = gyroXangle;

      timer = millis();
    }
};