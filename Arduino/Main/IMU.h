#include "SparkFunLSM6DSO.h"

class IMU {
  private:
    LSM6DSO myIMU;
    float pos;
    float posRef;
    float vel;
  
  public:
    IMU() {
      pos = 0;
      posRef = 0;
    }

    void init() {
      myIMU.begin();
      myIMU.initialize(BASIC_SETTINGS);
    }

    void update() {
      vel = myIMU.readFloatGyroZ();
      pos += vel * 50 / 1000;
    }

    long getPos() {
      return pos - posRef;
    }

    long getVel() {
      return vel;
    }

    void setRef() {
      posRef = pos;
    }
};