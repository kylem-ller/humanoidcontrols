#include "myLSM6DSO.h"
#include "Wire.h"

class IMU {
  private:
    volatile float pos;
    volatile float posRef;
    volatile float vel;
  
  public:
    LSM6DSO myIMU;
    
    IMU() {
      pos = 0;
      posRef = 0;
    }

    void init() {
      Wire.begin(23, 22);
      delay(10);
      myIMU.begin();
      myIMU.initialize(BASIC_SETTINGS);
    }

    void update() {
      vel = myIMU.readFloatGyroX();
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