#include "Motor.h"
#include "IMU.h"

class Controller {
  private:
    // Physical Properties
    const long pi = 3.14159;
    const long r = 1.5 * 0.0254;
    
    // Controls Variables
    long delta = 0.1;
    long state[6];
    long target[6];
    long k[6] = {1, 1, 1, 1, 0, 0};
    long fallingBound = pi/3;
  
  public:
    // Objects
    Motor* LW = new Motor(39, 36, 21, 17, 16, 620); // Maybe inaccurate PPR
    Motor* RW = new Motor(39, 36, 21, 17, 16, 620);
    Motor* A2 = new Motor(39, 36, 21, 17, 16, 540);
    IMU imu;

    Controller() {
    }

    void init() {
      LW.init();
      RW.init();
      A2.init();
      imu.init();
    }

    void zeroState() {
      LW->setRef();
      RW->setRef();
      A2->setRef();
      imu.setRef();
      updateState();
    }

    void updateState() {
      state[0] = r * (imu.getPos() + (LW->getPos() + RW->getPos()) / 2);
      state[1] = r * (imu.getVel() + (LW->getVel() + RW->getVel()) / 2);
      state[2] = imu.getPos();
      state[3] = imu.getVel();
      state[4] = A2->getPos();
      state[5] = A2->getVel();
    }

    void controlActuators() {
      updateState();
      long TW = 0;
      long TA2 = 0;
      for (int i = 0; i < 6; i++) {
        TA2 += k[i] * (target[i] - state[i]);
      }
      LW->setTorque(TW);
      RW->setTorque(TW);
      A2->setTorque(TA2);
    }

    void stopActuators() {
      LW->setVoltage(0);
      RW->setVoltage(0);
      A2->setVoltage(0);
    }

    bool falling() {
      return (abs(state[2]) > fallingBound);
    }

    void setTarget(long newTarget[]) {
      for (int i = 0; i < 6; i++) {
        target[i] = newTarget[i];
      }
    }
};