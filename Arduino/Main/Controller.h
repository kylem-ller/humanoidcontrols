#include "Motor.h"
#include "IMU.h"

class Controller {
  private:
    // Physical Properties
    const float pi = 3.14159;
    const float r = 1.5 * 0.0254;
    
    // Controls Variables
    float delta = 0.1;
    float state[6];
    float target[6];
    float k[6] = {1, 1, 1, 1, 0, 0};
    float fallingBound = pi/3;
  
  public:
    // Objects
    Motor* LW = new Motor(4, 14, 21, 17, 16, 620); // Maybe inaccurate PPR
    Motor* RW = new Motor(15, 12, 33, 26, 25, 620);
    Motor* A2 = new Motor(19, 32, 27, 18, 5, 540);
    IMU imu;

    Controller() {
    }

    void init() {
      //LW->init();
      //RW->init();
      //A2->init();
      imu.init();
    }

    void zeroState() {
      //LW->setRef();
      //RW->setRef();
      //A2->setRef();
      imu.setRef();
      //updateState();
    }

    void updateState() {
      //noInterrupts();
      state[0] = r * (imu.getPos() + (LW->getPos() + RW->getPos()) / 2);
      state[1] = r * (imu.getVel() + (LW->getVel() + RW->getVel()) / 2);
      state[2] = imu.getPos();
      state[3] = imu.getVel();
      state[4] = A2->getPos();
      state[5] = A2->getVel();
      //interrupts();
    }

    void controlActuators() {
      updateState();
      LW->setVoltage(150);
      RW->setVoltage(150);
      /*updateState();
      float TW = 0;
      float TA2 = 0;
      for (int i = 0; i < 6; i++) {
        TA2 += k[i] * (target[i] - state[i]);
      }
      LW->setTorque(TW);
      RW->setTorque(TW);
      A2->setTorque(TA2);*/
    }

    void stopActuators() {
      LW->setVoltage(0);
      RW->setVoltage(0);
      A2->setVoltage(0);
    }

    bool falling() {
      return (abs(imu.getPos()) > fallingBound);
    }

    void setTarget(float newTarget[]) {
      for (int i = 0; i < 6; i++) {
        target[i] = newTarget[i];
      }
      controlActuators();
    }
};