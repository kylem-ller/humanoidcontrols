#include "Motor.h"
#include "IMU.h"
#include "K.h"

class Controller {
  private:
    // Physical Properties
    const float pi = 3.14159;
    const float r = 1.5 * 0.0254;
    
    // Controls Variables
    const float delta = 0.1;
    float state[6];
    float target[6];
    const float fallingBound = pi/3;
  
  public:
    // Objects
    Motor LW;
    Motor RW;
    Motor A2;
    IMU imu;

    Controller() {}

    void init() {
      LW.init(14, 4, 21, 17, 16, 40);
      RW.init(15, 12, 33, 26, 25, 40);
      A2.init(19, 32, 27, 18, 5, 45);
      imu.init();
    }

    void zeroState() {
      LW.setRef();
      RW.setRef();
      A2.setRef();
      imu.setRef();
      updateState();
    }

    void stopActuators() {
      LW.setPWM(0);
      RW.setPWM(0);
      A2.setPWM(0);
    }
    
    void setTarget(float newTarget[]) {
      for (int i = 0; i < 6; i++) {
        target[i] = newTarget[i];
      }
      updateState();
      controlActuators();
    }

    void updateState() {
      state[0] = r * (imu.getPos() + (LW.getPos() + RW.getPos()) / 2);
      state[1] = r * (imu.getVel() + (LW.getVel() + RW.getVel()) / 2);
      state[2] = imu.getPos();
      state[3] = imu.getVel();
      state[4] = A2.getPos();
      state[5] = A2.getVel();
      controlActuators();
    }

    void controlActuators() {
      float TW = 0;
      float TA2 = 0;
      for (int i = 0; i < 6; i++) {
        TW += k[0][i] * (target[i] - state[i]) * r;
        TA2 += k[1][i] * (target[i] - state[i]) * r;
      }
      LW.setPWM(TW);
      RW.setPWM(TW);
      A2.setPWM(0);
    }

    bool falling() {
      return (abs(imu.getPos()) > fallingBound);
    }
};