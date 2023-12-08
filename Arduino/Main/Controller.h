#include "Motor.h"
#include "IMU.h"
#include "K.h"

class Controller {
  private:
    // Physical Properties
    const float pi = 3.14159;
    const float r = 0.04;
    
    // Controls Variables
    const float delta = 0.1;
    float state[6];
    float P[3] = {0, 0, 0};
    float target[6];
    const float fallingBound = 0.8*pi/2;
    float integralTimer;
  
  public:
    // Objects
    Motor LW;
    Motor RW;
    Motor A2;
    IMU imu;

    Controller() {}

    void init() {
      LW.init(14, 4, 21, 16, 17, 40);
      RW.init(12, 15, 33, 5, 18, 40);
      A2.init(19, 32, 27, 25, 26, 45);
      imu.init();
    }

    void zeroState() {
      LW.setRef();
      RW.setRef();
      A2.setRef();
      imu.setRef();
      updateState();
      integralTimer = millis();
    }

    void stopActuators() {
      LW.setPWM(0);
      RW.setPWM(0);
      A2.setPWM(0);
    }
    
    void setTarget(float newTarget[]) { //float newTarget[]) {
      for (int i = 0; i < 6; i++) {
        target[i] = newTarget[i];
      }
      updateState();
      controlActuators();
    }

    void updateState() {
      state[0] = r * (-imu.getPos() + (LW.getPos() + RW.getPos()) / 2);
      state[1] = imu.getPos();
      state[2] = A2.getPos();
      state[3] = r * (-imu.getVel() + (LW.getVel() + RW.getVel()) / 2);
      state[4] = imu.getVel();
      state[5] = A2.getVel();
    }

    void controlActuators() {
      float TW = 0;
      float TA2 = 0;
      for (int i = 0; i < 6; i++) {
        TW += K[i][0] * (target[i] - state[i]);
        TA2 += K[i][1] * (target[i] - state[i]);
      }
      Serial.println(" ");
      LW.setPWM(TW);
      RW.setPWM(TW);
      Serial.println(TW);
      if (TA2 < 0.01) {
        A2.setPWM(0);
      } else {
        A2.setPWM(6 * TA2 + 0.3 * TA2 / abs(TA2));
        Serial.println(8 * TA2 + 0.3 * TA2 / abs(TA2));
      }
    }

    bool falling() {
      return (abs(imu.getPos()) > fallingBound);
    }
};