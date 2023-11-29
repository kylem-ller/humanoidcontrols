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
    const float k2[2][6] = {{0, 2, 0, 0, 40, 0}, {0, 10, 4, 0, 40, 0.05}}; // Val 1 -> X Pos, Val 2 -> theta Pos, Val 4 -> X Vel, Val 5 -> Theta Vel
    const float k3[2][6] = {{20, 2, 0, 5, 40, 0}, {0, 0, 0, 0, 0, 0}};
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
      LW.setPWM(0, false);
      RW.setPWM(0, false);
      A2.setPWM(0, false);
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

      state[1] += H[2][1] * state[2];
      state[4] += H[5][4] * state[5];

      /*P[0] += state[0] / 1000 * (millis() - integralTimer);
      P[1] += state[1] / 1000 * (millis() - integralTimer);
      P[1] *= 0.8;
      if (abs(state[2] > pi/8)) {
        P[2] += 1.4 * state[2] / 1000 * (millis() - integralTimer);
      }
      P[2] *= 0.98;
      P[2] = constrain(P[2], -0.8, 0.8);
      integralTimer = millis();*/
    }

    void controlActuators() {
      float TW = 0;
      float TA2 = 0;
      for (int i = 0; i < 6; i++) {
        //TW += k2[0][i] * (target[i] - state[i]);
        //TA2 += k2[1][i] * (target[i] - state[i]);
        TW += K[i][0] * (target[i] - state[i]);
        TA2 += K[i][1] * (target[i] - state[i]);
      }
      //u = Km*(target - Hm*state);
      // TA2 -= P[2];
      // Serial.println(255 * P[2]);
      // Serial.println(255 * TA2);
      LW.setPWM(TW, false);
      RW.setPWM(TW, false);
      A2.setPWM(TA2, false);
      /*if (abs(state[2]) > pi/1.2 && state[2]*TA2 > 0) {
        A2.setPWM(0, true);
      } else {
        A2.setPWM(TA2, true); 
      }*/
    }

    bool falling() {
      return (abs(imu.getPos()) > fallingBound);
    }
};