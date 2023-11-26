#include <ESP32Encoder.h>

class Motor {
  private:
    float Ts = 0.0025;
    float Vm = 2 * 3.1415 * 6000 / 60;

    int ENC_A, ENC_B, PWM, IN_1, IN_2, ticks;
    float torqueTimer, tickTimer, tickRate, radToTick;
  
  public:
    ESP32Encoder encoder;

    Motor() {}

    void init(int ENC_A, int ENC_B, int PWM, int IN_1, int IN_2, int reduction) {
      ESP32Encoder::useInternalWeakPullResistors = UP;

      this->PWM = PWM;
      this->IN_1 = IN_1;
      this->IN_2 = IN_2;
      pinMode(PWM, OUTPUT);
      pinMode(IN_1, OUTPUT);
      pinMode(IN_2, OUTPUT);
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);

      encoder.attachFullQuad(ENC_A, ENC_B);
      encoder.setCount(0);
      
      radToTick = (2 * 3.1415 * reduction); // * (2 * 3.1415));
      Ts = Ts * reduction;
      Vm = Vm / reduction;

      tickTimer = millis();
      torqueTimer = millis();
    }

    void update() {
      tickRate = (encoder.getCount() - ticks) / (millis() - tickTimer) * 1000;
      ticks = encoder.getCount();
      tickTimer = millis();
    }

    /*void setTorque(float torque) {
      /*if (getVel() > 0) {
        torque += friction;
      } else if (getVel() < 0) {
        torque -= friction;
      }
      float torqueTerm = torque / Ts * 255;
      float velTerm = getVel() / Vm * 255;

      float changeTorqueTerm = 1 / 10 * (torque - old_torque) / (millis() - torqueTimer) * 255;
      old_torque = torque;
      torqueTimer = millis();

      Serial.println(torqueTerm);
      Serial.println(velTerm);
      Serial.println(changeTorqueTerm);
      Serial.println(" ");

      //pwm_old += torqueTerm + changeTorqueTerm;
      //pwm_old = constrain(pwm_old, -255, 255);
      setPWM(torqueTerm + velTerm); //+ changeTorqueTerm);
    }*/

    void setPWM(float voltage) {
      // int pwm = constrain(int(floatPWM), -255, 255);
      int pwm = constrain(int(255 * voltage), -255, 255);
      analogWrite(PWM, abs(pwm + 100));
      if (abs(pwm) < 1) {
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, LOW);
      } else if (pwm > 0) {
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      } else {
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }
    }

    float getPos() {
      return encoder.getCount() / radToTick;
    }

    float getVel() {
      return tickRate / radToTick;
    }

    void setRef() {
      encoder.clearCount();
      ticks = 0;
    }
};