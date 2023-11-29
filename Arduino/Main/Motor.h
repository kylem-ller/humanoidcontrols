#include <ESP32Encoder.h>

class Motor {
  private:
    int ENC_A, ENC_B, PWM, IN_1, IN_2, ticks;
    float tickTimer, tickRate, radToTick;
    
    ESP32Encoder encoder;
  
  public:
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

      tickTimer = millis();
    }

    void update() {
      tickRate = (encoder.getCount() - ticks) / (millis() - tickTimer) * 1000;
      ticks = encoder.getCount();
      tickTimer = millis();
    }

    void setPWM(float voltage, bool print) {
      // int pwm = constrain(int(floatPWM), -255, 255);
      int pwm = constrain(int(255 * voltage), -255, 255);
      int pwm2 = constrain(abs(pwm),0,255);
      if (print) {
        if (abs(pwm) < 1) {
          Serial.println(0);
        } else {
          Serial.println(pwm2);
        }
      }
      analogWrite(PWM, pwm2);
      if (abs(pwm) < 5) {
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