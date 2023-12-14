#include <Arduino.h>
#include "Controller.h"

#define BTN 34
#define LED_PIN 13
#define POT 39

// State Machine
enum Case { IDLE, CALIBRATE, BALANCE };
Case c = IDLE;
volatile bool buttonIsPressed = false;
volatile bool buttonT = false;            // check timer interrupt 2
volatile bool deltaT = false;            // check timer interrupt 1 (for 10ms)
volatile bool deltaT2 = false;             // check timer interrupt 4 (for 12ms)
volatile bool calibrateT = false;            // check timer interrupt 3

// Controller   
Controller controller;
const float pi = 3.14159;

// Timers (uses three timers)
int mainTimer;
int buttonTimer;
int calibrateTimer;
int i = 0;

hw_timer_t* timer0 = NULL; // main timer (for balancing) - 10 milliseconds
hw_timer_t* timer1 = NULL; // button timer (in between button presses) - 1 second
hw_timer_t* timer2 = NULL; // calibrate timer (for determining calibration time) - 3 seconds 
hw_timer_t* timer3 = NULL; // balancing timer for delay - 12 milliseconds 
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux3 = portMUX_INITIALIZER_UNLOCKED;

/*hw_timer_t * imuTimer = NULL;
hw_timer_t * motorTimer = NULL;
// portMUX_TYPE imuTimerMux = portMUX_INITIALIZER_UNLOCKED;*/

//Initialization ------------------------------------
void IRAM_ATTR buttonISR() {
  buttonIsPressed = true;
}

/*void IRAM_ATTR imuISR() {
  portENTER_CRITICAL_ISR(&imuTimerMux);
  controller.imu.update();
  portEXIT_CRITICAL_ISR(&imuTimerMux);
}*/

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  deltaT = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  timerStop(timer1);
}

void IRAM_ATTR onTime2() {
  portENTER_CRITICAL_ISR(&timerMux2);
  calibrateT = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux2);
}

void IRAM_ATTR onTime3() {
  timerStop(timer3);
}

void setup() {
  Serial.begin(9600);
  pinMode(BTN, INPUT);
  pinMode(POT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(BTN, buttonISR, RISING);

  mainTimer = millis();
  buttonTimer = millis();

  /*imuTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(imuTimer, &imuISR, true);
  timerAlarmWrite(imuTimer, 50000, true); // 50000 ticks -> 50 ms
  timerAlarmEnable(imuTimer);*/

  controller.init();

  // initilize timers
  timer0 = timerBegin(0, 80, true);              // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true);  // edge (not level) triggered
  timerAlarmWrite(timer0, 10000, true);        // 10000 * 1 us = 10 ms, autoreload true

  timer1 = timerBegin(1, 80, true);              // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true);  // edge (not level) triggered
  timerAlarmWrite(timer1, 1000000, true);          // 1000000 * 1 us = 1 s, autoreload true

  timer2 = timerBegin(2, 80, true);              // timer 2, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer2, &onTime2, true);  // edge (not level) triggered
  timerAlarmWrite(timer2, 3000000, true);          // 3000000 * 1 us = 3 s, autoreload true

  timer3 = timerBegin(3, 80, true);              // timer 2, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer3, &onTime3, true);  // edge (not level) triggered
  timerAlarmWrite(timer3, 12000, true);          // 12000 * 1 us = 12 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0);  // enable
  timerAlarmEnable(timer1);  // enable
  timerAlarmEnable(timer2);  // enable
  timerAlarmEnable(timer3);  // enable
  timerStop(timer0);
  timerStop(timer1);
  timerStop(timer2);
  timerStop(timer3);
}

void loop() {
  controller.imu.update();

  switch (c) {
    case IDLE:
      ledOff(); 
      if (checkForButtonPress()) { // Check if button has been pressed
        timerStart(timer2); // Reset Calibrate timer
        c = CALIBRATE; // Change state to calibrate
      }

      controller.stopActuators();
      break;

    case CALIBRATE:
      if (checkForButtonPress()) { // Check if button has been pressed
        ledOff();
        controller.stopActuators();
        c = IDLE; // Change state to idle
      }
      timerStart(timer2);
      if (calibrateT) { // Check if 3 seconds has passed in calibrate mode (using timer 2) prev: (pastTime(calibrateTimer, 3000))
        portENTER_CRITICAL(&timerMux2);
        calibrateT = false; // reset calibrateT flag to false
        portEXIT_CRITICAL(&timerMux2);

        timerStop(timer2);
        ledOn();
        controller.zeroState(); 
        c = BALANCE; // Change state to balancing
      }

      flashLight(); // Blink the LED
      controller.stopActuators();
      
      break;

    case BALANCE:
      ledOn();

      if (checkForButtonPress() || controller.falling()) { // Check if button has been pressed, or if the bot has tilted too much
        ledOff();
        controller.stopActuators(); 
        c = IDLE; // Change state to idle (turn off actuators)
      }
      timerStart(timer0);
      if (!deltaT) {
        timerStart(timer3); 
        if(timerStarted(timer3)) {} //if this is triggered, don't do anything
        // mainTimer = millis();

        portENTER_CRITICAL(&timerMux3);
        deltaT2 = false; 
        portENTER_CRITICAL(&timerMux3);

        if (i <= 0) {
          controller.LW.update();
          controller.RW.update();
          controller.A2.update();
          i += 4;
        }
        float potVal = analogRead(POT);
        float targetPos = ((potVal/4095-0.5) / 5);
        // Serial.println(targetPos);
        float target[6] = {targetPos, 0, 0, 0, 0, 0};
        controller.setTarget(target); // Run actuators to stabilize at position & velocity targets
      }
      timerStop(timer0);

      portENTER_CRITICAL(&timerMux0);
      deltaT = false; 
      portENTER_CRITICAL(&timerMux0);

      break;

    
  }
}

bool checkForButtonPress() {
  if (timerStarted(timer1)) {
    buttonIsPressed = false;
  }
  if (buttonIsPressed) {
    buttonIsPressed = false;
    timerStart(timer1);
    return true;
  }
  return false;
}

bool pastTime(int timer, int time) {
  return millis() - timer > time;
}

void flashLight() {
  if ((millis() - timerReadMilis(timer2)) / 500 % 2) {
    ledOn();
  } else {
    ledOff();
  }
}

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
}