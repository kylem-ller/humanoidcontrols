#include <SimplyAtomic.h>

#define ENCA_LEFT_WHEEL 39
#define ENCB_LEFT_WHEEL 36
#define PWM_LEFT_WHEEL 21
#define IN1_LEFT_WHEEL 17
#define IN2_LEFT_WHEEL 16

#define ENCA_RIGHT_WHEEL 13
#define ENCB_RIGHT_WHEEL 4
#define PWM_RIGHT_WHEEL 33
#define IN1_RIGHT_WHEEL 15
#define IN2_RIGHT_WHEEL 32

#define ENCA_LEFT_ARM 34
#define ENCB_LEFT_ARM 25
#define PWM_LEFT_ARM 5
#define IN1_LEFT_ARM 19
#define IN2_LEFT_ARM 18

#define ENCA_RIGHT_ARM 27
#define ENCB_RIGHT_ARM 12 
#define PWM_RIGHT_ARM 23
#define IN1_RIGHT_ARM 14
#define IN2_RIGHT_ARM 22


volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA_RIGHT_WHEEL,INPUT);
  pinMode(ENCB_RIGHT_WHEEL,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT_WHEEL),readEncoder,RISING);

  pinMode(ENCA_LEFT_WHEEL,INPUT);
  pinMode(ENCB_LEFT_WHEEL,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT_WHEEL),readEncoder,RISING);

  pinMode(ENCA_RIGHT_ARM,INPUT);
  pinMode(ENCB_RIGHT_ARM,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT_ARM),readEncoder,RISING);

  pinMode(ENCA_LEFT_ARM,INPUT);
  pinMode(ENCB_LEFT_ARM,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT_ARM),readEncoder,RISING);

  pinMode(PWM_RIGHT_WHEEL,OUTPUT);
  pinMode(IN1_RIGHT_WHEEL,OUTPUT);
  pinMode(IN2_RIGHT_WHEEL,OUTPUT);

  pinMode(PWM_LEFT_WHEEL,OUTPUT);
  pinMode(IN1_LEFT_WHEEL,OUTPUT);
  pinMode(IN2_LEFT_WHEEL,OUTPUT);

  pinMode(PWM_RIGHT_ARM,OUTPUT);
  pinMode(IN1_RIGHT_ARM,OUTPUT);
  pinMode(IN2_RIGHT_ARM,OUTPUT);

  pinMode(PWM_LEFT_ARM,OUTPUT);
  pinMode(IN1_LEFT_ARM,OUTPUT);
  pinMode(IN2_LEFT_ARM,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 1500;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC() {
    pos = posi;
  }
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = -1;
  if(u<0){
    dir = 1;
  }

  // signal the motor
  setMotor(1,250,PWM_RIGHT_ARM,IN1_RIGHT_ARM,IN2_RIGHT_ARM);
  setMotor(1,150,PWM_RIGHT_WHEEL,IN1_RIGHT_WHEEL,IN2_RIGHT_WHEEL);
  setMotor(-1,250,PWM_LEFT_ARM,IN1_LEFT_ARM,IN2_LEFT_ARM);
  setMotor(-1,150,PWM_LEFT_WHEEL,IN1_LEFT_WHEEL,IN2_LEFT_WHEEL);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB_RIGHT_WHEEL);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}