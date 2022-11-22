#include <AVR_RTC.h>



//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/
//**************************************

#include <util/atomic.h>

//**************************************

#define ENCA 3      //Encoder pinA White
#define ENCB 2      //Encoder pinB Yellow
#define PWM 11       //motor PWM pin Orange
#define IN2 6       //motor controller pin2 Green
#define IN1 7       //motor controller pin1 Yellow
const int buttonPin = 12; //reel-in btn Pink

volatile int posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//********************************************

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int counter = 0;
float wave = 0;
int retning=0;


void setup() {

  Serial.begin (9600);

  // ENCODER
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //btn config
  pinMode(buttonPin, INPUT);

  Serial.println("Input, Angle, DeltaT");
}

//************************************************

void loop() {

  // Set target position (1024 * X rounds) NB! Check encoder dipswitch for resolution (standard is 2048 for AMT 102V).
  int targTop = 3900; //4050 is 20CM from egg to floor without spring
  int targBot = 0; //0 is floor value

  //PID constants
  float kp = 0.4; //0.4
  float kd = 0.02; //0.02
  float ki = 0.0025; //0.004

// tall for trym
// 1.94, 2.29


  // Funka veldig fint!
  // 0.4
  // 0.02
  // 0.0025

  // Denne funka fint, rister litt på bunnen
  // 0.4
  // 0.025
  // 0.004 

  //time diference
  long currT = micros();
  long currTmillis = millis();
  float deltaT = ((float) (currT - prevT)) / (1.0e6);
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  //Error calculation
  int e = (pos - targBot);

  //PID Calculation
  float dedt = (e - eprev) / (deltaT); //Derivative
  eintegral = eintegral + e * deltaT; //Integral
  float u = kp * e + kd * dedt + ki * eintegral; //Control signal

  //Motor power
  float pwr = fabs(u);  //fabs == floating point, absolute value
  if (pwr > 170) {
    pwr = 170; //Capping
  }

  //Motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1; //if the control signal is negative, we flip the motor direction
  }

  //Store previous error
  eprev = e;

//counter++;
//counter = counter % 150;
//wave = cos((counter*3.14)/75);
//Serial.print(counter);
//Serial.print(" ");
//Serial.println(wave);
//
//if (wave > 0){
//  retning = 1;
//}
//else{
//  retning = -1;
//}
//setMotor(retning, abs(wave)*125,PWM,IN1,IN2);



  if (buttonPushCounter == 0) {

    setMotor(0, 0, PWM, IN1, IN2);
    buttonCounter(buttonPin);
    //Serial.println("wait for it: ");
  }
  if (buttonPushCounter == 1) {
    buttonCounter(buttonPin);
    if (pos < targTop) {
      buttonCounter(buttonPin);
      setMotor(-1, 170, PWM, IN1, IN2);
      //Serial.println("LOADING: ");

    }
    else {
    }

  }
  if (buttonPushCounter == 2) {
    //Serial.println("REGULATIN BBY ;) ");
    setMotor(dir, pwr, PWM, IN1, IN2);
    buttonCounter(buttonPin);
  }
      Serial.print("button: ");
      Serial.print(buttonPushCounter);
      Serial.print(" . pos: ");
      Serial.print(pos);
      Serial.print("  u: ");
      Serial.println(u);
        
//    Serial.print( (u / (255 / 12)) * 1000 );
//    Serial.print(",");
//    Serial.print(pos / 2.844);
//    Serial.print(",");
//    Serial.println(currTmillis);
}
















//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


//ENCODER
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  }
  else {
    posi--;
  }
}



//BUTTON
int buttonCounter(int pin)
{
  buttonState = digitalRead(pin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
    } else {
    }
  }
  lastButtonState = buttonState;
  buttonPushCounter = buttonPushCounter % 3;
  return buttonPushCounter;
}
