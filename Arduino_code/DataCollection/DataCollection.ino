#include <AVR_RTC.h>



//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/
//**************************************

#include <util/atomic.h>
#include "timer.h"

//**************************************

#define ENCA 3      //Encoder pinA White
#define ENCB 2      //Encoder pinB Yellow
#define PWM 11       //motor PWM pin Orange
#define IN2 6       //motor controller pin2 Green
#define IN1 7       //motor controller pin1 Yellow
const int buttonPin = 12; //reel-in btn Pink

Timer timer;

volatile int posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//********************************************

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

unsigned long int inputV[] = {50, 60, 100, 125, 150, 160};
int wait = 500;
int pause = 500;
int retning = -1;
int counter = 0;
int limit = 6;


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
  delay(5000);
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


  //Error calculation
  int e = (pos - targBot);

  //PID Calculation
  float dedt = (e - eprev) / (deltaT); //Derivative
  eintegral = eintegral + e * deltaT; //Integral
  float u = kp * e + kd * dedt + ki * eintegral; //Control signal

  //Motor power
  float pwr = fabs(u);  //fabs == floating point, absolute value
  if (pwr > 255) {
    pwr = 255; //Capping
  }

  //Motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1; //if the control signal is negative, we flip the motor direction
  }

  //Store previous error
  eprev = e;












  for (int i = 0; i < 2; i++) {
    if (i > 0) {
      retning = -1;
    }
    else {
      retning = 1;
    }
    driveMotor(retning, pos, inputV[counter]);
    stopMotor(retning, pos);
  }

  counter ++;

  if (counter == limit) {
    delay(5000);
  }

  counter = counter % limit;


}
















//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == 1) {
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
  return posi;
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

void printData(int pos, int input) {
  Serial.print("Hastighet ");
  Serial.print(input);
  Serial.print(" . pos: ");
  Serial.println(pos);
}

void printCSV(int pos, int input, unsigned long int timeNow) {
  Serial.print(input/21.25);
  Serial.print(",");
  Serial.print(pos / 1.42);
  Serial.print(",");
  Serial.println(timeNow);
}

void driveMotor(int retning, int pos, int inputV) {
  timer.getTimer(wait);
  while (!timer.timerHasExpired()) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = posi;
    }
    setMotor(retning, inputV, PWM, IN1, IN2);
    //    Serial.print("drive motor: ");
    //printData(pos, inputV);
    unsigned long int timeNow = millis();
    printCSV(pos, inputV*retning, timeNow);

  }


}

void stopMotor(int retning, int pos) {
  timer.getTimer(pause);
  while (!timer.timerHasExpired()) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = posi;
    }
    setMotor(retning, 0, PWM, IN1, IN2);
    //    Serial.print(" stop motor: ");
    //printData(pos, 0);
    unsigned long int timeNow = millis();
    printCSV(pos, 0, timeNow);

  }

}
