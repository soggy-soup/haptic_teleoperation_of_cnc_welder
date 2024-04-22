//2 DOF Hapkit with GRBL interface

#include <math.h>
#define pi 3.141592
#include <string.h>
#include <stdio.h>

//Pin declares for Motor+Encoder A
int pwmPinA = 5;         // PWM output pin for motor 1
int dirPinA = 8;         // direction output pin for motor 1
int dirPin2A = 9;        // 2nd direction output pin for motor 1
//int sensorPosPinA = A2;  // input pin for MR sensor *NOT USED*
//int fsrPinA = A3;        // input pin for FSR sensor *NOT USED*
int encPin1A = 2;        // encoder read pin 1
int encPin2A = 7;        // encoder read pin 2
int encIncA = 0;

//Pin declares for Motor+Encoder B
int pwmPinB = 6;         // PWM output pin for motor 1
int dirPinB = 10;         // direction output pin for motor 1
int dirPin2B = 13;        // 2nd direction output pin for motor 1
//int sensorPosPinB = A2;  // input pin for MR sensor *NOT USED*
//int fsrPinB = A3;        // input pin for FSR sensor *NOT USED*
int encPin1B = 11;        // encoder read pin 1
int encPin2B = 12;        // encoder read pin 2
int encIncB = 0;

//Pulley and sector radii
double rs = 0.075;   //[m]
double rp = 0.0094;   //[m]

//Leader variables
int updatedPosA = 0;     // keeps track of the encoder position
volatile int lastRawPosA = 0; // 'volatile' as it is modified in an interrupt                                 
int updatedPosB = 0;     // keeps track of the encoder position
volatile int lastRawPosB = 0; // 'volatile' as it is modified in an interrupt                     // last last raw reading from MR sensor
int EncLookup[16] = { 0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0 };  // encoder increment lookup table

// Kinematics variables
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double forceA = 0;           			// Force at the handle
double TpA = 0;              			// Torque of the motor pulley
double dutyA = 0;            			// Duty cylce (between 0 and 255)
unsigned int outputA = 0;    			// Output command to the motor

// Force output variables
double forceB = 0;           			// Force at the handle
double TpB = 0;              			// Torque of the motor pulley
double dutyB = 0;            			// Duty cylce (between 0 and 255)
unsigned int outputB = 0;    			// Output command to the motor

unsigned long curr_time=0.0;
unsigned long prev_time=0.0;

void readEncoderA() {
  int newRawPosA = digitalRead(encPin1A) * 2 + digitalRead(encPin2A);
  int encIncA = EncLookup[lastRawPosA * 4 + newRawPosA];

  updatedPosA += encIncA;
  lastRawPosA = newRawPosA;
}

void readEncoderB() {
  int newRawPosB = digitalRead(encPin1B) * 2 + digitalRead(encPin2B);
  int encIncB = EncLookup[lastRawPosB * 4 + newRawPosB];

  updatedPosB += encIncB;
  lastRawPosB = newRawPosB;
}

// work with known geometries
// show on welder - even if in a very controlled setting
// tune a damping parameter to correspond with the cnc machine
// test encoder in lab


void setup() {
   // Set up serial communication
  Serial.begin(115200);
  //MOTOR A
  // Set PWM frequency 
  //setPwmFrequency(pwmPinA,1); 
  
  // Input pins
  //pinMode(sensorPosPinA, INPUT); // set MR sensor pin to be an input
  //pinMode(fsrPinA, INPUT);       // set FSR sensor pin to be an input
  pinMode(encPin1A,INPUT);       // set encoder pin 1 to be an input
  pinMode(encPin2A,INPUT);       // set encoder pin 2 to be an input

  // Output pins
  pinMode(pwmPinA, OUTPUT);  // PWM pin for motor A
  pinMode(dirPinA, OUTPUT);  // dir pin for motor A
  pinMode(dirPin2A,OUTPUT);   // second dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPinA, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinA, LOW);  // set direction
  digitalWrite(dirPin2A,LOW);
  
  // Initialize position valiables
  lastRawPosA = digitalRead(encPin1A) * 2 + digitalRead(encPin2A);

  attachInterrupt(digitalPinToInterrupt(encPin1A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2A), readEncoderA, CHANGE);

  //MOTOR B
  // Set PWM frequency 
  //setPwmFrequency(pwmPinB,1); 
  
  // Input pins
  //pinMode(sensorPosPinB, INPUT); // set MR sensor pin to be an input
  //pinMode(fsrPinB, INPUT);       // set FSR sensor pin to be an input
  pinMode(encPin1B,INPUT);       // set encoder pin 1 to be an input
  pinMode(encPin2B,INPUT);       // set encoder pin 2 to be an input

  // Output pins
  pinMode(pwmPinB, OUTPUT);  // PWM pin for motor A
  pinMode(dirPinB, OUTPUT);  // dir pin for motor A
  pinMode(dirPin2B,OUTPUT);   // second dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPinB, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinB, LOW);  // set direction
  digitalWrite(dirPin2B,LOW);
  
  // Initialize position valiables
  lastRawPosB = digitalRead(encPin1B) * 2 + digitalRead(encPin2B);

  attachInterrupt(digitalPinToInterrupt(encPin1B), readEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2B), readEncoderB, CHANGE);
}

float tsA, tsB;
float x2, y2, x4, y4, xH, yH;
float ttf,tth,yh,xh,thrth;

float x3,y3;
float machine_x, machine_y;

float l1 = 5;
float l2 = 6;
float l3 = 6;
float l4 = 5;
float l5 = 2.5;

float p4p2, php2, p3ph;


struct position{
  float x;
  float y;
};

struct angle_pair {
  float t1;
  float t2;
};

angle_pair test_pair;
// https://replit.com/@jafferali1/inversekinematics#main.cpp
void forward_kinematics(float tsA, float tsB){
  x2 = l1*cosf(tsA);
  //Serial.print(x2);
  //Serial.print("a");
  y2 = l1*sinf(tsA);
  x4 = l4*cosf(tsB) + l5;
  //Serial.print(x4);
  //Serial.print("a");
  y4 = l4*sinf(tsB);

  p4p2 = sqrt(pow(x2 - x4, 2) + pow(y2 - y4, 2)); // distance from p4 to p2
  php2 = (pow(l2, 2) - pow(l3, 2) + pow(p4p2, 2)) / (2 * p4p2);

  xH = x2 + (php2 / p4p2) * (x4 - x2);
  yH = y2 + (php2 / p4p2) * (y4 - y2);

  p3ph = sqrt(pow(l2, 2) - pow(php2, 2));
  
  x3 = xH + -(p3ph / p4p2) * (y4 - y2);
  y3 = yH + (p3ph / p4p2) * (x4 - x2);

  test_pair = inverse_kinematics(x3, y3);



}

float l13, l53, alphaOne, betaOne, thetaOne, alphaFive, betaFive, thetaFive;

struct angle_pair inverse_kinematics(float target_x, float target_y){
  l13 = sqrt(pow(target_x, 2) + pow(target_y, 2));
  l53 = sqrt(pow(l5 - target_x, 2) + pow(target_y, 2));

  alphaOne = acos((pow(l1, 2) + pow(l13, 2) - pow(l2, 2)) / (2 * l1 * l13));
  betaOne = atan2(target_y, target_x);
  thetaOne = alphaOne + betaOne;


  alphaFive = atan2(target_y, l5 - target_x);
  betaFive = acos((pow(l53, 2) + pow(l4, 2) - pow(l3, 2)) / (2 * l53 * l4));
  thetaFive = pi - (alphaFive + betaFive);

  angle_pair end_position;

  end_position.t1 = thetaOne;
  end_position.t2 = thetaFive;

  return end_position;
}

String machine_pos = "";
void serial_read_machine(){
  if (Serial.available()){
    machine_pos = Serial.readString();
  }
}


void loop() {
  curr_time = millis();//64.0;
  // 
  tsA = -radians(0.35*(rp/rs)*updatedPosA) + radians(180-40); //theta1
  tsB = radians(0.35*(rp/rs)*updatedPosB) + radians(40); //theta2

  forward_kinematics(tsA, tsB);
  serial_read_machine();

  Serial.print(x3);
  Serial.print("a");
  Serial.print(y3);
  Serial.print("a");
  Serial.println(machine_pos);
  delay(30);
//  Serial.println("Test");
//  Serial.println(String(x3)+"a"+String(y3));
/*
  Serial.print(x3);
  Serial.print(" ");
  Serial.println(y3);
*/
  forceA = 0;
  forceB = 0;


  TpA= rp/rs * forceA;
  TpB = rp/rs * forceB;





//FORCE OUTPUT
// Determine correct direction for motor torque
  if(forceA > 0) { 
    digitalWrite(dirPinA, LOW);
    digitalWrite(dirPin2A,HIGH);
  } else {
    digitalWrite(dirPinA, HIGH);
    digitalWrite(dirPin2A,LOW);
  }
  if(forceB > 0) { 
    digitalWrite(dirPinB, LOW);
    digitalWrite(dirPin2B,HIGH);
  } else {
    digitalWrite(dirPinB, HIGH);
    digitalWrite(dirPin2B,LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  dutyA = sqrt(abs(TpA)/0.03);
  dutyB = sqrt(abs(TpB)/0.03);
  // Make sure the duty cycle is between 0 and 100%
  if (dutyA > 1) {            
    dutyA = 1;
  } else if (dutyA < 0) { 
    dutyA = 0;
  }  
  if (dutyB > 1) {            
    dutyB = 1;
  } else if (dutyB < 0) { 
    dutyB = 0;
  }  
  
  outputA = (int)(dutyA* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinA,outputA);  // output the signal   
  outputB = (int)(dutyB* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinB,outputB);  // output the signal 




  if(curr_time-prev_time>33){
   //PUT CODE HERE TO RUN EVERY 100 ms
   //Serial.println(xh); //UNCOMMENT for part 1A
   //Serial.println(String(tsA)+"a"+String(tsB));
   //Serial.println(String(x3)+"a"+String(y3));
   //Serial.println(String(updatedPosA)+"a"+String(updatedPosB));
   prev_time = curr_time;
  }

  
}








//PWM Set Function
/*
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
*/