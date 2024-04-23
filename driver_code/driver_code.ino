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

//Force rendering
float k = 1.25; //spring constant
float ff_x1 = 4.31;
float ff_y1 = 8.3; //Force field location
float ff_x2 = -1.79;
float ff_y2 = 8.3; 
float distance; //distance of hapkit from target location
float max_attraction = 1;
//Render line equation given ff_x1,ff_y1,ff_x2,ff_y2
float x_online;
float y_online;

unsigned long curr_time=millis();
unsigned long prev_time=millis();

float tsA, tsB;
float x2, y2, x4, y4, xH, yH;
float ttf,tth,yh,xh,thrth;

float x3,y3;
float last_x3, last_y3;
float velocity;

float l1 = 5;
float l2 = 6;
float l3 = 6;
float l4 = 5;
float l5 = 2.5;

float p4p2, php2, p3ph;


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
  y4 = l4*sinf(tsB);

  p4p2 = sqrt(pow(x2 - x4, 2) + pow(y2 - y4, 2)); // distance from p4 to p2

  if(p4p2 != 0){
    php2 = (pow(l2, 2) - pow(l3, 2) + pow(p4p2, 2)) / (2 * p4p2);
  }
  xH = x2 + (php2 / p4p2) * (x4 - x2);
  yH = y2 + (php2 / p4p2) * (y4 - y2);

  p3ph = sqrt(pow(l2, 2) - pow(php2, 2));
  
  x3 = xH + -(p3ph / p4p2) * (y4 - y2);
  y3 = yH + (p3ph / p4p2) * (x4 - x2);

  //test_pair = inverse_kinematics(x3, y3);
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

float norm(float v1_x, float v1_y, float v2_x, float v2_y){
  float result = sqrt((v1_x-v2_x)*(v1_x-v2_x) + (v1_y-v2_y)*(v1_y-v2_y));
  return result;
}

// Jacobian variables
float jd, jb, jh, del1_x2, del1_y2, del5_x4, del5_y4;
float del1_y4 = 0.0;
float del1_x4 = 0.0;
float del5_y2 = 0.0;
float del5_x2 = 0.0;
float del1_d ,del1_b, del1_h;  
float del1_yh,del1_xh,del1_y3,del1_x3;
float del5_d ,del5_b ,del5_h,del5_yh, del5_xh, del5_y3, del5_x3; 
float Fx, Fy;

void Jacobian(){     
    jd = norm(x2,y2,x4,y4);
    jb = norm(x2,y2,xH,yH);
    jh = norm(x3,y3,xH,yH);
    
    if(jd == 0 || jb == 0 || jh == 0){ 
      return;
    }

    del1_x2 = -l1*sin(tsA);  //NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del1_y2 = l1*cos(tsA);
    del5_x4 = -l4*sin(tsB);  //NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del5_y4 = l4*cos(tsB);
    
    //joint 1
    del1_d = ( ((x4-x2)*(del1_x4-del1_x2)) + ((y4-y2)*(del1_y4-del1_y2)) ) / jd;
    del1_b = del1_d - (del1_d*(((l2*l2)-(l3*l3)+(jd*jd))/(2.0*jd*jd)));
    del1_h = -jb*del1_b / jh;
    
    del1_yh = del1_y2 + (del1_b*jd-del1_d*jb)/(jd*jd) * (y4-y2) + jb/jd * (del1_y4 - del1_y2);
    del1_xh = del1_x2 + (del1_b*jd-del1_d*jb)/(jd*jd) * (x4-x2) + jb/jd * (del1_x4 - del1_x2);
    
    del1_y3 = del1_yh - jh/jd * (del1_x4-del1_x2) - (del1_h*jd - del1_d*jh)/(jd*jd) *(x4 - x2);
    del1_x3 = del1_xh + jh/jd * (del1_y4-del1_y2) + (del1_h*jd - del1_d*jh)/(jd*jd) *(y4 - y2);
    
    //joint 2
    del5_d = ( ((x4-x2)*(del5_x4-del5_x2))+((y4-y2)*(del5_y4-del5_y2)) ) / jd;
    del5_b = del5_d - (del5_d*(l2*l2-l3*l3+jd*jd))/(2.0*jd*jd);
    del5_h = -jb*del5_b / jh;
    
    del5_yh = del5_y2 + (del5_b*jd-del5_d*jb)/(jd*jd) * (y4-y2) + jb/jd * (del5_y4 - del5_y2);
    del5_xh = del5_x2 + (del5_b*jd-del5_d*jb)/(jd*jd) * (x4-x2) + jb/jd * (del5_x4 - del5_x2);
    
    del5_y3 = del5_yh - jh/jd * (del5_x4-del5_x2) - (del5_h*jd - del5_d*jh)/(jd*jd) * (x4 - x2);
    del5_x3 = del5_xh + jh/jd * (del5_y4-del5_y2) + (del5_h*jd - del5_d*jh)/(jd*jd) * (y4 - y2);
}


//angle_pair target_thetas = inverse_kinematics(ff_x,ff_y);

float dxydt, dxdt, dydt; 
float dxydt_filt, dxdt_filt, dydt_filt;
float last_dxydt, last_dxdt, last_dydt;

float dt; 

float last_tsA, last_tsB; 
float dtsA, dtsB;
float b = .03;
float dampening;






struct line_render{
  float distance;
  float x_online;
  float y_online;
};




//find shortest distance to line and point on line closest to current position
struct line_render closestpoint(float cur_x,float cur_y, float slope, float intercept){
  distance = (abs((slope*cur_x)-(cur_y)+intercept))/(sqrt(pow(slope,2))+pow(-1,2));
  x_online = (-(-cur_x - (slope*cur_y)) - (slope*intercept))/(pow(slope,2)+pow(-1,2));
  y_online = (slope*(cur_x + (slope*cur_y)) - (-1*intercept))/(pow(slope,2)+pow(-1,2));
  
  line_render line_prop;
  line_prop.distance = distance;
  line_prop.x_online = x_online;
  line_prop.y_online = y_online;

  return line_prop;
}


line_render line_prop;
float target_slope = (ff_y1-ff_y2)/(ff_x1-ff_x2);
float y_intercept = ff_y1 - (target_slope*ff_x1);


void loop() {
  curr_time = millis();//64.0;
  last_tsA = tsA;
  last_tsB = tsB;

  tsA = -radians(0.35*(rp/rs)*updatedPosA) + radians(180-40); //theta1
  tsB = radians(0.35*(rp/rs)*updatedPosB) + radians(40); //theta2


  dt = float(curr_time - prev_time)/1000;
  last_x3 = x3;
  last_y3 = y3;
  forward_kinematics(tsA, tsB);

  // VELOCITY CALCULATIONS
  dxydt = sqrt(pow(last_x3 - x3, 2.) + pow(last_y3 - y3, 2.))/dt;
  dxdt = (x3 - last_x3)/dt;
  dydt = (y3 - last_y3)/dt;
  
  // FILTERS, UNTESTED
  dxydt_filt = dxydt*.9 + last_dxydt*.1;
  dxdt_filt = dxdt*.9 + last_dxdt*.1;
  dydt_filt = dydt*.9 + last_dydt*.1;

  Serial.print(x3);
  Serial.print("a");
  Serial.println(y3);

  
  //force rendering
  line_prop = closestpoint(x3,y3, target_slope, y_intercept);

  if (line_prop.distance < .25){
    Fx = -k*(x3 - line_prop.x_online);
    Fy = -k*(y3 - line_prop.y_online);

  } else {
    //Fx = 0;
    //Fy=0;
    Fx = dxdt_filt*b;
    Fy = -dydt_filt*b;
  }

  Jacobian(); // compute jacobian

  float Tleftx = Fx*del1_x3;
  float Trightx = Fx*del5_x3;
  float Tlefty = Fy*del1_y3;
  float Trighty = Fy*del5_y3;

  TpA= rp/rs * (Tleftx + Tlefty);
  TpB = rp/rs * (Trightx + Trighty);
  
  /*Serial.print(dampening);
  Serial.print(",");
  Serial.print(TpA);
  Serial.print(",");
  Serial.print(TpB);
  Serial.print(",");
  Serial.println(distance);
  */
  last_dxdt = dxdt;
  last_dydt = dydt;
  last_dxydt = dxydt; 
//FORCE OUTPUT
// Determine correct direction for motor torque
  if(TpA > 0) { 
    digitalWrite(dirPinA, LOW);
    digitalWrite(dirPin2A,HIGH);
  } else {
    digitalWrite(dirPinA, HIGH);
    digitalWrite(dirPin2A,LOW);
  }
  if(TpB > 0) { 
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
  prev_time = curr_time;
  delay(30);
}
