#include <Wire.h>
#include <Servo.h>
#include <Ultrasonic.h>

///////////////////////Inputs/outputs///////////////////////
#define pino_trigger 4
#define pino_echo 5
const int servoP = A0;
const int servoI = A1;
const int servoD = A2;
Servo myservo;  // create servo object to control a servo, later attatched to D9
Ultrasonic ultrasonic(pino_trigger, pino_echo);
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=0.02; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 25;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////



void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(125); //Put the servco at angle 125, so the balance is in the middle
  time = millis();
  pinMode(servoP, INPUT);
  pinMode(servoI, INPUT);
  pinMode(servoD, INPUT);
}

void loop() {
  if (millis() > time+period)
  {
    kp = int(analogRead(servoP)/100)*0.4;
    ki = int(analogRead(servoI)/100)*0.1;
    kd = int(analogRead(servoD)/100)*0.4;

    float cmMsec;
    long microsec = ultrasonic.timing();
    cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);

    time = millis();    
    distance = cmMsec;   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-13 < distance_error && distance_error < 13)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total+20);
    distance_previous_error = distance_error;
  }
}
