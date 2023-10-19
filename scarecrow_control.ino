/*
	Felicia's ScareCrow Controller. 
	
	Arduino Source code for Arduino Mega 2560
	
	Mega 2560 Connections:
		TPIO pin 5,6 for Left Motor  (IBT-2 PWM inputs)
		TPIO pin 7,8 for Right Motor (IBT-2 PWM inputs)
			Might need to swap polority for each motor for correct directions
		TPIO pin  8: for Pan Servo
		TPIO pin  9: for Tilt Servo
		TPIO pin 10: for Left-Flag Servo
		TPIO pin 11: for Right-Flag Servo
		TPIO pin 14: Serial3 TX	to ScareCrow Monitor RX
		TPIO pin 15: Serial3 RX	to ScareCrow Monitor TX
		GND	: connect to ScareCrow Monitor GND
		Vin	: for power distributor 5V
		GND	: for power distributor GND
	Note:
		This code is mainly referencing the Arduino website's reference library.
*/

#include <Servo.h>

// Motor Connections (Both must use PWM pins)
#define PIN_DRIVE_MOTOR_LEFT_RPWM 4
#define PIN_DRIVE_MOTOR_LEFT_LPWM 5
#define PIN_DRIVE_MOTOR_RIGHT_RPWM 6
#define PIN_DRIVE_MOTOR_RIGHT_LPWM 7

#define PIN_PAN   8
#define PIN_TITL  9
#define PIN_FLAG_LEFT 10
#define PIN_FLAG_RIGHT 11

#define REUSE_PAN_THRESHOLD  100
#define REUSE_TILT_THRESHOLD 100
#define REUSE_DRIVE_THRESHOLD 100
#define REUSE_FLAG_THRESHOLD  6000
#define REUSE_HORN_THRESHOLD  1000

unsigned long last_pan_used =0;
unsigned long last_tilt_used =0;
unsigned long last_flag_used =0;

int posPan  = 0;
int posTilt = 0;
int posFlag = 0;

Servo servoPan;
Servo servoTilt;
Servo servoFlagLeft;
Servo servoFlagRight;

void doServoTiltUp() {
  //if(millis() - last_tilt_used < REUSE_TILT_THRESHOLD) return;
  if(posTilt<=170) {    posTilt += 10;  }
  servoTilt.write(posTilt);
  Serial.print("doServoTiltUp:"); Serial.println(posTilt);
  delay(20);
  last_tilt_used = millis();
}

void doServoTiltDown() {
  //if(millis() - last_tilt_used < REUSE_TILT_THRESHOLD) return;
  if(posTilt>=10) {    posTilt -= 10;  }
  servoTilt.write(posTilt);
  delay(20);
  Serial.print("doServoTiltDown:"); Serial.println(posTilt);
  last_tilt_used = millis();
}

void doServoPanLeft() {
  //if(millis() - last_pan_used < REUSE_PAN_THRESHOLD) return;
  if(posPan>=10) {    posPan -= 10;  }
  servoPan.write(posPan);
  delay(20);
  Serial.print("doServoPanLeft:"); Serial.println(posPan);
  last_pan_used = millis();
}

void doServoPanRight() {
  //if(millis() - last_pan_used < REUSE_PAN_THRESHOLD) return;
  if(posPan<=170) {    posPan += 10;  }
  servoPan.write(posPan);
  delay(20);
  Serial.print("doServoPanRight:"); Serial.println(posPan);
  last_pan_used = millis();
}

void doWaveFlags() {
  //if(millis() - last_flag_used < REUSE_FLAG_THRESHOLD) return;
  
  for (int i = 0; i <= 120; i += 1) { // goes from 0 degrees to 120 degrees
    // in steps of 1 degree
    servoFlagLeft.write(i);
    servoFlagRight.write(120-i);
    delay(15);
  }
  for (int i = 120; i >= 0; i -= 1) { // goes from 120 degrees to 0 degrees
    servoFlagLeft.write(i);
    servoFlagRight.write(120-i);
    delay(15);
  }
  Serial.println("doWaveFlags:");
  last_flag_used = millis();
}



void setupSerial() {
  Serial3.begin(9600);
  Serial.begin(9600); // open the serial port at 9600 bps
  Serial.println("Serial Ready");
}

void setupServos() {
  servoPan.attach(PIN_PAN);  // attaches the servo
  servoTilt.attach(PIN_TITL);  // attaches the servo
  servoFlagLeft.attach(PIN_FLAG_LEFT);  // attaches the servo
  servoFlagRight.attach(PIN_FLAG_RIGHT);  // attaches the servo
  posPan  = 0;
  posTilt = 0;
  servoPan.write(posPan);
  servoTilt.write(posTilt);
  servoFlagLeft.write(0);
  servoFlagLeft.write(120);
  unsigned long last_pan_used  = millis();
  unsigned long last_tilt_used = millis();
  unsigned long last_flag_used = millis();
  Serial.println("Servos Ready");
}

void setupDriveMotors() {
  // Set motor connections as outputs
  pinMode(PIN_DRIVE_MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(PIN_DRIVE_MOTOR_LEFT_LPWM, OUTPUT);
  pinMode(PIN_DRIVE_MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_DRIVE_MOTOR_RIGHT_LPWM, OUTPUT); 
  // Stop motors
  analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM, 0);
  analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM, 0);
  analogWrite(PIN_DRIVE_MOTOR_RIGHT_RPWM, 0);
  analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM, 0);
  Serial.println("DriveMotors Ready");
}

void setup() {
  setupSerial();
  setupServos();
  setupDriveMotors();
}

void doDriveOnlyLeft(int mode) {
  analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);
  analogWrite(PIN_DRIVE_MOTOR_RIGHT_RPWM,0);
  if(mode>0) {
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,250);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,0);        
  } else if(mode<0) {
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,250);
  } else {
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,0);    
  }
  Serial.print("doDriveOnlyLeft:");
  Serial.println(mode);
  delay(1000);
}

void doDriveOnlyRight(int mode) {
  analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,0);
  analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,0);
  if(mode>0) {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,250);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);       
  } else if(mode<0) {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,250);
  } else {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);   
  }
  Serial.print("doDriveOnlyRight:");
  Serial.println(mode);
  delay(1000);
}

void doDriveBoth(int mode) {
  
  if(mode>0) {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,250);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_RPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,250);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,0);
  } else if(mode<0) {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_RPWM,250);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,250);
  } else {
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_RIGHT_RPWM,0);   
    analogWrite(PIN_DRIVE_MOTOR_LEFT_LPWM,0);
    analogWrite(PIN_DRIVE_MOTOR_LEFT_RPWM,0);
  }
  Serial.print("doDriveBoth:");
  Serial.println(mode);
  delay(1000);
}

void doMechanicalTest() {
  Serial.println("doMechanicalTest Begin");
  doDriveBoth(1);
  doDriveBoth(1);
  doDriveBoth(1);
  doDriveBoth(0);

  doServoPanLeft();
  doServoPanLeft();
  doServoPanLeft();
  doServoPanLeft();
  doServoPanRight();
  doServoPanRight();
  doServoPanRight();
  doServoPanRight();
  
  doServoTiltUp();
  doServoTiltUp();
  doServoTiltUp();
  doServoTiltUp();
  doServoTiltDown();
  doServoTiltDown();
  doServoTiltDown();
  doServoTiltDown();

  doWaveFlags();

  doDriveBoth(-1);
  doDriveBoth(-1);
  doDriveBoth(-1);
  doDriveBoth(0);
  Serial.println("doMechanicalTest End");
}

void loop() {
  if (Serial3.available()) {
    String cmd = Serial3.readString();
    cmd.trim();
    //Serial.print("Commad = ");Serial.println(cmd);
    if(cmd=="WV") { doWaveFlags(); }
    else if(cmd=="HN") { doMechanicalTest(); } 
    else if(cmd=="PL") { doServoPanLeft(); }
    else if(cmd=="PR") { doServoPanRight(); }
    else if(cmd=="TU") { doServoTiltUp(); }
    else if(cmd=="TD") { doServoTiltDown(); }
    else if(cmd=="FS") { doDriveOnlyLeft(1); }
    else if(cmd=="FF") { doDriveBoth(1); } 
    else if(cmd=="SF") { doDriveOnlyRight(1); }
    else if(cmd=="SS") { doDriveBoth(0); } 
    else if(cmd=="RS") { doDriveOnlyLeft(-1); }
    else if(cmd=="RR") { doDriveBoth(-1); } 
    else if(cmd=="SR") { doDriveOnlyRight(-1); }
  }
}