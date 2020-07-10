
#ifndef AGV_HEADER_
#define  AGV_HEADER_

// Pin declaration
#define IN1 18 // motor driver
#define serialEnA 13 // front
#define jPulseA 12 // back
#define serialEnB 11 // front
#define jPulseB 10 // back
#define stepperDirection 2 // stepper direction pin
#define stepperPulse 3 // stepper pulse pin
#define stepperEna 4 // enable pin stepper
#define homingSwitch 5 // limit switch bottom (homing)


// variable declaration
#define baudrate 9600
int junctionA = 0; // Front junction
int junctionB = 0; // Back junction
int sensorPositionA, sensorPositionB; // Sensor reading
int motorLeft, motorRight; // motor speed
int motorSpeed; // pid motor speed
const float kp = 1.6; // P element 1.6
const float kd = 2.4; // D element 2.4
float error = 0; // PID eror
float lasterror = 0; // PID previous error
const int sensorSetPoint = 35; // line set point
const int maxSpeed = 16; // max speed
const int baseSpeed = 12; // Operating speed
unsigned int task = 1; // tasking counter
int flagA; // As a conditional marker
bool pulse = LOW; // pulse for stepper motor


// counter for scanning
int scannerStage  = 0; // scanning number stage level in the aisle
int counterAisle = 0;  // number of aisle being scan


// Scanning data
int scanPair = 3; // number of scanning pair(forward and reverse)
unsigned int stepNumber = 62500; // stepper motor step

// serial data
String input; // JSON input
char* board; // board type
int mode; // operation mode
int numberAisle; // number of aisle need to be scan (get from serial)





#endif 