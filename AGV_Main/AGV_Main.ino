// 10/7/2020 by Yee Kong

// Sketch created on 13/4/2020 at 0930hr by Afiq
// Line sensor and motor
// Design to test on AGV project with PID

// motor speed chart
/*

   |-------------------|-------------------|
  -100                 0                  100
  reverse             stop              forward

*/

// line location chart
/*

   |-------------------|-------------------|
   0                   35                  70
  left              center               right

*/

// stepper motor direction (low = up) (high = down)

/*
  task list
  task 1: finding aisle + turning tu aisle + align
  task 2: forward scanning
  task 3: reverse scanning
  task 4: return to mainline
  task 5: parking
*/

/*
  uart list
  uart 0: usb
  uart 1: motor driver MDDS30
  uart 2: front line sensor
  uart 3: back line sensor
*/

// Library use in the sketch
#include <Cytron_SmartDriveDuo.h>
#include <ArduinoJson.h>
#include "AGV_Main.h"
// Initiaization
Cytron_SmartDriveDuo motor(SERIAL_SIMPLFIED, IN1, baudrate); // initialize connection to motor driver

void setup()
{
  // Starting the serial connection
  Serial.begin(115200);
  //Serial1.begin(baudrate);
  Serial2.begin(baudrate); // sensor front
  Serial3.begin(baudrate); // sensor back

  // pinMode setting
  pinMode(serialEnA, OUTPUT); // front
  pinMode(jPulseA, INPUT);
  pinMode(serialEnB, OUTPUT); //back
  pinMode(jPulseB, INPUT);
  pinMode(stepperDirection, OUTPUT); // stepper
  pinMode(stepperPulse, OUTPUT);
  pinMode(stepperEna, OUTPUT);
  pinMode(homingSwitch, INPUT_PULLUP);

  // Setting initial
  digitalWrite(serialEnA, HIGH); // stop serial front reading
  digitalWrite(serialEnB, HIGH); // stop serial back reading

  motor.control(0, 0); // stop motor
  clearJunctionA(); // clear all junction
  clearJunctionB();

  Serial.println("Startup"); // Done setup

}

void loop()
{
  serialRead();

  switch(mode)
  {
    case 1: automation(); break;
    case 2: calibrate(); break;
    case 3: controlHoming(); break;
    default: motor.control(0,0); task = 1; break;
  }

}


// -------------------------- Task 1 -------------------------- //

// Finding junction and turn to aisle
void movingA()
{
  junctionA = getJunctionA();

  // find for first 2 junction
  if (junctionA < 2)
  {
    forwardLine(); // straight pid
    junctionA = getJunctionA();
    //    Serial.println(junctionA);
    flagA = 1;
  }
  else
  {
    motor.control(0, 0);
  }


  // Turning to the aisle
  // Eliminate line into open space
  if (junctionA == 2 && flagA == 1)
  {
    motor.control(-11, 11);
    delay(5750);
    motor.control(0, 0);
    delay(100);
    flagA = 2;
  }


  // sense aisle line (find aisle line)
  if (flagA == 2 && junctionA < 3)
  {
    leftTurn(); // call left turning function
  }


  // if line detected continue straight
  if (flagA == 3 )
  {
    if (junctionA < 3)
    {
      forwardLine(); // straight pid
      junctionA = getJunctionA();
      //      Serial.println("Here"); // debug
    }
    else
    {
      motor.control(0, 0);
    }
  }


  // update task
  if (junctionA == 3)
  {
    delay(350);
    task ++;
  }
}

// -------------------------- Task 2 -------------------------- //
// forward scanning
void movingB()
{
  // find end of aisle
  junctionA = getJunctionA();

  // finding end of aisle
  if (junctionA < 1)
  {
    forwardLine(); // straight pid
    junctionA = getJunctionA();
    //    Serial.println(junctionA)
    flagA = 1;
  }
  else
  {
    motor.control(0, 0);
  }


  // update task
  if ( junctionA == 1 && flagA == 1)
  {
    delay(350);
    task ++;
  }
}

// -------------------------- Task 3 -------------------------- //
// reverse scanning
void movingC()
{
  // find end of aisle
  junctionA = getJunctionA();


  // finding start end of aisle
  if (junctionA < 2)
  {
    reverseLine(); // straight pid
    junctionA = getJunctionA();
    //    Serial.println(junctionA);
    flagA = 1;
  }
  else
  {
    motor.control(0, 0);
  }


  // update task
  if ( junctionA == 2 && flagA == 1)
  {
    motor.control(0, 0);
    scannerStage ++; // update number of scane stage
    task ++;
  }
}

// -------------------------- Task 4 -------------------------- //
// back to main line
void movingD()
{
  //  Serial.println("Here"); // debug
  junctionB = getJunctionB();

  // find for reverse marker
  if (junctionB < 1)
  {
    reverseLine(); // straight pid
    junctionB = getJunctionB();
    //    Serial.println(junctionA);
    flagA = 1;
  }
  else
  {
    motor.control(0, 0);
  }


  // eliminate line sensor into open space
  if (junctionB == 1 && flagA == 1)
  {
    motor.control(11, -11);
    delay(5750);
    motor.control(0, 0);
    delay(100);
    flagA = 2;
  }


  // find mainline
  if (flagA == 2)
  {
    rightTurn(); // call right turning function
  }


  //  aligning to main line
  if (flagA == 3)
  {
    //    Serial.println("Here flag 3"); // debug
    motor.control(10, 10);
    delay(750);
    motor.control(0, 0);
    flagA = 4;
  }


  // update task
  if (flagA == 4)
  {
    counterAisle ++; // update number of aisle scan
    task++;
  }
}

// -------------------------- Task 5 -------------------------- //
// parking
void parking()
{
  junctionB = getJunctionB();
  //  Serial.println(junctionB);
  //  Serial.println("Here");


  // finding start end of aisle
  if (junctionB < ((numberAisle * 2) - 1) )
  {
    reverseLine(); // straight pid
    junctionB = getJunctionB();
    //    Serial.println(junctionB);
    flagA = 1;
  }

  if (flagA == 1 && junctionB == ((numberAisle * 2) - 1))
  {
    motor.control(0, 0);
    task++;
  }
}

// -------------------------- Counter -------------------------- //
void aisleCounter() // check aisle scanning
{
  if (counterAisle == numberAisle)
  {
    scannerStage = 0;
    task++;
  }
  else
  {
    scannerStage = 0; // reset stage that have been scan
    task = 1; // number for find two junction aisle. (restart for the next)
  }
}


void levelCounter() // check level scanning
{
  if (scannerStage == scanPair)
  {
    task++;
  }
  else
  {
    task = 3; // to start scan next pair
  }
}

// -------------------------- turn auto -------------------------- //
// front sensor
void leftTurn() // left turn find line using front sensor
{
  // reading line sensor front
  digitalWrite(serialEnA, LOW);
  while ( Serial2.available() <= 0);
  sensorPositionA = Serial2.read();
  digitalWrite(serialEnA, HIGH);

  do  {
    // reading line sensor front
    digitalWrite(serialEnA, LOW);
    while ( Serial2.available() <= 0);
    sensorPositionA = Serial2.read();
    digitalWrite(serialEnA, HIGH);

    // If sensor detected
    if (sensorPositionA <= 35)
    {
      flagA = 3;
      //    Serial.println(flagA);
    }

    motor.control(-11, 11); // move left
    //    Serial.println(sensorPositionA);
  }  while (sensorPositionA > 30);
}


// back sensor
void rightTurn() // left turn find line using front sensor
{
  //  Serial.println("Here"); // debug

  // reading line sensor front
  digitalWrite(serialEnB, LOW);
  while ( Serial3.available() <= 0);
  sensorPositionB = Serial3.read();
  digitalWrite(serialEnB, HIGH);

  do  {
    // reading line sensor front
    digitalWrite(serialEnB, LOW);
    while ( Serial3.available() <= 0);
    sensorPositionB = Serial3.read();
    digitalWrite(serialEnB, HIGH);

    if (sensorPositionB <= 43)
    {
      flagA = 3;
    }

    motor.control(11, -11); // move right
    //    Serial.println(sensorPositionB);
  }  while (sensorPositionB > 40);
}

// -------------------------- serial -------------------------- //
// read serial and deserialize JSON
void serialRead()
{
  StaticJsonDocument<256> data; // create JSON object

  if (Serial.available() > 0)
  {
    input = Serial.readStringUntil('\n'); // read serial

    DeserializationError err = deserializeJson(data, input); // check if error | deserialize JSON
    // {"Board":"Arduino Mega","Mode":2,"Aisle":1}

    if (err)
    {
      Serial.print("Deserialization Error: ");
      Serial.println(err.c_str());
    }

    // Spliting JSON data
    board = data["Board"];
    mode = data["Mode"];
    numberAisle = data["Aisle"];
  }
}

// -------------------------- automation -------------------------- //
void automation()
{
  switch (task)
  {
    case 1: homing(); break; // home the camera
    case 2: movingA(); break; // call task 1
    case 3: stepperUpForward(); break; // call increse stepper height
    case 4: clearAllJunction(); break; // clear previous
    case 5: movingB(); break; // call task 2 (forward scanning)
    case 6: stepperUpReverse(); break; // call increse stepper height
    case 7: clearAllJunction(); break; // clear previous
    case 8: movingC(); break; // call task 3 (reverse scanning)
    case 9: levelCounter(); break;
    case 10: clearAllJunction(); break; // clear previous
    case 11: movingD(); break; // reverse to mainline (task 4)
    case 12: homing(); break; // home the camera
    case 13: clearAllJunction(); break; // clear previous
    case 14: aisleCounter(); break; // check number of aisle scan
    case 15: clearAllJunction(); break; // clear previous
    case 16: parking(); break; // parking (task 5)
    case 17: clearAllJunction(); break; // clear previous
    case 18: stepperParking(); break;
    case 19: mode = 15; break;// break first switch case
  }
}

// -------------------------- calibrate -------------------------- //

void calibration_Front()
{
  char address = 0x01;
  char command = 'C';
  char data = 0x00;
  char checksum = address + command + data;

  Serial2.write(address);
  Serial2.write(command);
  Serial2.write(data);
  Serial2.write(checksum);
}

void calibration_Back()
{
  char address = 0x01;
  char command = 'C';
  char data = 0x00;
  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);
}


void calibrate()
{
  calibration_Front();
  calibration_Back();
  // movement
  motor.control(-11, 11);
  delay(750);
  motor.control(11, -11);
  delay(1500);
  motor.control(-11, 11);
  delay(1500);
  motor.control(11, -11);
  delay(1500);
  motor.control(-11, 11);
  delay(750);
  motor.control(0, 0);
  delay(100);
  mode = 15;

}

// -------------------------- stepper -------------------------- //
void stepperUpForward() // for forward scanning
{
  if (scannerStage  == 0 )
  {
    task++;
  }
  else
  {
    motor.control(0 , 0);
    digitalWrite(stepperEna, HIGH); // enable driver
    digitalWrite(stepperDirection, LOW); // go up

    for (unsigned int i = 0; i < stepNumber; i++)
    {
      pulse = !pulse;
      digitalWrite(stepperPulse, pulse);
      delayMicroseconds(250);
      if (i == (stepNumber - 1))
      {
        motor.control(11, 11);
        delay(2150);
        task++;
      }
    }
  }
}

void stepperUpReverse()// for reverse scanning
{
  motor.control(0 , 0);
  digitalWrite(stepperEna, HIGH); // enable driver
  digitalWrite(stepperDirection, LOW); // go up

  for (unsigned int i = 0; i < stepNumber; i++)
  {
    pulse = !pulse;
    digitalWrite(stepperPulse, pulse);
    delayMicroseconds(250);
    if (i == (stepNumber - 1))
    {
      task++;
    }
  }
}

void stepperParking()
{
  motor.control(0 , 0);
  digitalWrite(stepperEna, HIGH); // enable driver
  digitalWrite(stepperDirection, LOW); // go up

  for (unsigned int i = 0; i < 200; i++)
  {
    pulse = !pulse;
    digitalWrite(stepperPulse, pulse);
    delayMicroseconds(250);
    if (i == 199)
    {
      counterAisle = 0;
      task++;
    }
  }
}

// -------------------------- homing -------------------------- //
// homing sequence
void homing()
{
  digitalWrite(stepperDirection, HIGH); // down

  if (digitalRead(homingSwitch) == HIGH) // if not touching
  {
    digitalWrite(stepperEna, HIGH);
    pulse = !pulse;
    digitalWrite(stepperPulse, pulse);
    delayMicroseconds(250);
  }
  else
  {
    task++;
  }
}

// -------------------------- gui stepper -------------------------- //
void controlHoming()
{
  switch(task)
  {
    case 1: homing(); break;
    case 2: mode = 15; break;
  }
}

// -------------------------- movement line  -------------------------- //
void forwardLine() // move forward function with line sensor
{
  // reading line sensor front
  digitalWrite(serialEnA, LOW);
  while ( Serial2.available() <= 0);
  sensorPositionA = Serial2.read();
  digitalWrite(serialEnA, HIGH);

  if (sensorPositionA == 255 ) // if no line detected
  {
    motor.control(0, 0);
  }
  else
  {
    // PID calculation
    error = sensorPositionA - sensorSetPoint; // calculating error
    motorSpeed = kp * (error) + kd * (error - lasterror); // calculate the pid
    lasterror = error; // update the last error

    // Adjust speed based on the line sensor
    motorRight = baseSpeed - motorSpeed;
    motorLeft = baseSpeed + motorSpeed;

    // Adjust speed if maximum
    if (motorRight > maxSpeed)
      motorRight = maxSpeed;
    if (motorLeft > maxSpeed)
      motorLeft = maxSpeed;

    // If the motor is reverse (negative)
    if (motorRight < 0)
      motorRight = 0;
    if (motorLeft < 0)
      motorLeft = 0;

    motor.control(motorLeft, motorRight);
  }

}

void reverseLine() // move reverse with line sensor
{
  // reading line sensor back
  digitalWrite(serialEnB, LOW);
  while ( Serial3.available() <= 0);
  sensorPositionB = Serial3.read();
  digitalWrite(serialEnB, HIGH);

  if (sensorPositionB == 255 ) // if no line detected
  {
    motor.control(0, 0);
  }
  else
  {
    // PID calculation
    error = sensorPositionB - sensorSetPoint; // calculating error
    motorSpeed = kp * (error) + kd * (error - lasterror); // calculate the pid
    lasterror = error; // update the last error

    // Adjust speed based on the line sensor
    motorRight = baseSpeed - motorSpeed;
    motorLeft = baseSpeed + motorSpeed;

    // Adjust speed if maximum
    if (motorRight > maxSpeed)
      motorRight = maxSpeed;
    if (motorLeft > maxSpeed)
      motorLeft = maxSpeed;

    // If the motor is reverse (negative)
    if (motorRight < 0)
      motorRight = 0;
    if (motorLeft < 0)
      motorLeft = 0;

    motor.control(motorRight * -1, motorLeft * -1); // left, right
  }
}

// -------------------------- junction -------------------------- //
// read junction counter
int getJunctionA() // front
{
  char address = 0x01;
  char command = 'X';
  char data = 0x01;
  char checksum = address + command + data;

  Serial2.write(address);
  Serial2.write(command);
  Serial2.write(data);
  Serial2.write(checksum);

  while (Serial2.available() <= 0);
  return (int(Serial2.read()));
}

int getJunctionB() // back
{
  char address = 0x01;
  char command = 'X';
  char data = 0x01;
  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);

  while (Serial3.available() <= 0);
  return (int(Serial3.read()));
}


// reset juncton counter
void clearJunctionA() // front
{
  char address = 0x01;
  char command = 'X';
  char data = 0x00;
  char checksum = address + command + data;

  Serial2.write(address);
  Serial2.write(command);
  Serial2.write(data);
  Serial2.write(checksum);
}

void clearJunctionB() // back
{
  char address = 0x01;
  char command = 'X';
  char data = 0x00;
  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);
}

// -------------------------- junction 2 -------------------------- //

void clearAllJunction()
{
  flagA = 0;
  clearJunctionA();
  clearJunctionB();
  delay(100);

  junctionA = 0;
  junctionB = 0;

  // update task
  task ++;
}
