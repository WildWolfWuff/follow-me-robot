#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <Ramp.h>

/*
 * Object Ramp
 * Documentation regarding rhe Ramp.h
 * https://github.com/siteswapjuggler/RAMP
 */
rampInt speedRamp;
//Object Bluetooth (RX,TX)
//SoftwareSerial Bluetooth(A8, 38);
/*
 * Object Stepper
 * Documentation regarding the AccelStepper
 * http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a73bdecf1273d98d8c5fbcb764cabeea5ac3523e4cf6763ba518d16fec3708ef23
 */
AccelStepper LeftBackWheel(1, 3, 6);     // Stepper1 (x,step, dir)
AccelStepper LeftFrontWheel(1, 2, 5);    // Stepper2 (x,step, dir)
AccelStepper RightBackWheel(1, 10, 9);  // Stepper3 (x,step, dir) (1, 44, 45)
AccelStepper RightFrontWheel(1, 7, 8); // Stepper4 (x,step, dir) (1, 46, 47)

bool control = false;
bool startTestCode = false;
bool alreadyStarted = false;
bool testWithoutRamp = false;
bool inside = true;
int state = 0;
int testControl = 0;
int minRampVal = 105;
int steps = 0;
int commands = 0;
int wheelSpeed = 500; //Initial speed value from 0 //previous value is 255
int rampSpeed = 0;
int minRampTime = 500;
int rampTime = 2000; //1 second and 500 ms ramp as default
char readSerial = 0;
char ramainMovement = 's';
unsigned long myTime;
unsigned long newTimer;
unsigned long generalTime;

/*
 * Movement Functions
 * Diferent combinations of the mecanum wheels.
 */
void moveForward() {
  LeftFrontWheel.setSpeed(-rampSpeed);
  LeftBackWheel.setSpeed(-rampSpeed);
  RightFrontWheel.setSpeed(rampSpeed);
  RightBackWheel.setSpeed(rampSpeed);
}
void moveBackward() {
  LeftFrontWheel.setSpeed(rampSpeed);
  LeftBackWheel.setSpeed(rampSpeed);
  RightFrontWheel.setSpeed(-rampSpeed);
  RightBackWheel.setSpeed(-rampSpeed);
}
void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(-rampSpeed);
  LeftBackWheel.setSpeed(rampSpeed);
  RightFrontWheel.setSpeed(-rampSpeed);
  RightBackWheel.setSpeed(rampSpeed);
}
void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(rampSpeed);
  LeftBackWheel.setSpeed(-rampSpeed);
  RightFrontWheel.setSpeed(rampSpeed);
  RightBackWheel.setSpeed(-rampSpeed);
}
void rotateLeft() {
  LeftFrontWheel.setSpeed(rampSpeed);
  LeftBackWheel.setSpeed(rampSpeed);
  RightFrontWheel.setSpeed(rampSpeed);
  RightBackWheel.setSpeed(rampSpeed);
}
void rotateRight() {
  LeftFrontWheel.setSpeed(-rampSpeed);
  LeftBackWheel.setSpeed(-rampSpeed);
  RightFrontWheel.setSpeed(-rampSpeed);
  RightBackWheel.setSpeed(-rampSpeed);
}
void moveRightForward() {
  LeftFrontWheel.setSpeed(-rampSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(rampSpeed);
}
void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(rampSpeed);
  RightFrontWheel.setSpeed(-rampSpeed);
  RightBackWheel.setSpeed(0);
}
void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-rampSpeed);
  RightFrontWheel.setSpeed(rampSpeed);
  RightBackWheel.setSpeed(0);
}
void moveLeftBackward() {
  LeftFrontWheel.setSpeed(rampSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-rampSpeed);
}
void stopMoving() {
  //add cases just work forward
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
}

void secondRampChangeDir() {
  if (speedRamp.update() > 0 && steps == 0) {
    Serial.println("Ramp change dir step 1");
    speedRamp.go(minRampVal, 500);
    steps = 1;
  }
  if (speedRamp.update() == minRampVal && steps == 1 ) {
    //change to new direction
    speedRamp.go(wheelSpeed, rampTime);
    steps = 0;
    control = false;
  }
}

void setup() {
  // Declaration from Stepper Motor Speed
  LeftBackWheel.setMaxSpeed(1000);
  LeftFrontWheel.setMaxSpeed(1000);
  RightBackWheel.setMaxSpeed(1000);
  RightFrontWheel.setMaxSpeed(1000);
  /*
   * Setup of Serial Ports
   * Serial.begin(9600) -> Main Serial where we will recive the error or information in case we have
   * Bluetooth.begin(9600) -> We start the BLE Port where is connected the BLE Module
   * 
   */
  Serial.begin(9600);
  Serial1.begin(9600);
  // Bluetooth.begin(9600);
  /*
   * Start the SpeedRamp as a initial value of 100
   */
  speedRamp.go(100);
}

void loop() {
  //Main code:
  /*
   * This Functions need also to be in the main code to run every time
   * RightBackWheel.runSpeed();
   * RightFrontWheel.runSpeed();
   * LeftBackWheel.runSpeed();
   * LeftFrontWheel.runSpeed();
   */
  RightBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  LeftFrontWheel.runSpeed();
  /*
   * generalTime is a timer we use in the situation we want to use the test code. The test code are controlled by timers,
   * to make it work we need to send the correct commands via a BLE device could be a cellphone or computer. 
   */
  generalTime = millis();
  /*
   * SpeedRamp use the function update() and this should be at the main loop to change
   */
  speedRamp.update();
  /*
   * If the minimum value of the speed ramp is 100 then the speed of the motor is 0
   * after the value increse more than 100 the motor will work without waiting
   * in case we would like to start the motor with 0 it will take some time to start and as
   * we could se the motor reacted correctly at a minimum value of 100
   */
  if (speedRamp.update() == 100) {
    rampSpeed = 0;
    if (inside) {
      ramainMovement = 's';
      inside = false;
    }
  } else {
    rampSpeed = speedRamp.update();
    inside = true;
  }

  if (Serial1.available() > 0) // any character available
  {
    readSerial = Serial1.read();
    //Serial.println(readSerial); // echo...
  } // if not (+ or -) set command sign back to 0 otherwise remember + or -
  else if (!( (readSerial == '+') || (readSerial == '-')))
    readSerial = 0;

  /*
   * If the commands are more than 0 is going to try to reduce the speed of the motors and after that is going 
   * to return to the original speed value that was already set
   */
  if (commands > 1 && control) {
    secondRampChangeDir();
  }
  if (commands == 1 && !alreadyStarted) {
    Serial.println("Start Ramp");
    speedRamp.go(wheelSpeed, rampTime);
    alreadyStarted = true;
  }

  switch (ramainMovement) {
    case '0':
      moveLeftForward();
      break;
    case '1':
      moveForward();
      break;
    case '2':
      moveRightForward();
      break;
    case '3':
      moveSidewaysLeft();
      break;
    case '4':
      moveSidewaysRight();
      break;
    case '5':
      moveLeftBackward();
      break;
    case '6':
      moveBackward();
      break;
    case '7':
      moveRightBackward();
      break;
    case '8':
      rotateLeft();
      break;
    case '9':
      rotateRight();
      break;
    case 's':
      stopMoving();
      break;
  }

  /*
   * This option is to test the movement of the Exam Robot using a Ramp
   */
  if (startTestCode) { //With current Ramps
    if (testControl == 0 && (generalTime - myTime) >= 3000) { //forward
      readSerial = 'B';
      testControl = 1;
      myTime = millis();
      Serial.println("mande");
    }
    if (testControl == 1 && (generalTime - myTime) >= 3000) { //Stop
      readSerial = '0';
      testControl = 2;
      Serial.println(testControl);
      myTime = millis();
    }
    if (testControl == 2 && (generalTime - myTime) >= 3000) { //Right Side
      readSerial = 'E';
      myTime = millis();
      testControl = 3;
      Serial.println(testControl);
    }
    if (testControl == 3 && (generalTime - myTime) >= 3000) { //Stop
      readSerial = '0';
      myTime = millis();
      testControl = 4;
      Serial.println(testControl);
    }
    if (testControl == 4 && (generalTime - myTime) >= 3000) { //backward
      readSerial = 'G';
      myTime = millis();
      testControl = 5;
      Serial.println(testControl);
    }
    if (testControl == 5 && (generalTime - myTime) >= 3000) {//Stop
      readSerial = '0';
      myTime = millis();
      testControl = 6;
      Serial.println(testControl);
    }
    if (testControl == 6 && (generalTime - myTime) >= 3000) { //left
      readSerial = 'D';
      myTime = millis();
      testControl = 7;
      Serial.println(testControl);
    }
    if (testControl == 7 && (generalTime - myTime) >= 3000) { // Stop Code
      readSerial = '0';
      testControl = 0;
      myTime = millis();
      startTestCode = false;
      Serial.println(testControl);
    }
  }

  /*
   * This are the diferent input command that is possible to recive wirth a BLE communication
   */
  switch (readSerial) {
    case 'A':
      Serial.println("Left Forward");
      commands++;
      ramainMovement = '0';
      control = true;
      break;
    case 'B':
      Serial.println("Move Forward");
      commands++;
      ramainMovement = '1';
      control = true;
      break;
    case 'C':
      Serial.println("Right Forward");
      commands++;
      ramainMovement = '2';
      control = true;
      break;
    case 'D':
      Serial.println("Sideway Left");
      commands++;
      ramainMovement = '3';
      control = true;
      break;
    case 'E':
      Serial.println("Sideway Right");
      commands++;
      ramainMovement = '4';
      control = true;
      break;
    case 'F':
      Serial.println("Left Backward");
      commands++;
      ramainMovement = '5';
      control = true;
      break;
    case 'G':
      Serial.println("Move Backward");
      commands++;
      ramainMovement = '6';
      control = true;
      break;
    case 'H':
      Serial.println("Right Backward");
      commands++;
      ramainMovement = '7';
      control = true;
      break;
    case 'I':
      Serial.println("Rotate Left");
      commands++;
      ramainMovement = '8';
      control = true;
      break;
    case 'J':
      Serial.println("Rotate Right");
      commands++;
      ramainMovement = '9';
      control = true;
      break;
    case '0': //Stop gradualy all the movement of all the wheels
      Serial.println(" Stop Moving");
      control = false;
      alreadyStarted = false;
      commands = 0;
      speedRamp.go(100, 700); //original speedRamp.go(100, 1000);
      break;
    case 'S': //Set a new Speed to the motors, the maximum value is already set in the app
      Serial.print("New Speed:");
      wheelSpeed = Serial1.parseInt();
      Serial.println(wheelSpeed);
      //create a ramp with the new incoming value for the speed Max 250 and the ramp is will take 1,5 Sec
      Serial.println("Start new Ramp");
      speedRamp.go(wheelSpeed, rampTime);
      break;
    case 'R': //Set the new Ramp Time (Function no tested)
      Serial.print("New Ramp Time:");
      rampTime = Serial1.parseInt();
      if (rampTime < minRampTime) {
        rampTime = minRampTime;
      }
      Serial.println(rampTime);
      break;
    case 'T': //Test method run for one time using the ramps
      startTestCode = true;
      myTime = millis();
      break;
    case 'Z': //Test Code without Ramp
      speedRamp.go(wheelSpeed, rampTime);
      testWithoutRamp = true;
      newTimer = millis();
      break;
  }

  /*
   * This is a option to test the movement of the omniwheels without using any kind of ramp
   */
  if (testWithoutRamp) {
    if (state == 0 && (generalTime - newTimer) >= 2000) {
      ramainMovement = '1';
      newTimer = millis();
      state = 1;
    }
    if (state == 1 && (generalTime - newTimer) >= 3000) {
      ramainMovement = 's';
      state = 2;
      newTimer = millis();
    }
    if (state == 2 && (generalTime - newTimer) >= 3000) {
      ramainMovement = '4';
      state = 3;
      newTimer = millis();
    }
    if (state == 3 && (generalTime - newTimer) >= 3000) {
      ramainMovement = 's';
      state = 4;
      newTimer = millis();
    }
    if (state == 4 && (generalTime - newTimer) >= 3000) {
      ramainMovement = '6';
      state = 5;
      newTimer = millis();
    }
    if (state == 5 && (generalTime - newTimer) >= 3000) {
      ramainMovement = 's';
      state = 6;
      newTimer = millis();
    }
    if (state == 6 && (generalTime - newTimer) >= 3000) {
      ramainMovement = '3';
      state = 7;
      newTimer = millis();
    }
    if (state == 7 && (generalTime - newTimer) >= 3000) {
      ramainMovement = 's';
      newTimer = millis();
      testWithoutRamp = false;
      state = 0;

    }
  }
}
