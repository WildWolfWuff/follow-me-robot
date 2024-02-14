#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Ramp.h>
#define TIMEOUT_DURATION 1000
#define MAX_SPEED 5000
#define STOP_MOVE stop()
//setMovement(0, 0, 0)
/*
 * Object Ramp
 * Documentation regarding rhe Ramp.h
 * https://github.com/siteswapjuggler/RAMP
 */
// rampInt speedRamp;

// Object Bluetooth (RX,TX)
// SoftwareSerial Bluetooth(A8, 38);
/*
 * Object Stepper
 * Documentation regarding the AccelStepper
 * http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a73bdecf1273d98d8c5fbcb764cabeea5ac3523e4cf6763ba518d16fec3708ef23
 */
AccelStepper LeftBackWheel(1, 3, 6);   // Stepper1 (x,step, dir)
AccelStepper LeftFrontWheel(1, 2, 5);  // Stepper2 (x,step, dir)
AccelStepper RightBackWheel(1, 10, 9); // Stepper3 (x,step, dir) (1, 44, 45)
AccelStepper RightFrontWheel(1, 7, 8); // Stepper4 (x,step, dir) (1, 46, 47)

void logLn(String text){
  Serial.println(text);
}
void log(String text){
  Serial.print(text);
}

// input vector linear x,y and angular z
String x="0.0";
String y="0.0";
String z="0.0";
int wheelSpeed = 5000; //Initial speed value from 0 //previous value is 255

double wheel_distance_width=20;
double wheel_distance_length=21;
double wheel_separation_width = wheel_distance_width/2;
double wheel_separation_length = wheel_distance_length/2;
double wheel_radius=7.5;

rampFloat lf_wheel_speed;
rampFloat rf_wheel_speed;
rampFloat lb_wheel_speed;
rampFloat rb_wheel_speed;

float tolerance = 1e-9;
unsigned long transitionMs=500;

void setMovement(double linearX, double linearY, double angularZ)
{
  // caluclates the movement of each wheel
  // see http://robotsforroboticists.com/drive-kinematics/ for more informations
  lf_wheel_speed.go(-((1/wheel_radius) * (linearX - linearY - angularZ) * wheelSpeed),transitionMs,LINEAR);
  rf_wheel_speed.go((1/wheel_radius) * (linearX + linearY + angularZ) * wheelSpeed,transitionMs,LINEAR);
  lb_wheel_speed.go(- ((1/wheel_radius) * (linearX + linearY - angularZ) * wheelSpeed),transitionMs,LINEAR);
  rb_wheel_speed.go((1/wheel_radius) * (linearX - linearY + angularZ) * wheelSpeed,transitionMs,LINEAR);
  // sets the movement to the wheels
  logLn(">x:"+String(linearX)+"\n>y:"+String(linearY)+"\n>z:"+String(angularZ));
}

bool equal(float a, float b) {
  return fabs(a - b) < tolerance;
}

void stop(){
  lf_wheel_speed.go(0,0);
  rf_wheel_speed.go(0,0);
  lb_wheel_speed.go(0,0);
  rb_wheel_speed.go(0,0);
}

int baud=9600;

void move(){
  float f=lf_wheel_speed.update();
  if(!equal(LeftFrontWheel.speed(), f)){
    LeftFrontWheel.setSpeed(f);
    logLn(">lf:"+String(f));
  }
  f=rf_wheel_speed.update();
  if(!equal(RightFrontWheel.speed(), f)){
    RightFrontWheel.setSpeed(f);
    logLn(">rf:"+String(f));
  }
  f=lb_wheel_speed.update();
  if(!equal(LeftBackWheel.speed(), f)){
    LeftBackWheel.setSpeed(f);
  logLn(">lb:"+String(f));
  }
  f=rb_wheel_speed.update();
  if(!equal(RightBackWheel.speed(), f)){
    RightBackWheel.setSpeed(f);
    logLn(">rb:"+String(f));
  }
}

void setup(){
    // Declaration from Stepper Motor Speed
    LeftBackWheel.setMaxSpeed(MAX_SPEED);
    LeftFrontWheel.setMaxSpeed(MAX_SPEED);
    RightBackWheel.setMaxSpeed(MAX_SPEED);
    RightFrontWheel.setMaxSpeed(MAX_SPEED);
    /*
    * Setup of Serial Ports
    * Serial.begin(9600) -> Main Serial where we will recive the error or information in case we have
    * Bluetooth.begin(9600) -> We start the BLE Port where is connected the BLE Module
    * 
    */

    Serial.begin(baud);
    Serial1.begin(baud);
    // Serial2.begint(baud);
    // Bluetooth.begin(9600);
    /*
    * Start the SpeedRamp as a initial value of 100
    */
    lf_wheel_speed.go(0);
    rf_wheel_speed.go(0);
    lb_wheel_speed.go(0);
    rb_wheel_speed.go(0);
    logLn("ready");
}

// Gets the sub string from a string based on a separator char and its index.
// If index not exists it retuns a default value
String getValue(String data, char separator, int index, String defaultValue)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : defaultValue;
}

// logs a string to a Serial output for monitor the system

char separator=';';

void loop(){
  static unsigned long lastDataTime = 0;
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
    * Recives the movement values from seral input.
    * The recived string is split by ';' to get x,y and z values.
    * This cordinates will be given in the set move function
    */
  String vectorData = "";
  if(Serial.available() > 0){
    vectorData = Serial.readStringUntil('\n');
  }
  else if(Serial1.available() > 0){
    vectorData = Serial1.readStringUntil('\n');
  }
  if(vectorData.startsWith("+DISC:")){
    logLn("Connection lost");
    STOP_MOVE;
  }else if(vectorData != "") {
    x = getValue(vectorData,separator,0, "0.0");
    y = getValue(vectorData,separator,1, "0.0");
    z = getValue(vectorData,separator,2, "0.0");
    setMovement(x.toDouble(),y.toDouble(),z.toDouble());
  }
  // TODO add speed regulation
  move();
}