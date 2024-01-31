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
AccelStepper LeftBackWheel(1, 3, 6);   // Stepper1 (x,step, dir)
AccelStepper LeftFrontWheel(1, 2, 5);  // Stepper2 (x,step, dir)
AccelStepper RightBackWheel(1, 10, 9); // Stepper3 (x,step, dir) (1, 44, 45)
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

double wheel_distance_width;
double wheel_distance_length;
double wheel_separation_width=wheel_distance_width/2;
double wheel_separation_length=wheel_distance_length/2;
double wheel_radius;

void setMovement(int linearX, int linearY, int angularZ)
{
  double lf_wheel_speed = (1/wheel_radius) * (linearX - linearY - (wheel_separation_width+wheel_separation_length) * angularZ) * wheelSpeed; // * rampSpeed
  double rf_wheel_speed = (1/wheel_radius) * (linearX - linearY - (wheel_separation_width+wheel_separation_length) * angularZ) * wheelSpeed; // * rampSpeed
  double lb_wheel_speed = (1/wheel_radius) * (linearX - linearY - (wheel_separation_width+wheel_separation_length) * angularZ) * wheelSpeed; // * rampSpeed
  double rb_wheel_speed = (1/wheel_radius) * (linearX - linearY - (wheel_separation_width+wheel_separation_length) * angularZ) * wheelSpeed; // * rampSpeed
  LeftFrontWheel.setSpeed(lf_wheel_speed);
  LeftBackWheel.setSpeed(lb_wheel_speed);
  RightFrontWheel.setSpeed(rf_wheel_speed);
  RightBackWheel.setSpeed(rb_wheel_speed);
}
void stopMovement(){
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

void setup(){
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

String getValue(String data, char separator, int index,String defaultValue)
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

void loop(){
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
    if (speedRamp.update() == 100) {
        rampSpeed = 0;
        if (inside) {
        ramainMovement = '0';
        inside = false;
        }
    } else {
        rampSpeed = speedRamp.update();
        inside = true;
    }
    String x="0.0";
    String y="0.0";
    String z="0.0";
    if(Serial1.available() > 0){
        String vector = Serial1.readStringUntil('\n');
        x = getValue(vector,';',0, "0.0");
        y = getValue(vector,';',1, "0.0");
        z = getValue(vector,';',2, "0.0");
        // Serial.println("x="+x+" y="+y+" z="+z);
    }
    
    if (commands > 1 && control) {
        secondRampChangeDir();
    }

    if (commands == 1 && !alreadyStarted) {
        Serial.println("Start Ramp");
        speedRamp.go(wheelSpeed, rampTime);
        alreadyStarted = true;
    }
    setMovement(x.toDouble(),y.toDouble(),z.toDouble());
}