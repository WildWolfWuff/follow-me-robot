#include "InvertedAccelStepper.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Ramp.h>

#define MAX_SPEED 10000  // 10000 steps per second, reasonable driving speed
#define WHEEL_SPEED 1    // 1.00 * linX, linY & angZ = 300% of MAX_SPEED on a single wheel, not reachable from driving strictly forward
#define WHEEL_RADIUS 7.5 // who cares
#define BAUD 9600

InvertedAccelStepper LeftFrontWheel(1, 2, 5); // Stepper2 (x,step, dir)
InvertedAccelStepper LeftRearWheel(1, 3, 6);  // Stepper1 (x,step, dir)

AccelStepper RightFrontWheel(1, 7, 8); // Stepper4 (x,step, dir) (1, 46, 47)
AccelStepper RightRearWheel(1, 10, 9); // Stepper3 (x,step, dir) (1, 44, 45)

// input vector linear x,y and angular z
String x = "0.0";
String y = "0.0";
String z = "0.0";

float left_front_wheel_speed;
float left_rear_wheel_speed;
float right_front_wheel_speed;
float right_rear_wheel_speed;

// Input Values in Percent, output should max out at 300 @ 100% WHEEL_SPEED
// Max Output is 100% of MAX_SPEED when only one driving in one direction
void setWheelSpeedValues(double speedX, double speedY, double speedZ)
{
  Serial.println("x: " + String(speedX) + " y: " + String(speedY) + " z: " + String(speedZ));
  left_front_wheel_speed = (speedX + speedY + speedZ) * WHEEL_SPEED;
  left_rear_wheel_speed = (speedX - speedY + speedZ) * WHEEL_SPEED;
  right_front_wheel_speed = (speedX - speedY - speedZ) * WHEEL_SPEED;
  right_rear_wheel_speed = (speedX + speedY - speedZ) * WHEEL_SPEED;
}

void move()
{
  const float tolerance = 0.0001; // Define a small tolerance value
  if (fabs(left_front_wheel_speed - LeftFrontWheel.speed()) > tolerance)
  {
    LeftFrontWheel.setSpeed(left_front_wheel_speed * MAX_SPEED / 100);
    Serial.println("Left Front Wheel: " + String(left_front_wheel_speed));
  }
  if (fabs(left_rear_wheel_speed - LeftRearWheel.speed()) > tolerance)
    Serial.println("Left Rear Wheel: " + String(left_rear_wheel_speed));
  {
    LeftRearWheel.setSpeed(left_rear_wheel_speed * MAX_SPEED / 100);
  }
  if (fabs(right_front_wheel_speed - RightFrontWheel.speed()) > tolerance)
  {
    RightFrontWheel.setSpeed(right_front_wheel_speed * MAX_SPEED / 100);
    Serial.println("Right Front Wheel: " + String(right_front_wheel_speed));
  }
  if (fabs(right_rear_wheel_speed - RightRearWheel.speed()) > tolerance)
  {
    RightRearWheel.setSpeed(right_rear_wheel_speed * MAX_SPEED / 100);
    Serial.println("Right Rear Wheel: " + String(right_rear_wheel_speed));
  }

  LeftFrontWheel.runSpeed();
  LeftRearWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightRearWheel.runSpeed();
}

void setup()
{
  // Max Speed for Stepper Motors
  // three times MAX_SPEED, because of the 3x speed loss in the setWheelSpeedValues function
  LeftFrontWheel.setMaxSpeed(MAX_SPEED * 3);
  LeftRearWheel.setMaxSpeed(MAX_SPEED * 3);
  RightFrontWheel.setMaxSpeed(MAX_SPEED * 3);
  RightRearWheel.setMaxSpeed(MAX_SPEED * 3);

  Serial.begin(BAUD);
  Serial1.begin(BAUD);

  setWheelSpeedValues(0, 0, 0);

  Serial.println("ready");
}

void loop()
{
  if (Serial.available() > 0)
  {
    String json = Serial.readStringUntil('\n');
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, json);

    if (!error)
    {
      float x = doc["x"];
      float y = doc["y"];
      float z = doc["z"];
      setWheelSpeedValues(x, y, z);
    }
    else
    {
      setWheelSpeedValues(0, 0, 0);
      Serial.println("Failed to parse JSON");
    }
  } else {
    setWheelSpeedValues(0, 0, 0);
  }
  move();
}