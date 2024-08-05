#include "InvertedAccelStepper.h"
#include <ArduinoJson.h>

#define MAX_SPEED 30000  // 30000 steps per second
#define WHEEL_RADIUS 7.5 // who cares
#define BAUD 9600

InvertedAccelStepper LeftFrontWheel(1, 2, 5); // Stepper2 (x,step, dir)
InvertedAccelStepper LeftRearWheel(1, 3, 6);  // Stepper1 (x,step, dir)

AccelStepper RightFrontWheel(1, 7, 8); // Stepper4 (x,step, dir) (1, 46, 47)
AccelStepper RightRearWheel(1, 10, 9); // Stepper3 (x,step, dir) (1, 44, 45)

// input vector linear x,y and angular z
float x = 0.0;
float y = 0.0;
float z = 0.0;
const float tolerance = 0.0001;

float left_front_wheel_speed;
float left_rear_wheel_speed;
float right_front_wheel_speed;
float right_rear_wheel_speed;

// Input Values in Percent, output should max out at 100 (%) per wheel
void setWheelSpeedValues(float speedX, float speedY, float speedZ)
{
  left_front_wheel_speed = (speedX + speedY + speedZ) / 3;
  left_rear_wheel_speed = (speedX - speedY + speedZ) / 3;
  right_front_wheel_speed = (speedX - speedY - speedZ) / 3;
  right_rear_wheel_speed = (speedX + speedY - speedZ) / 3;
}

void move()
{
  if (abs(left_front_wheel_speed * MAX_SPEED / 100 - LeftFrontWheel.speed()) > tolerance)
  {
    LeftFrontWheel.setSpeed(left_front_wheel_speed * MAX_SPEED / 100);
    Serial.println("Left Front Wheel: " + String(left_front_wheel_speed) + "%");
  }
  if (abs(left_rear_wheel_speed * MAX_SPEED / 100 - LeftRearWheel.speed()) > tolerance)
  {
    LeftRearWheel.setSpeed(left_rear_wheel_speed * MAX_SPEED / 100);
    Serial.println("Left Rear Wheel: " + String(left_rear_wheel_speed) + "%");
  }
  if (abs(right_front_wheel_speed * MAX_SPEED / 100 - RightFrontWheel.speed()) > tolerance)
  {
    RightFrontWheel.setSpeed(right_front_wheel_speed * MAX_SPEED / 100);
    Serial.println("Right Front Wheel: " + String(right_front_wheel_speed) + "%");
  }
  if (abs(right_rear_wheel_speed * MAX_SPEED / 100 - RightRearWheel.speed()) > tolerance)
  {
    RightRearWheel.setSpeed(right_rear_wheel_speed * MAX_SPEED / 100);
    Serial.println("Right Rear Wheel: " + String(right_rear_wheel_speed) + "%");
  }

  LeftFrontWheel.runSpeed();
  LeftRearWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightRearWheel.runSpeed();
}

void setup()
{
  // Max Speed for Stepper Motors
  LeftFrontWheel.setMaxSpeed(MAX_SPEED);
  LeftRearWheel.setMaxSpeed(MAX_SPEED);
  RightFrontWheel.setMaxSpeed(MAX_SPEED);
  RightRearWheel.setMaxSpeed(MAX_SPEED);

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
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (!error)
    {
      float old_x = x;
      float old_y = y;
      float old_z = z;
      float x = doc["x"];
      float y = doc["y"];
      float z = doc["z"];
      if (old_x - x > tolerance || old_y - y > tolerance || old_z - z > tolerance)
        Serial.println("x: " + String(x) + " y: " + String(y) + " z: " + String(z));
      setWheelSpeedValues(x, y, z);
    }
    else
    {
      Serial.println("Failed to parse JSON");
      if (json == "stop")
      {
        setWheelSpeedValues(0, 0, 0);
      }
    }
  }
  move();
}