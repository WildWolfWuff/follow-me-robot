#include "InvertedAccelStepper.h"

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

float old_x = x;
float old_y = y;
float old_z = z;

const float tolerance = 0.0001;

float left_front_wheel_speed;
float left_rear_wheel_speed;
float right_front_wheel_speed;
float right_rear_wheel_speed;

/**
 * @brief Sets the speed values for the mecanum wheels.
 * 
 * This function takes in three input values representing the desired speed in the X, Y, and Z directions.
 * The input values are in percent, where 100% represents the maximum speed for each wheel.
 * The function calculates the speed values for each wheel based on the input values and sets the corresponding variables.
 * 
 * @param speedX The desired speed in the X direction.
 * @param speedY The desired speed in the Y direction.
 * @param speedZ The desired speed in the Z direction.
 *  @note Input Values in Percent, output should max out at 1(00%) per wheel
 */
void setWheelSpeedValues(float speedX, float speedY, float speedZ)
{
  left_front_wheel_speed = (speedX + speedY + speedZ) / 3;
  left_rear_wheel_speed = (speedX - speedY + speedZ) / 3;
  right_front_wheel_speed = (speedX - speedY - speedZ) / 3;
  right_rear_wheel_speed = (speedX + speedY - speedZ) / 3;
}

/**
 * @brief Moves the mecanum wheels based on the specified speeds.
 * 
 * This function adjusts the speed of each wheel based on the specified speeds for the left front, left rear, right front, and right rear wheels.
 * If the difference between the desired speed and the current speed of a wheel is greater than the tolerance value, the speed of that wheel is adjusted.
 * The adjusted speed is calculated by multiplying the desired speed by the maximum speed.
 * 
 * @note This function assumes that the `LeftFrontWheel`, `LeftRearWheel`, `RightFrontWheel`, and `RightRearWheel` objects have been properly initialized.
 * 
 * @note The desired speeds for each wheel should be in the range of -1.0 to 1.0, where -1.0 represents full reverse, 0.0 represents stop, and 1.0 represents full forward.
 * 
 * @note The tolerance value determines the maximum allowable difference between the desired speed and the current speed of a wheel.
 * 
 * @note After adjusting the speeds of all the wheels, the `runSpeed()` function is called for each wheel to apply the adjusted speeds.
 */
void move()
{
  if (abs(left_front_wheel_speed * MAX_SPEED - LeftFrontWheel.speed()) > tolerance)
  {
    LeftFrontWheel.setSpeed(left_front_wheel_speed * MAX_SPEED);
    // Serial.println("Left Front Wheel: " + String(left_front_wheel_speed*100) + "%");
  }
  if (abs(left_rear_wheel_speed * MAX_SPEED - LeftRearWheel.speed()) > tolerance)
  {
    LeftRearWheel.setSpeed(left_rear_wheel_speed * MAX_SPEED);
    // Serial.println("Left Rear Wheel: " + String(left_rear_wheel_speed*100) + "%");
  }
  if (abs(right_front_wheel_speed * MAX_SPEED - RightFrontWheel.speed()) > tolerance)
  {
    RightFrontWheel.setSpeed(right_front_wheel_speed * MAX_SPEED);
    // Serial.println("Right Front Wheel: " + String(right_front_wheel_speed*100) + "%");
  }
  if (abs(right_rear_wheel_speed * MAX_SPEED - RightRearWheel.speed()) > tolerance)
  {
    RightRearWheel.setSpeed(right_rear_wheel_speed * MAX_SPEED);
    // Serial.println("Right Rear Wheel: " + String(right_rear_wheel_speed*100) + "%");
  }

  LeftFrontWheel.runSpeed();
  LeftRearWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightRearWheel.runSpeed();
}

/**
 * @brief Reads serial input and updates the values of x, y, and z.
 * 
 * This function reads a line of serial input and parses it into three float values.
 * The values are separated by semicolons (;) and are expected to be in the format: x;y;z.
 * The parsed values are then assigned to the variables x, y, and z respectively.
 * 
 * Note: The function assumes that the serial input is terminated with a newline character ('\n').
 * 
 * @param None
 * @return None
 */
void readSerial()
{
  old_x = x;
  old_y = y;
  old_z = z;

  const int bufferSize = 50;
  char buffer[bufferSize];
  int bytesRead = Serial.readBytesUntil('\n', buffer, bufferSize - 1);
  buffer[bytesRead] = '\0';

  float values[3] = {0.0, 0.0, 0.0};
  char *token = strtok(buffer, ";");
  int index = 0;

  while (token != NULL && index < 3)
  {
    values[index] = atof(token);
    token = strtok(NULL, ";");
    index++;
  }

  x = values[0];
  y = values[1];
  z = values[2];
}

/**
 * @brief Initializes the motor controller and sets up the necessary configurations.
 * 
 * This function sets the maximum speed for the stepper motors and initializes the serial communication.
 * It also sets the initial wheel speed values to zero and prints a message to indicate that the motor controller is ready.
 */
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

/**
 * @brief The main loop of the program.
 * 
 * This function is called repeatedly in the program. It checks if there is any data available
 * from the serial port and reads it. If the values of x, y, or z have changed beyond the tolerance
 * level, it sets the wheel speed values accordingly. Finally, it moves the robot.
 */
void loop()
{
  if (Serial.available() > 0)
  {
    readSerial();

    if (old_x - x > tolerance || old_y - y > tolerance || old_z - z > tolerance)
      // Serial.println("x: " + String(x) + " y: " + String(y) + " z: " + String(z));
    setWheelSpeedValues(x, y, z);
  }
  move();
}