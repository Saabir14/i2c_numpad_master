#include "motor.h"

const double distancePerEncCount = 4.385139;
#define I2C_SLAVE_ADDR 0x04 // Slave address
const double leftMotorDistancePerCount = 5000 / 1082;
const double rightMotorDistancePerCount = 5000 / 1099;

const int wheelBase = 147;
MotorController motorController(I2C_SLAVE_ADDR, -32);

const double turnFrictionCorrection = 1.15;
const int lowTurnSpeed = 160, highTurnSpeed = 180;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  motorController.setEncodeDirection(1, -1, true);
  motorController.setMotorDirection(-1, 1, false);
  motorController.setDistancePerEncCount(leftMotorDistancePerCount, rightMotorDistancePerCount);
  motorController.setWheelBase(wheelBase);

  // motorController.setMotorSteer();
  // motorController.moveForward(500);
  motorController.turnRad(PI, highTurnSpeed, lowTurnSpeed, turnFrictionCorrection);
}

void loop() {}