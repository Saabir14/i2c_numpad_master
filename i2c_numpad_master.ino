#include "motor.h"

const double distancePerEncCount = 4.385139;
#define I2C_SLAVE_ADDR 0x04 // Slave address
const double leftMotorDistancePerCount = 5000 / 1082;
const double rightMotorDistancePerCount = 5000 / 1099;

MotorController motorController(I2C_SLAVE_ADDR, -32);

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  motorController.setEncodeDirection(1, -1, true);
  motorController.setMotorDirection(-1, 1, false);
  motorController.setDistancePerEncCount(leftMotorDistancePerCount, rightMotorDistancePerCount);

  // motorController.setMotorSteer();
  motorController.moveForward(500);
}

void loop()
{
  int encL, encR;
  motorController.getEncoderCount(&encL, &encR);
  Serial.print("Left: ");
  Serial.print(encL);
  Serial.print("\tRight: ");
  Serial.println(encR);
  delay(1000);
}