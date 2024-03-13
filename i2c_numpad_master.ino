#include "motor.h"

const double distancePerEncCount = 4.385139;
#define I2C_SLAVE_ADDR 0x04 // Slave address
const double leftMotorDistancePerCount = 5000 / 1082;
const double rightMotorDistancePerCount = 5000 / 1099;

MotorController motorController(I2C_SLAVE_ADDR, -32);

void setup()
{
  Serial.begin(9600);

  motorController.setEncodeDirection(1, -1, true);
  motorController.setMotorDirection(-1, 1, false);
  motorController.setDistancePerEncCount(leftMotorDistancePerCount, rightMotorDistancePerCount);

  motorController.setMotorSteer();
  // motorController.moveForward(100);
}

void loop()
{
  static int el = 0;
  static int er = 0;
  motorController.getEncoderValues(&el, &er);
  Serial.printf("El: %d\t", el);
  Serial.printf("Er: %d\n", er);
  delay(500);
}