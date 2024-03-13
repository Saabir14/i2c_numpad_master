#include "motor.h"

#define I2C_SLAVE_ADDR 0x04 // Slave address

const double distancePerEncCount = 4.385139;
const double leftMotorDistancePerCount = 1082 / 5000;
const double rightMotorDistancePerCount = 1099 / 5000;

MotorController motorController(I2C_SLAVE_ADDR, -1500);

int el = 0;
int er = 0;

void setup()
{
  Serial.begin(9600);
  motorController.setEncodeDirection(1, -1, true);
  motorController.setMotorDirection(-1, 1, false);
  motorController.setDistancePerEncCount(leftMotorDistancePerCount, rightMotorDistancePerCount);
}

void loop()
{
  motorController.setMotorSteer(0, 0, 0);
  motorController.getEncoderValues(&el, &er);
  Serial.printf("El: %d\t", el);
  Serial.printf("Er: %d\n", er);
  Serial.printf("offset: %d\n", motorController.offset);
  delay(500);
}