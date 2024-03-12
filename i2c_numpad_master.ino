#include "motor.h"

#define I2C_SLAVE_ADDR 0x04 // Slave address

const double distancePerEncCount = 4.385139;
MotorController motorController(I2C_SLAVE_ADDR);

int el = 0;
int er = 0;

void setup()
{
  Serial.begin(9600);
  // motorController.setEncodeDirection(1, -1, true);
  // motorController.setDistancePerEncCount(4.385139);

  motorController.setMotorSteer(0, 255, 255);
  // motorController.moveForward(100);
}

void loop()
{
  motorController.getEncoderValues(&el, &er);
  Serial.printf("El: %d\t", el);
  Serial.printf("Er: %d\n", er);
  delay(250);
}