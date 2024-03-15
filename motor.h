// By Saabir Ahmed Parvaiz Ahmed (2024-3-14)
// This code is used to control the motors and read the encoder values of the car
#include <Wire.h>

/**
 * @class MotorController
 * @brief Represents a motor controller for controlling the movement of a car.
 */
class MotorController
{
public:
  // This is the address of the slave device
  int I2C_SLAVE_ADDR;
  int offset;
  int maxL;
  int maxR;

  double lMotorDirection = 1, rMotorDirection = 1;
  bool invertMotor = false;

  double lEncDirection = 1, rEncDirection = 1;
  bool invertEncoder = false;

  double lMotorDistancePerEncCount = 0, rMotorDistancePerEncCount = 0;

  double wheelBase = 147;

  /**
   * @brief Constructs a MotorController object.
   *
   * @param slaveAddr The I2C slave address of the motor driver.
   * @param offset The offset value for motor control. Default is 0.
   * @param maxL Maximum value for left turning of steering motor
   * @param maxR Maximum value for right turning of steering motor
   * @param personalAddress The I2C address of the motor controller. -2 for not specified, -1 for no Wire.begin().
   */
  MotorController(int slaveAddr, int offset = 0, int maxL = -75, int maxR = 75, int personalAddress = -2)
  {
    // Set up the I2C communication incase it is not setup in main code (so people don't have to in main code)
    if (personalAddress == -2)
      Wire.begin();
    else if (personalAddress != -1)
      Wire.begin(personalAddress);

    I2C_SLAVE_ADDR = slaveAddr;
    this->offset = offset;
    this->maxL = maxL;
    this->maxR = maxR;
  }

  /**
   * @brief Function to set the motor direction. Can be used to invert motor values if L and R value is not (1 or -1) for advanced uses
   *
   * Sets the direction of the motor.
   *
   * @param L The value for the left motor direction.
   * @param R The value for the right motor direction.
   * @param invert If true, swap the (L) and (R) motors. Default is false.
   */
  void setMotorDirection(double L, double R, bool invert = false)
  {
    lMotorDirection = L, rMotorDirection = R;
    invertMotor = invert;
  }

  /**
   * @brief Function to set the encoder direction. Can be used to scale encoder values if L and R value is not (1 or -1) for advanced uses. Better to use setDistancePerEncCount() instead for scaling
   *
   * Sets the encode direction for the motor.
   *
   * This function sets the encode direction for the motor based on the provided
   * values for the left (L) and right (R) directions. The `invert` parameter can
   * be used to swap the (L) and (R) motors if needed.
   *
   * @param L The value for the left direction.
   * @param R The value for the right direction.
   * @param invert Whether to swap the (L) and (R) motors (default: false).
   */
  void setEncodeDirection(double L, double R, bool invert = false)
  {
    lEncDirection = L, rEncDirection = R;
    invertEncoder = invert;
  }

  /**
   * @brief Sets the distance per encoder count for the motor.
   *
   * This function sets the conversion factor between encoder counts.
   * It takes two parameters, L and R, which represent the distance traveled per encoder count for the left and right motors, respectively.
   *
   * @param L The distance traveled per encoder count for the left motor.
   * @param R The distance traveled per encoder count for the right motor.
   * @return Returns 0 if successful, -1 if unsuccessful
   */
  int setDistancePerEncCount(double L, double R)
  {
    if (L <= 0 || R <= 0)
      return -1;

    lMotorDistancePerEncCount = L;
    rMotorDistancePerEncCount = R;
    return 0;
  }

  /**
   * @brief Sets the wheel base of the motor.
   *
   * This function sets the wheel base of the motor to the specified value.
   *
   * @param wb The wheel base value to set.
   */
  int setWheelBase(double wb)
  {
    if (wb <= 0)
      return -1;

    wheelBase = wb;
    return 0;
  }

  /**
   * Moves the motor forward for a specified distance.
   *
   * @param distance The distance to move forward.
   * @param steering The steering value (default: 0).
   * @param motorL The left motor speed (default: 255).
   * @param motorR The right motor speed (default: 255).
   * @param brakeAtEnd Whether to stop the motors at the end (default: true).
   */
  void moveForward(int distance, int steering = 0, int motorL = 255, int motorR = 255, bool brakeAtEnd = true)
  {
    if (!lMotorDistancePerEncCount)
      return;

    if (!distance)
      return;

    int encL, encR;
    getEncoderCount(&encL, &encR);
    const int startEncL = encL, startEncR = encR;

    int currentCountL, currentCountR;

    setMotorSteer(steering, motorL, motorR);

    do
    {
      getEncoderCount(&encL, &encR);
      currentCountL = encL - startEncL;
      currentCountR = encR - startEncR;
    } while (currentCountL * lMotorDistancePerEncCount + currentCountR * rMotorDistancePerEncCount < distance << 1);

    // Serial.print("Done moving forward\n");

    if (brakeAtEnd)
      // Serial.print("Braking\n");
      setMotorSteer(0, 0, 0);
  }

  /**
   * Turns the robot by a specified angle in radians.
   *
   * @param angle The angle to turn in radians.
   * @param motorL The speed of the left motor (default: 255).
   * @param motorR The speed of the right motor (default: 255).
   * @param brakeAtEnd Whether to stop the motors at the end of the turn (default: true).
   */
  void turnRad(double angleRad, int motorL = 255, int motorR = 255, bool brakeAtEnd = true, double turnFrictionCorrection = 1)
  {
    if (!lMotorDistancePerEncCount || !angleRad || !wheelBase || turnFrictionCorrection <= 0)
      return;

    angleRad *= turnFrictionCorrection;

    int encL, encR;
    getEncoderCount(&encL, &encR);
    const int startEncL = encL, startEncR = encR;

    int currentCountL, currentCountR;
    double currentAngle; // in rads

    if (angleRad > 0)
    {
      setMotorSteer(maxR, motorL, motorR);
    }
    else
    {
      setMotorSteer(maxL, motorL, motorR);
    }

    do
    {
      getEncoderCount(&encL, &encR);
      currentCountL = encL - startEncL;
      currentCountR = encR - startEncR;

      const double lDistance = currentCountL * lMotorDistancePerEncCount;
      const double rDistance = currentCountR * rMotorDistancePerEncCount;

      currentAngle = (lDistance - rDistance) / wheelBase;

      Serial.printf("lDistance: %f\t", lDistance);
      Serial.printf("rDistance: %f\t", rDistance);
      Serial.printf("Current Angle: %f\n", currentAngle);

    } while (currentAngle < angleRad && angleRad > 0 || currentAngle > angleRad && angleRad < 0);

    // Serial.print("Done turning\n");

    if (brakeAtEnd)
      // Serial.print("Braking\n");
      setMotorSteer(0, 0, 0);
  }

  /**
   * Turns the motor by the specified angle.
   *
   * @param angle The angle to turn the motor by.
   * @param motorL The speed of the left motor (default: 255).
   * @param motorR The speed of the right motor (default: 255).
   * @param brakeAtEnd Whether to stop motors at the end of the turn (default: true).
   */
  void turnDeg(double angle, int motorL = 255, int motorR = 255, double turnFrictionCorrection = 1, bool brakeAtEnd = true)
  {
    turnRad(angle * DEG_TO_RAD, motorL, motorR, brakeAtEnd, turnFrictionCorrection);
  }

  /**
   * Sets the steering and motor speeds for the motor.
   *
   * @param steering The steering value (-255 to 255, where negative values turn left and positive values turn right). Default is 0.
   * @param motorL The speed of the left motor (0 to 255, where 0 is stopped and 255 is maximum speed). Default is 255.
   * @param motorR The speed of the right motor (0 to 255, where 0 is stopped and 255 is maximum speed). Default is 255.
   */
  void setMotorSteer(int steering = 0, int motorL = 255, int motorR = 255)
  {
    // Add offset before applying the steering
    steering += offset;
    // Do not exceed boundaries for steering
    if (steering > maxR)
      steering = maxR;
    else if (steering < maxL)
      steering = maxL;

    // Multiply by direction to fix issue where motors could move the wrong direction
    motorL *= lMotorDirection;
    motorR *= rMotorDirection;

    Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
    /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
    if you want to increase the value range, first use a suitable variable type and then modify the code below
    for example; if the variable used to store motorL and motorR is 32-bits and you want to use signed values between -2^31 and (2^31)-1
    uncomment the four lines below relating to bits 32-25 and 24-17 for motorL and motorR
    for my microcontroller, int is 32-bits hence motorL and motorR are AND operated with a 32 bit hexadecimal number - change this if needed

    >> motorL refers to a shift right operator by motorL bits
    */

    // Wire.write((byte)((motorL & 0xFF000000) >> 24)); // bits 32 to 25 of motorL
    // Wire.write((byte)((motorL & 0x00FF0000) >> 16)); // bits 24 to 17 of motorL
    Wire.write((byte)((motorL & 0x0000FF00) >> 8)); // first byte of motorL, containing bits 16 to 9
    Wire.write((byte)(motorL & 0x000000FF));        // second byte of motorL, containing the 8 LSB - bits 8 to 1
    // Wire.write((byte)((motorR & 0xFF000000) >> 24)); // bits 32 to 25 of motorR
    // Wire.write((byte)((motorR & 0x00FF0000) >> 16)); // bits 24 to 17 of motorR
    Wire.write((byte)((motorR & 0x0000FF00) >> 8));   // first byte of motorR, containing bits 16 to 9
    Wire.write((byte)(motorR & 0x000000FF));          // second byte of motorR, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((steering & 0x0000FF00) >> 8)); // first byte of motorR, containing bits 16 to 9
    Wire.write((byte)(steering & 0x000000FF));        // second byte of motorR, containing the 8 LSB - bits 8 to 1
    Wire.endTransmission();                           // stop transmitting  }

    // Serial.print("MotorL: ");
    // Serial.print(motorL);
    // Serial.print("\tMotorR: ");
    // Serial.print(motorR);
    // Serial.print("\tSteering: ");
    // Serial.println(steering);
  }

  /// @brief function to get the encoder count from the slave
  /// @param encoderL Pointer to store the left encoder value
  /// @param encoderR Pointer to store the right encoder value
  void getEncoderCount(int *encoderL, int *encoderR)
  {
    int16_t a, b;

    // two 16-bit integer values are requested from the slave
    const uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4); // 4 indicates the number of bytes that are expected
    const uint8_t a16_9 = Wire.read();                                 // receive bits 16 to 9 of a (one byte)
    const uint8_t a8_1 = Wire.read();                                  // receive bits 8 to 1 of a (one byte)
    const uint8_t b16_9 = Wire.read();                                 // receive bits 16 to 9 of b (one byte)
    const uint8_t b8_1 = Wire.read();                                  // receive bits 8 to 1 of b (one byte)

    if (!invertEncoder)
    {
      a = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
      b = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number
    }
    else
    {
      b = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
      a = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number
    }

    // Print out a and b
    // Serial.printf("a: %d\t", a);
    // Serial.printf("b: %d\n", b);

    *encoderL = int(a * lEncDirection);
    *encoderR = int(b * rEncDirection);
  }
};