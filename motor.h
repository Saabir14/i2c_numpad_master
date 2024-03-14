// By Saabir Ahmed Parvaiz Ahmed (2024-3-14)
// This code is used to control the motors and read the encoder values of the car
#include <Wire.h>

/// @brief Class to control the motors and read the encoder values of the car using I2C
class MotorController {
public:
  // This is the address of the slave device
  int I2C_SLAVE_ADDR;
  int offset;
  int maxL;
  int maxR;

  int lMotorDirection = 1, rMotorDirection = 1;
  bool invertMotor = false;

  int lEncDirection = 1, rEncDirection = 1;
  bool invertEncoder = false;

  double lMotorDistancePerEncCount = 0, rMotorDistancePerEncCount = 0;

  /// @brief Function to initialize the MotorController
  /// @param slaveAddr Address of the slave device
  /// @param offset Offset to add to the steering value
  /// @param maxL Maximum value for left turning of steering motor
  /// @param maxR Maximum value for right turning of steering motor
  MotorController(int slaveAddr, int offset = 0, int maxL = -75, int maxR = 75)
  {
    // Set up the I2C communication incase it is not setup in main code (so people don't have to in main code)
    Wire.begin();
    I2C_SLAVE_ADDR = slaveAddr;
    this->offset = offset;
    this->maxL = maxL;
    this->maxR = maxR;
  }

  /// @brief Function to set the motor direction. Can be used to invert motor values if L and R value is not (1 or -1) for advanced uses
  /// @param L Direction of left motor
  /// @param R Direction of right motor
  /// @param invert Flag that indicates if the motor values should be inverted (right motor is left and vice versa)
  void setMotorDirection(int L, int R, bool invert = false)
  {
    lMotorDirection = L, rMotorDirection = R;
    invertMotor = invert;
  }

  /// @brief Function to set the encoder direction. Can be used to scale encoder values if L and R value is not (1 or -1) for advanced uses. Better to use setDistancePerEncCount() instead for scaling
  /// @param L Direction of left encoder
  /// @param R Direction of right encoder
  /// @param invert Flag that indicates if the encoder values should be inverted (right encoder is left and vice versa)
  void setEncodeDirection(int L, int R, bool invert = false)
  {
    lEncDirection = L, rEncDirection = R;
    invertEncoder = invert;
  }

  /// @brief Function to set the distance per encoder count
  /// @param L Distance per encoder count for left encoder
  /// @param R Distance per encoder count for right encoder
  /// @return Returns 0 if successful, -1 if unsuccessful
  int setDistancePerEncCount(double L, double R)
  {
    if (L <= 0 || R <= 0)
      return -1;

    lMotorDistancePerEncCount = L;
    rMotorDistancePerEncCount = R;
    return 0;
  }

  /// @brief Function to move the car forward for a certain distance
  /// @param distance distance to move forward in mm
  /// @param steering steering value (not angle)
  /// @param motorL power to left motor
  /// @param motorR power to right motor
  /// @param brakeAtEnd flag to brake at end
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

  /// @brief Function to set the motor speed and steering
  /// @param steering 
  /// @param motorL 
  /// @param motorR 
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

    //Wire.write((byte)((motorL & 0xFF000000) >> 24)); // bits 32 to 25 of motorL
    //Wire.write((byte)((motorL & 0x00FF0000) >> 16)); // bits 24 to 17 of motorL
    Wire.write((byte)((motorL & 0x0000FF00) >> 8));    // first byte of motorL, containing bits 16 to 9
    Wire.write((byte)(motorL & 0x000000FF));           // second byte of motorL, containing the 8 LSB - bits 8 to 1
    //Wire.write((byte)((motorR & 0xFF000000) >> 24)); // bits 32 to 25 of motorR
    //Wire.write((byte)((motorR & 0x00FF0000) >> 16)); // bits 24 to 17 of motorR
    Wire.write((byte)((motorR & 0x0000FF00) >> 8));    // first byte of motorR, containing bits 16 to 9
    Wire.write((byte)(motorR & 0x000000FF));           // second byte of motorR, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((steering & 0x0000FF00) >> 8));    // first byte of motorR, containing bits 16 to 9
    Wire.write((byte)(steering & 0x000000FF));           // second byte of motorR, containing the 8 LSB - bits 8 to 1
    Wire.endTransmission();   // stop transmitting  }

    // Serial.print("MotorL: ");
    // Serial.print(motorL);
    // Serial.print("\tMotorR: ");
    // Serial.print(motorR);
    // Serial.print("\tSteering: ");
    // Serial.println(steering);
  }

  /// @brief function to get the encoder count from the slave
  /// @param encoderL 
  /// @param encoderR 
  void getEncoderCount(int *encoderL, int *encoderR)
  {
    int16_t a, b;

    // two 16-bit integer values are requested from the slave
    const uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
    const uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
    const uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
    const uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
    const uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

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

    *encoderL = a * lEncDirection;
    *encoderR = b * rEncDirection;
  }
};