// #include <Keypad.h>
// #include "motor.h"

// #define ROW_NUM     4 // four rows
// #define COLUMN_NUM  3 // three columns

// char keys[ROW_NUM][COLUMN_NUM] = {
//   {'1', '2', '3'},
//   {'4', '5', '6'},
//   {'7', '8', '9'},
//   {'*', '0', '#'}
// };

// byte pin_rows[ROW_NUM] = {0, 13, 16, 15}; // GPIO18, GPIO5, GPIO17, GPIO16 connect to the row pins
// byte pin_column[COLUMN_NUM] = {4, 2, 17};  // GPIO4, GPIO0, GPIO2 connect to the column pins

// Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

// #define I2C_SLAVE_ADDR 0x04 // Slave address

// enum State {
//   WAIT_FOR_COMMAND,
//   ENTERING_DISTANCE,
//   ENTERING_ANGLE,
//   STOP_SERVICES
// };

// State currentState = WAIT_FOR_COMMAND;
// char commandType = ' ';
// String inputValue = "";
// String commandHistory = "";

// const double distancePerEncCount = 4.385139;
// MotorController motorController(I2C_SLAVE_ADDR);

// void executeCommands() {
//   if (commandHistory.isEmpty()) {
//     Serial.println("No commands to execute.");
//   }
//   else {
//     Serial.println("Running Commands");

//     // Example: commandHistory = "F100 L90 F200 R180 B100"
//     // Parse commandHistory by space character and send commands to motorController
//     char* command = strtok(&commandHistory[0u], " ");
//     while(command) {
//       char commandType = command[0];
//       int value = atoi(command + 1);

//       char argument;
//       switch(commandType) {
//         case '2':
//           argument = 'F';
//           break;
//         case '8':
//           argument = 'B';
//           break;
//         case '6':
//           argument = 'R';
//           break;
//         case '4':
//           argument = 'L';
//           break;
//         default:
//           argument = commandType; // If no mapping is defined, keep the original value
//           break;
//       }

//       Serial.print("Now Running: ");
//       Serial.print(argument);
//       Serial.println(value);

//       // Commands usable hear, argument is the command type and value is the distance or angle
//       if (argument == 'F') {
//         motorController.moveForward(value);
//       }
//       // else if (argument == 'L' || argument == 'R') {
//       //   motorController.turn(value);
//       // }

//       command = strtok(NULL, " ");
//     }

//     Serial.println("All Commands Executed");
//     commandHistory = "";
//   }
    
//   currentState = WAIT_FOR_COMMAND;
// }

// void setup() {
//   motorController.setEncodeDirection(1, -1, true);
//   motorController.setDistancePerEncCount(distancePerEncCount);

//   Serial.begin(9600);
//   Wire.begin(); // Initialize I2C communication
//   Serial.println("Ready for command. Press '2' for forward, '8' for backward, '4' for left, '6' for right, '*' to enter next movement, '#' to finish all commands, and '5' to start executing.");
// }

// void sendCommandOverI2C(char commandType, int value) {
//   Wire.beginTransmission(I2C_SLAVE_ADDR);
//   Wire.write(commandType); // Send the command type as before

//   // Convert int to bytes and send
//   Wire.write((byte*)&value, sizeof(value));

//   Wire.endTransmission();
// }


// void processCommand() {
//   if (!inputValue.isEmpty()) {
//     int value = inputValue.toInt(); // Convert input value to integer

//     // Debug print the command and value
//     Serial.print("Command stored: ");
//     Serial.print(commandType);
//     Serial.print(" with value: ");
//     Serial.println(value);

//     // Append command to commandHistory
//     commandHistory += commandType;
//     commandHistory += value;
//     commandHistory += ' ';

//     // Send command and integer value
//     //sendCommandOverI2C(commandType, value);

//     // Reset for the next command
//     inputValue = "";
//     commandType = ' ';
//     currentState = WAIT_FOR_COMMAND;
//   }
// }

// void loop() {
//   char key = keypad.getKey();
//   if (key) {
//     Serial.println(key);
//     switch (currentState) {
//       case WAIT_FOR_COMMAND:
//         if (key == '2' || key == '8' || key == '4' || key == '6') {
//           commandType = key == '2' ? 'F' : key == '8' ? 'B' : key == '4' ? 'L' : 'R';
//           currentState = (key == '2' || key == '8') ? ENTERING_DISTANCE : ENTERING_ANGLE;
//           Serial.print("Enter ");
//           Serial.print((key == '2' || key == '8') ? "distance" : "angle. Press '1' for 90 degrees, '2' for 180 degrees.");
//           Serial.println(":");
//         } else if (key == '#') {
//           currentState = STOP_SERVICES;
//           Serial.println("Input stopped. Press '5' to execute stored commands.");
//         }
//         break;
//       case ENTERING_DISTANCE:
//         if (key >= '0' && key <= '9') {
//           inputValue += key;
//         } else if (key == '*') {
//           processCommand();
//         } else if (key == '#') {
//           processCommand();
//           currentState = STOP_SERVICES;
//           Serial.println("Input stopped. Press '5' to execute stored commands.");
//         }
//         break;
//       case ENTERING_ANGLE:
//         if (key == '1' || key == '2') {
//           inputValue = key == '1' ? "90" : "180";
//         } else if (key == '*') {
//           processCommand();
//         } else if (key == '#') {
//           processCommand();
//           currentState = STOP_SERVICES;
//           Serial.println("Input stopped. Press '5' to execute stored commands.");
//         }
//         break;
//       case STOP_SERVICES:
//         if (key == '5') {
//           executeCommands();
//         }
//         break;
//     }
//   }
// }