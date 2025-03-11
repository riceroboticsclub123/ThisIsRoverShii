#include <DynamixelShield.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8);

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const uint8_t DXL_ID_3 = 3;
const uint8_t DXL_ID_4 = 4;
const float DXL_PROTOCOL_VERSION = 2.0;

#define STOP 0
#define MOVE_FORWARD 1
#define MOVE_BACKWARDS 2
#define ROTATE_LEFT 3
#define ROTATE_RIGHT 4

#define MOTOR_MODE_POSITION 0
#define MOTOR_MODE_VELOCITY 1

#define DEBUG

String inputString = "";

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  // DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);
  dxl.ping(DXL_ID_3);
  dxl.ping(DXL_ID_4); 

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_1);
  dxl.torqueOff(DXL_ID_2);
  dxl.torqueOff(DXL_ID_3);
  dxl.torqueOff(DXL_ID_4);
  dxl.setOperatingMode(DXL_ID_1, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_2, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_3, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_4, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_1);
  dxl.torqueOn(DXL_ID_2);
  dxl.torqueOn(DXL_ID_3);
  dxl.torqueOn(DXL_ID_4);

  soft_serial.begin(9600);
}

#define VELOCITY 50

// void setState(int state_id)
// {
//   switch (state_id)
//   {
//     case STOP:
//       dxl.setGoalVelocity(DXL_ID_1, 0);
//       dxl.setGoalVelocity(DXL_ID_2, -0);
//       dxl.setGoalVelocity(DXL_ID_3, -0);
//       dxl.setGoalVelocity(DXL_ID_4, 0);
//       break;
//     case MOVE_FORWARD:
//       dxl.setGoalVelocity(DXL_ID_1, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_2, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_3, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_4, VELOCITY);
//       break;
//     case MOVE_BACKWARDS:
//       dxl.setGoalVelocity(DXL_ID_1, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_2, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_3, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_4, -VELOCITY);
//       break;
//     case ROTATE_LEFT:
//       dxl.setGoalVelocity(DXL_ID_1, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_2, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_3, VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_4, VELOCITY);
//       break;
//     case ROTATE_RIGHT:
//       dxl.setGoalVelocity(DXL_ID_1, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_2, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_3, -VELOCITY);
//       dxl.setGoalVelocity(DXL_ID_4, -VELOCITY);
//       break;
//   }
// }

#define MOTOR_ID_ERROR "Invalid Motor Id \"%c\"\n"
#define FORMATTING_ERROR_C "Invalid formatting Error: Invalid character \"%c\" at position %d\n"

void motor_command() // Example Motor Command "M1:position:100&"
{
  // Getting motor id
  char id = inputString.charAt(1) - '0';

  // checking for valid id 1,2,3 or 4
  #ifdef DEBUG
  if (id < 1 || id > 4) {
    soft_serial.println("Invalid Motor Id \""+String(inputString.charAt(1))+"\"");
    return;
  }
  
  if (inputString.charAt(2) != ':') {
    soft_serial.println("Invalid formatting Error: Invalid character \""+String(inputString.charAt(2))+"\" at position 2");
    return;
  }
  #endif

  // Getting the command
  int arg_position = inputString.indexOf(':', 3);

  #ifdef DEBUG
  if (arg_position == -1) {
    soft_serial.println("Formatting Error: Unable to find the : needed to mark the end of an argument");
  }
  #endif

  String motor_command = inputString.substring(3, arg_position);
  int value_arg_pos = inputString.indexOf('&', arg_position+1);
  String value_str = inputString.substring(arg_position+1, value_arg_pos);
  float value = value_str.toFloat();

  if (motor_command == "set_position_raw") {
    dxl.setGoalPosition(id, value);
  } else if (motor_command == "set_position_deg") {
    dxl.setGoalPosition(id, value, UNIT_DEGREE);
  } else if (motor_command == "set_velocity_raw") {
    dxl.setGoalVelocity(id, value);
  } else if (motor_command == "set_velocity_rpm") {
    dxl.setGoalVelocity(id, value, UNIT_RPM);
  } else if (motor_command == "set_velocity_per") {
    dxl.setGoalVelocity(id, value, UNIT_PERCENT);
  } 
  #ifdef DEBUG
  else {
    soft_serial.println("Invalid Motor Command:\"" + motor_command+"\"");
  }
  #endif
}

void loop() {
  // dxl.setGoalVelocity(DXL_ID_1, VELOCITY);
  int incomingByte = 0;
  char read_message = 0;
  while (soft_serial.available() > 0) {
    // read the incoming byte:
    incomingByte = soft_serial.read();
    inputString += incomingByte;

    #ifdef DEBUG
    soft_serial.println("I received: "+String(incomingByte));
    #endif

    if (incomingByte == '&') { // end character
      read_message = 1;

      // print command 
      #ifdef DEBUG
      soft_serial.println("I received command: \"" + inputString + "\"");
      #endif
    }
  }

  // Guard Clause
  if (read_message == 0) {return;}

  // Parsing the command
  switch (inputString.charAt(0)) {
    case 'M':
      motor_command();
      break;
  }

  inputString = ""; // clear after use
}
