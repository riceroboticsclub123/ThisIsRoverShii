
#include <DynamixelShield.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8);

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const uint8_t DXL_ID_3 = 3;
const uint8_t DXL_ID_4 = 4;
const float DXL_PROTOCOL_VERSION = 2.0;


#define MOTOR_MODE_POSITION 0
#define MOTOR_MODE_VELOCITY 1

#define DEBUG

#define VELOCITY 50
#define MOTOR_ID_ERROR "Invalid Motor Id \"%c\"\n"
#define FORMATTING_ERROR_C "Invalid formatting Error: Invalid character \"%c\" at position %d\n"

String inputString = "";

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
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

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming byte
    String message = Serial.readString();
    //Print the received message back to the serial monitor
    Serial.print("Received: ");
    Serial.println(message);

    
    
  }
}



void motor_command() // Example Motor Command "M1:position:100&"
{
  // Getting motor id
  char id = inputString.charAt(1) - '0';

  // Getting the command
  int arg_position = inputString.indexOf(':', 3);

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