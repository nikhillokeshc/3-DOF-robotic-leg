// Include necessary libraries.

#include <DynamixelShield.h>  // to communicate with dynamixel shied and motors
#include <SoftwareSerial.h>   // to communicate with Nano using software programmed serial communication.

// Debug serial to log data using Nano.
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial

// initializing motor id's
const uint8_t AnkleMotor_ID = 1;
const uint8_t KneeMotor_ID = 2;
const uint8_t HipMotor_ID = 3;

const float DXL_PROTOCOL_VERSION = 2.0;

// initializing global variables for future use.
double q[3] = {0,0,0}; // joint angles(degrees)
float qh[3] = {180, 180, 180};  // Joint angles home configuration(degrees)
float qr[3] = {0,0,0};          // Reference Joint angles
float w[3] = {0,0,0};           // Joint angular velocities
float theta = 0;                // variable used for ref trajectory generation

// dynamixel object initialisation
DynamixelShield dxl;


void setup()
{
  dxl.begin(9600);              //Initializing dynamixel comm with 9600 baud rate
  DEBUG_SERIAL.begin(57600);    //Initialize serial comm with Nano.
  
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get info of the dynamixel motors attached to the shield
  dxl.ping(AnkleMotor_ID);
  dxl.ping(KneeMotor_ID);
  dxl.ping(HipMotor_ID);

  // Setting the motors in position mode.
  // Torque has to be turned off to change the operating mode of 
  // motors. As this is in EEPROM area of controller in motors
  
  dxl.torqueOff(AnkleMotor_ID);
  dxl.torqueOff(KneeMotor_ID);
  dxl.torqueOff(HipMotor_ID);
  
  dxl.setOperatingMode(AnkleMotor_ID, OP_POSITION);
  dxl.setOperatingMode(KneeMotor_ID, OP_POSITION);
  dxl.setOperatingMode(HipMotor_ID, OP_POSITION);

  dxl.torqueOn(AnkleMotor_ID);
  dxl.torqueOn(KneeMotor_ID);
  dxl.torqueOn(HipMotor_ID);
}



void loop()
{
  // Go to home configuration
  dxl.setGoalPosition(AnkleMotor_ID, qh[0], UNIT_DEGREE);
  dxl.setGoalPosition(KneeMotor_ID, qh[1], UNIT_DEGREE);
  dxl.setGoalPosition(HipMotor_ID, qh[2], UNIT_DEGREE);

  // wait for 5 seconds
  delay(5000);
  
  // Generate a sin wave from phase (0 - pi) in 64 steps.
  for(int i = 0; i<64; i++)
  {
    theta = M_PI/64.0*(float)i;
    
    q[0] = qh[0] - 30*sin(theta); // in Degrees
    q[1] = qh[1] + 60*sin(theta);
    q[2] = qh[2] - 30*sin(theta);

    // set the next reference position for all joints
    dxl.setGoalPosition(AnkleMotor_ID, q[0], UNIT_DEGREE);
    dxl.setGoalPosition(KneeMotor_ID, q[1], UNIT_DEGREE);
    dxl.setGoalPosition(HipMotor_ID, q[2], UNIT_DEGREE);

    // Read the present postion and velocity of joints
    qr[0] = dxl.getPresentPosition(AnkleMotor_ID, UNIT_DEGREE);
    w[0] = dxl.getPresentVelocity(AnkleMotor_ID, UNIT_RPM);
    qr[1] = dxl.getPresentPosition(KneeMotor_ID, UNIT_DEGREE);
    w[1] = dxl.getPresentVelocity(KneeMotor_ID, UNIT_RPM);
    qr[2] = dxl.getPresentPosition(HipMotor_ID, UNIT_DEGREE);
    w[2] = dxl.getPresentVelocity(HipMotor_ID, UNIT_RPM);

    // wait for 10 milliseconds.
    // Changing this wait time changes the frequency of input ref sin angles
    delay(10);

    // Print all the results to Nano.
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print(qr[0]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.print(qr[1]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.print(qr[2]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.print(w[0]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.print(w[1]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.print(w[2]);
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.println(millis());
  }
  //while(1){};
}
