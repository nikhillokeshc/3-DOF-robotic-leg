//Include necessary libraries.

#include <SoftwareSerial.h>   // to communicate with Arduino Uno over UART

SoftwareSerial serial(7,8);    // Initialize the rx/tx pins for serial communication

void setup()
{
  Serial.begin(115200);        // Initialize to enable communicaiton from Nano to Personal computer.
   
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Nano ready to recieve! :)");
  
  serial.begin(57600);          // This baud rate has to match the UNO's Debug_serial baud rate
}

void loop()
{
  // if any data is available on programmed Rx pin. Print to Serial monitor. 
  if (serial.available()) {
    Serial.write(serial.read());
  }
}
