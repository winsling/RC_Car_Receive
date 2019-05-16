// Simple serial pass through program
// It initializes the RFM12B radio with optional encryption and passes through any valid messages to the serial port
// felix@lowpowerlab.com

#include <RFM12B.h>
#include <Servo.h>
#include <Wire.h>


// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID           1  //network ID used for this unit
#define NETWORKID       99  //the network ID we are on
#define SERIAL_BAUD 115200

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";

struct command_type {
  int Speed;
  int SteeringAngle;
};

union SerializedData_type {
  command_type command;
  char command_serial[14];
} SerializedData;

Servo ESC1;
Servo Steering;
// Need an instance of the Radio Module
RFM12B radio;
void setup()
{
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);      //comment this out to disable encryption
  Serial.begin(SERIAL_BAUD);
  Serial.println("Listening...");
  ESC1.attach(9);
  Steering.attach(8);
  Wire.begin();
}

void loop()
{
  if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      for (byte i = 0; i < *radio.DataLen; i++) //can also use radio.GetDataLen() if you don't like pointers
        SerializedData.command_serial[i]= radio.Data[i];

      ESC1.write(SerializedData.command.Speed);
      Steering.write(SerializedData.command.SteeringAngle);

      if (radio.ACKRequested())
      {
        radio.SendACK();
      }
    }
  }

  Wire.beginTransmission(42);
  Wire.write(0);
  boolean WireResult = Wire.endTransmission();
}
