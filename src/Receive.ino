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

struct MultiBtnBitFieldType {
  bool MultiBlueBtn:1;
  bool MultiWhiteBtn:1;
  bool MultiYellowBtn:1;
  bool MultiRedBtn:1;
  bool MultiOrangeBtn:1;
  bool MultiGreenBtn:1;
  bool MultiGrayBtn:1;
  bool MultiBlackBtn:1;
};

union MultiBtnCharType {

  MultiBtnBitFieldType MultiBtnBitField;
  char MultiBtnByte;
} MultiBtnRcvCharType;

struct command_type {
  int Speed;
  int SteeringAngle;
  bool FrontLight;
  bool ENPO;
  char MultiBtnChar;
};

union SerializedData_type {
  command_type command;
  char command_serial[10];
} SerializedData;

Servo ESC1;
Servo Steering;
// Need an instance of the Radio Module
RFM12B radio;

int ENPOPin = 3;
int IR_LPin = 4;
int IR_FLPin = 5;
int IR_RPin = 6;
int IR_FRPin = 7;

bool IR_L=0;
bool IR_FL=0;
bool IR_R=0;
bool IR_FR=0;

const int MaxSpeed = 85;


void setup()
{
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);      //comment this out to disable encryption
  Serial.begin(SERIAL_BAUD);
  Serial.println("Listening...");
  pinMode(ENPOPin,OUTPUT);
  pinMode(IR_LPin,INPUT);
  pinMode(IR_FLPin,INPUT);
  pinMode(IR_RPin,INPUT);
  pinMode(IR_FRPin,INPUT);
  ESC1.attach(9);
  Steering.attach(8);
  Wire.begin();
}

void loop()
{
  boolean WireResult = 0;

  IR_L = digitalRead(IR_LPin) > 0;
  IR_FL = digitalRead(IR_FLPin) > 0;
  IR_R = digitalRead(IR_RPin) > 0;
  IR_FR = digitalRead(IR_FRPin) > 0; 

  bool ObstDetect = IR_L | IR_FL | IR_R | IR_FR;

  if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      for (byte i = 0; i < *radio.DataLen; i++) //can also use radio.GetDataLen() if you don't like pointers
        SerializedData.command_serial[i]= radio.Data[i];

      Serial.println(SerializedData.command.ENPO);

      if ((ObstDetect) && (SerializedData.command.Speed > MaxSpeed)) {

        SerializedData.command.Speed = MaxSpeed;
      }

      ESC1.write(SerializedData.command.Speed);
      Steering.write(SerializedData.command.SteeringAngle);
      
      Wire.beginTransmission(8); // I2C to Light Controller
      Wire.write(SerializedData.command.FrontLight); // sending front light switch state to light controller
      Wire.write(SerializedData.command.SteeringAngle & 0xff);  // sending steering angle to light controller for turn indicator
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      Wire.beginTransmission(9); // I2C to Audio Controller
      Wire.write(SerializedData.command.MultiBtnChar);
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      if (SerializedData.command.ENPO) digitalWrite(ENPOPin,HIGH);
      else  digitalWrite(ENPOPin,LOW);

      if (radio.ACKRequested())
      {
        radio.SendACK();
      }
    }
  }


}
