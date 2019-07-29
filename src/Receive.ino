#include <RFM12B.h>
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

// Need an instance of the Radio Module
RFM12B radio;

int ENPOPin = 3;
#define TRIGGER_PIN 5 // Arduino Pin an HC-SR04 Trig
#define ECHO_PIN 4    // Arduino Pin an HC-SR04 Echo

#define MaxObstacleSpeed 90

bool Obst_Detect = 0;


void setup()
{
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
//  radio.Encrypt(KEY);      //comment this out to disable encryption
  Serial.begin(SERIAL_BAUD);
  Serial.println("Listening...");
  pinMode(ENPOPin,OUTPUT);
  pinMode(TRIGGER_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  //ESC1.attach(9);
  //Steering.attach(8);
  Wire.begin();
  digitalWrite(TRIGGER_PIN,HIGH);
}

int getDist()
{
  long dist=0;
  long time_echo=0;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(TRIGGER_PIN, HIGH); //Trigger Impuls 10 us
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  time_echo = pulseIn(ECHO_PIN, HIGH); // Echo-Zeit messen
  interrupts();
  time_echo = (time_echo/2); // Zeit halbieren
  dist = time_echo / 29.1; // Zeit in Zentimeter umrechnen
  //Serial.println(dist);
  return(dist);
}


void loop()
{
  boolean WireResult = 0;

  long act_dist = 0;
  
  
  if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      for (byte i = 0; i < *radio.DataLen; i++) //can also use radio.GetDataLen() if you don't like pointers
        SerializedData.command_serial[i]= radio.Data[i];

      Obst_Detect = false;
      
      act_dist = getDist();

      if ((act_dist<90) && (SerializedData.command.Speed > MaxObstacleSpeed) && (act_dist>0)) {
        Obst_Detect = true;
        SerializedData.command.Speed = MaxObstacleSpeed;
      }

      if (SerializedData.command.Speed >88 && (act_dist<40)) {
        SerializedData.command.Speed = 88;
        Obst_Detect = true;
      }

//      if (SerializedData.command.Speed >85 && (act_dist<20)) {
//        SerializedData.command.Speed = 85;
//      }

      
      Wire.beginTransmission(7); // I2C to Servo Controller
      Wire.write(SerializedData.command.SteeringAngle & 0xff);  // sending steering angle to light controller for turn indicator
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      Wire.beginTransmission(8); // I2C to Light Controller
      Wire.write(SerializedData.command.FrontLight); // sending front light switch state to light controller
      Wire.write(SerializedData.command.SteeringAngle & 0xff);  // sending steering angle to light controller for turn indicator
      Wire.write(Obst_Detect);
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      Wire.beginTransmission(9); // I2C to Audio Controller
      Wire.write(SerializedData.command.MultiBtnChar);
      Wire.write(SerializedData.command.SteeringAngle & 0xff);  // sending steering angle to light controller for turn indicator
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
