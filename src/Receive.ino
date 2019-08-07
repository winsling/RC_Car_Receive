#include <RFM12B.h>
#include <Wire.h>

// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID 1     //network ID used for this unit
#define NETWORKID 99 //the network ID we are on
#define SERIAL_BAUD 115200

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";

struct MultiBtnBitFieldType
{
  bool MultiBlueBtn : 1;
  bool MultiWhiteBtn : 1;
  bool MultiYellowBtn : 1;
  bool MultiRedBtn : 1;
  bool MultiOrangeBtn : 1;
  bool MultiGreenBtn : 1;
  bool MultiGrayBtn : 1;
  bool MultiBlackBtn : 1;
};

union MultiBtnCharType {

  MultiBtnBitFieldType MultiBtnBitField;
  char MultiBtnByte;
} MultiBtnRcvCharType;

struct command_type
{
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

#define ENPOPin 3
#define TRIGGER_PIN 5 // Arduino Pin an HC-SR04 Trig
#define ECHO_PIN 4    // Arduino Pin an HC-SR04 Echo

#define MaxObstacleSpeed 90

bool Obst_Detect = false;

void setup()
{
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  //  radio.Encrypt(KEY);      //comment this out to disable encryption
  Serial.begin(SERIAL_BAUD);
  Serial.println("Listening...");
  pinMode(ENPOPin, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  //ESC1.attach(9);
  //Steering.attach(8);
  Wire.begin();
  digitalWrite(TRIGGER_PIN, HIGH);
}

float getDist()
{
  float dist = 0;
  float time_echo = 0;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(TRIGGER_PIN, HIGH); //Trigger Impuls 10 us
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  time_echo = pulseIn(ECHO_PIN, HIGH); // Echo-Zeit messen
  interrupts();
  time_echo = (time_echo / 2); // Zeit halbieren
  dist = time_echo / 29.1;     // Zeit in Zentimeter umrechnen
  //Serial.println(dist);
  return (dist);
}

void loop()
{
  boolean WireResult = 0;

  static float act_dist = 0;
  float old_dist = 0;
  static unsigned long t1, t2;
  float speed = 0;
  static int threshold = 0;
  static unsigned long Obst_t0 = 0;
  static unsigned long Break_t0 = 0;
  static bool Break_flag = false;

  if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      for (byte i = 0; i < *radio.DataLen; i++) //can also use radio.GetDataLen() if you don't like pointers
        SerializedData.command_serial[i] = radio.Data[i];

      if (!Break_flag)
      {
        old_dist = act_dist;
        act_dist = getDist();
        t2 = t1;
        t1 = millis();

        if (SerializedData.command.Speed > 81)
        {

          if ((old_dist < 121) && (act_dist > 0) && (act_dist < 120) && (act_dist < old_dist))
          {

            speed = (old_dist - act_dist) / (t1 - t2);
            if ((speed > 0.3) && (act_dist > 80))
            {
              threshold += 1;
              Obst_t0 = millis();
            }
            
            if ((speed > 0.2) && (act_dist < 80) && (act_dist > 40))
            {
              threshold += 1;
              Obst_t0 = millis();
            }
            if ((speed > 1) && (act_dist < 40))
            {
              threshold += 1;
              Obst_t0 = millis();
            }

            if (threshold > 0)
            {
              if (millis() > (Obst_t0 + 2000))
                threshold = 0;
            }

            if (threshold > 5)
            {
              //Serial.println("Obstacle");
              threshold = 0;
              Break_t0 = millis();
              Break_flag = true;
              SerializedData.command.Speed = 60;
              Obst_Detect = true;
            }
          }
        }
      }

      if (Break_flag)
      {
        SerializedData.command.Speed = 60;
        if (millis() > Break_t0 + 50)
          SerializedData.command.Speed = 81;
        if (millis() > Break_t0 + 500)
        {
          Break_flag = false;
          Obst_Detect = false;
        }
      }

      if (SerializedData.command.Speed > 94)
        SerializedData.command.Speed = 94;

      Wire.beginTransmission(7);                               // I2C to Servo Controller
      Wire.write(SerializedData.command.SteeringAngle & 0xff); // sending steering angle to light controller for turn indicator
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      Wire.beginTransmission(8);                               // I2C to Light Controller
      Wire.write(SerializedData.command.FrontLight);           // sending front light switch state to light controller
      Wire.write(SerializedData.command.SteeringAngle & 0xff); // sending steering angle to light controller for turn indicator
      Wire.write(Obst_Detect);
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      Wire.beginTransmission(9); // I2C to Audio Controller
      Wire.write(SerializedData.command.MultiBtnChar);
      Wire.write(SerializedData.command.SteeringAngle & 0xff); // sending steering angle to light controller for turn indicator
      Wire.write(SerializedData.command.Speed);
      WireResult = Wire.endTransmission();

      if (SerializedData.command.ENPO)
        digitalWrite(ENPOPin, HIGH);
      else
        digitalWrite(ENPOPin, LOW);

      if (radio.ACKRequested())
      {
        radio.SendACK();
      }
    }
  }
}
