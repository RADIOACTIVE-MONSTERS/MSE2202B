#include <ExtraCore.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <NewPing.h>

ExtraCore extraCore;

//Ultrasonic (TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE)
NewPing sonar(7, 8, 200);

//Servos
Servo shoulderServo;
Servo elbowServo;
Servo handServo;
Servo torsoServo;
Servo leftMotor;
Servo rightMotor;


typedef enum ultrasonic { distance_front, distance_back, distance_up, distance_angle };

//Force Sensor
const int forceSensor = 0; 

// Hall Effect
const int ci_HallEffect_Data = A3;

// Line Sensor
const int ci_LineSensor_Data = A3;

const int shoulderPin = 8;   
const int elbowPin = 9;
const int handPin = 2;
const int torsoPin = 3;

const int rightMotorPin = 10;
const int leftMotorPin = 11;

void setup()
{
  extraCore.beginManager();//begin Manager role. (beginClient() on remote)
  //extraCore.setPinIOstate(pinNumber, OUTPUT);
  Serial.begin(9600);  
  Serial.println("Manager Ready.");

  //attaching servos
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);
  handServo.attach(handPin, 500, 2500);
  torsoServo.attach(torsoPin, 500, 2500);

  leftMotor.attach(leftMotorPin, 500, 2500);
  rightMotor.attach(rightMotorPin, 500, 2500);
  
  // set up Hall Effect
  pinMode(ci_HallEffect_Data, INPUT);
  //digitalWrite(ci_HallEffect_Data, HIGH);
}

  
void loop()
{
  delay(100);//Don't remove this delay. Sending updates too quickly will block data returning.
  // Setup PWM example.

  /* 2nd core EXAMPLE **
  extraCore.setAnalogOutput(pinNumber, value);//steps LED brightness up and down.
  extraCore.setDigitalOutput(pinNumber, HIGH);//Set the remote digital high.
  extraCore.sendConfig();//changes won't take effect until you sendConfig();*/
  
  leftMotor.write(150);     //driving straight(ish)
  rightMotor.write(150);
  
  /* MAIN CODE GOES HERE! :) */

}


// measure distance to target using ultrasonic sensor  
int Ping(int type)
{
  switch(type)
  {
      case distance_front  : return (extraCore.getAnalogReading(A1))/10;   break;
      case distance_back: return (extraCore.getAnalogReading(A2))/10;  break;
      case distance_angle : return (extraCore.getAnalogReading(A0))/5;  break;
      case distance_up : return sonar.ping_cm(); break;
  }
  
  return 0;
}

// measure distance to target using ultrasonic sensor  
boolean HallEffect()
{
  int sensorValue = analogRead(ci_HallEffect_Data);

  Serial.print("Hall Effect Value: ");
  Serial.println(sensorValue, DEC);
  return true;
}

// detect black line  
boolean DetectBlackLine()
{
  int sensorValue = analogRead(ci_LineSensor_Data);

  if(sensorValue > 500) {
     return true;
  }
  return false;
  
  /*
  Serial.print("Hall Effect Value: ");
  Serial.println(sensorValue, DEC);
  */
}

void moveArm(int pos)           //moves arm to different positions
{
  if (pos == 1)   //searching
  {
    torsoServo.write(10);
    delay(500);
    //handServo.write(125);
    moveMagnet(true);
    delay(100);
    elbowServo.write(180);
    delay(100);
    shoulderServo.write(36);
    delay(100);
  }
  else if (pos == 2)    //dropping off
  {
    torsoServo.write(100);
    delay(500);
    shoulderServo.write(60);
    delay(100);
    elbowServo.write(180);
    delay(500);
    //handServo.write(15);
    moveMagnet(false);
    delay(100);
  }
  else if (pos == 3)  //going under bridge
  {
    torsoServo.write(10);
    delay(500);
    //handServo.write(125);
    moveMagnet(true);
    delay(100);
    elbowServo.write(116);
    delay(100);
    shoulderServo.write(36);
    delay(100);
  }
  else if (pos == 4)  //extended
  {
    torsoServo.write(10);
    delay(500);
    //handServo.write(125);
    moveMagnet(true);
    delay(100);
    elbowServo.write(116);
    delay(100);
    shoulderServo.write(108);
    delay(100);
  }
}

int readForceSensor()           //returns value from the force sensor
{
  int forceReading = analogRead(forcePin);
  Serial.print("Analog reading = ");
  Serial.println(forceReading);
  return forceReading;
}

void armJiggle()        //will eventually move the arm back and forth for searching
{
  torsoServo.write(20);
  delay(20);
  torsoServo.write(0);
  delay(20);
}

void moveMagnet (boolean extended)      //extends and retracts magnet
{
  if (extended)
    handServo.write(125);
  else
    handServo.write(25);
}
