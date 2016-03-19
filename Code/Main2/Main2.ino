#include <ExtraCore.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <NewPing.h>

ExtraCore extraCore;

//Ultrasonic (TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE)
NewPing sonar(7, 8, 200);


typedef enum ultrasonic { distance_front, distance_back, distance_up, distance_angle };

// Hall Effect
const int ci_HallEffect_Data = A3;

// Line Sensor
const int ci_LineSensor_Data = A3;

void setup()
{
  extraCore.beginManager();//begin Manager role. (beginClient() on remote)
  //extraCore.setPinIOstate(pinNumber, OUTPUT);
  Serial.begin(9600);  
  Serial.println("Manager Ready.");
  
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
