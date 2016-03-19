/*******************************************************************
 *
 *      No need to modify this file     *
 *
 *******************************************************************/

#include <ExtraCore.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

#include <NewPing.h>

#define SONAR_NUM     4 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(10, 12, MAX_DISTANCE),
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE)
};
 
ExtraCore extraCore;

void setup()
{
  //Serial.begin(9600);  
  extraCore.onReceive(onRecieve);
  extraCore.beginClient();
  //Serial.println("client ready");
  
  //Ultrasonic
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() 
{
  //Ultrasonic
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      //if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  
  
  //Send data 50 times a second.
  static long lastUpdate = 0;
  if(lastUpdate + 20 < millis())
  {  
    lastUpdate = millis();
    getDigitalData();
    getAnalogData();
    extraCore.sendData();
    delay(1);
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) {
    cm[currentSensor] = sonar[currentSensor].ping_result * 10 / US_ROUNDTRIP_CM;
  }
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

// Process incoming data.
void onRecieve()
{
    setPinModes();
    setIOstates();
}

//Set the local pins to the desired INPUT/OUTPUT
void setPinModes()
{
  for(int i = 0; i < IOPINCOUNT; i++)
  {
    // A4 and A5 are reserved for i2c
    if(i == A4 || i == A5) { continue; }
    pinMode(i, extraCore.getPinIOstate(i));
    digitalWrite(i, extraCore.getTriStateValue(i));
  }
}

//Set local I/O to the desired values.
void setIOstates()
{
  for(int i = 0; i < IOPINCOUNT; i++)
  {
    // A4 and A5 are reserved for i2c
    if(i == A4 || i == A5) { continue; }
    // If pin is output and not PWM
    if(extraCore.getPinIOstate(i)  &&  extraCore.getAnalogValue(i) == 0)
    {
      digitalWrite(i, extraCore.getOutputValue(i));
    }

    // If pin is output and PWM
    if(extraCore.getPinIOstate(i) && extraCore.getAnalogValue(i))
    {
      analogWrite(i, extraCore.getAnalogValue(i));
    }
  }
}

//Gather digital input values
void getDigitalData()
{
  for(int i = 0; i < IOPINCOUNT; i++)
  {
    // A4 and A5 are reserved for i2c
    if(i == A4 || i == A5) { continue; }
    if(extraCore.getPinIOstate(i) == INPUT)
    {
      extraCore.setDigitalReading(i, digitalRead(i));
    }
  }
}

// Gather analog data
void getAnalogData()
{
  
  extraCore.setAnalogReading(A0, (cm[0]-cm[2]));
  extraCore.setAnalogReading(A1, cm[1]);
  extraCore.setAnalogReading(A2, cm[3]);
  /*for(int i = A0; i < A6 + 1; i++)
  {
    // A4 and A5 are reserved for i2c
    if(i == A4 || i == A5 || i== A0) { continue; }
    if(extraCore.getPinIOstate(i) == INPUT)
    {
      extraCore.setAnalogReading(i, analogRead(i));
    }  
  }*/
}
