#include <ExtraCore.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

/*******************************************************************
 * No need to modify this *
 *******************************************************************/
 
ExtraCore extraCore;

void setup()
{
  //Serial.begin(9600);  
  extraCore.onReceive(onRecieve);
  extraCore.beginClient();
  //Serial.println("client ready");
}

void loop() 
{
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
  for(int i = A0; i < A6 + 1; i++)
  {
    // A4 and A5 are reserved for i2c
    if(i == A4 || i == A5) { continue; }
    if(extraCore.getPinIOstate(i) == INPUT)
    {
      extraCore.setAnalogReading(i, analogRead(i));
    }  
  }
}
