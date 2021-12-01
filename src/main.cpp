#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  stepper.setMaxSpeed(1000);  
  stepper.setAcceleration(500);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("Y: ");
  Serial.println(event.orientation.y);

  stepper.moveTo(event.orientation.y);
  stepper.runToPosition(); // this function is blocking, worth considering 
  delay(500);
}