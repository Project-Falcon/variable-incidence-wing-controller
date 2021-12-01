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
  Serial.begin(115200);
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
  stepper.moveTo(event.orientation.y * 100);
  /*
  Serial.print("Y: ");
  Serial.print(event.orientation.y);
  Serial.print(" | distanceToGo: ");
  Serial.print(stepper.distanceToGo());
  Serial.print(" | targetPosition: ");
  Serial.print(stepper.targetPosition());
  Serial.print(" | currentPosition: ");
  Serial.print(stepper.currentPosition());
  Serial.print(" | speed: ");
  Serial.print(stepper.speed());
  Serial.println();
  */

  Serial.println(stepper.distanceToGo());

  
  
  
  if (abs(stepper.distanceToGo()) > 100) {
    for (int i = 0; i <= 100; i++) {
      stepper.run();
    }
    Serial.println("Stepping");
  }

  //stepper.moveTo(8000000);
  //stepper.runToPosition();
}