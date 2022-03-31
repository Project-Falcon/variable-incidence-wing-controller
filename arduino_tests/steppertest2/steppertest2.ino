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
// AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin) maybe?

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(5000);
  // add setMinPulseWidth()?

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

int counter = 0;
int setPoint = -15; //  units to be calibrated
int targetPos = 0;
sensors_event_t event;


void loop(void)
{
  if (counter % 500 == 0)
  {
    bno.getEvent(&event);
    targetPos = -(event.orientation.y - setPoint);
    // targetPos = -(event.orientation.y);
    //  units to be calibrated
    // Negative feedback law used to remove offset from the setpoint
    stepper.moveTo(targetPos*100);
    Serial.print("currentPosition: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" | Target Position ");
    Serial.print(targetPos);
    Serial.print(" | distanceToGo: ");
    Serial.print(stepper.distanceToGo());
    Serial.println();
  }

  if (abs(stepper.distanceToGo()) > 60)
  {
    stepper.run();
    // try runToNewPosition()
    // or runToPosition()
    // runSpeed()
  }
  ++counter;
}
