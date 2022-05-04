#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
#define stepsPerRevolution 5

        int upperDelayTimeLimit;
        int lowerDelayTimeLimit;
        int delayTimeRange;
        int delayTimeGradient;
        int DistanceToGo;
        long delayTime;


AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin) maybe?

void setup(void)
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  delayTime = 500;
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");
       
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
  if (counter % 250 == 0)
  {
    bno.getEvent(&event);
    targetPos = -(event.orientation.y - setPoint);

    //  units to be calibrated for the targetPos equation
    // Negative feedback law used to remove offset from the setpoint

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
      // set the motor direction
      if((targetPos-stepper.currentPosition())>0)
      {
           digitalWrite(dirPin, LOW);
      }
      if((targetPos-stepper.currentPosition())<0)
      {
           digitalWrite(dirPin, HIGH);
      }

        // set delayTime to set motor speed - add sensor derivative term into the speed equation?

        upperDelayTimeLimit = 500;
        lowerDelayTimeLimit = 200;
        delayTimeRange = upperDelayTimeLimit - lowerDelayTimeLimit;
        delayTimeGradient = delayTimeRange/360;// does the motor give angle values > +-360?

        DistanceToGo = targetPos-stepper.currentPosition();


        if(DistanceToGo>0)
           {
              delayTime = -delayTimeGradient*DistanceToGo+upperDelayTimeLimit;
           }

        if(DistanceToGo<0)
           {
              delayTime = delayTimeGradient*DistanceToGo+upperDelayTimeLimit;
           }


        // rotate the motor

        digitalWrite(stepPin, HIGH);
        delay(delayTime);
        digitalWrite(stepPin, LOW);
        delay(delayTime);

  }
  ++counter;
}
