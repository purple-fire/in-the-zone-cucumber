/**
 * Functions for controlling the lift, during both driver control and
 * autonomous.
 */

#include "liftControl.h"
#include "pid.h"
#include "utilities.h"
#include "motor.h"

int liftToggle = false;
int desiredLiftAngle = LIFT_UP;
int liftPosition;

void liftControl(void *parameter)
{
    // Separate variables to simplify tuning
    const float kp = 0.15;
    const float ki = 0.010; //0.7;
    const float kd = 0.0;

    PIDData data;
    pidDataInit(&data, kp, ki, kd, 125, 4095, 150);

    while (true)
    {
        if (liftToggle)
        {
            liftPosition = analogRead(POTENTIOMETER_PORT);
            int errorLiftAngle = desiredLiftAngle - liftPosition;
            int liftPowerOut = pidNextIteration(&data, errorLiftAngle);

            motorSet (liftMotor,liftPowerOut);
        }
        else
        {
            motorStop(liftMotor);
        }

        delay(20);
    }
}

void setLiftAngle(int liftAngle)
{
    desiredLiftAngle = liftAngle;
}
