/**
 * Functions for controlling the lift, during both driver control and
 * autonomous.
 */

#include "liftControl.h"
#include "pid.h"
#include "utilities.h"

void liftControl(void *parameter)
{
    // Separate variables to simplify tuning
    const float kp = 0.2;
    const float ki = 0.001; //0.7;
    const float kd = 0.0;

    PIDData data;
    pidDataInit(&data, kp, ki, kd, 125,4095, 50);

    while (true)
    {
        if (liftToggle==1)
        {
            int errorLiftAngle = desiredLiftAngle - analogRead(POTENTIOMETER_PORT);
            int liftPowerOut = limitMotorPower(pidNextIteration(&data, errorLiftAngle));

            motorSet (liftMotor,liftPowerOut);
        }
        else
        {
            motorSet (liftMotor,0);
        }

        delay(20);
    }
}

void setLiftAngle(int liftAngle)
{
    desiredLiftAngle = liftAngle;
}

