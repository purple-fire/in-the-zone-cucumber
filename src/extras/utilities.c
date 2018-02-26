/**
 * Assorted utilities
 */

#include "main.h"
#include "utilities.h"

int limitMotorPower(int power)
{
    int	outputPower = ABS(power);

    if(outputPower > MAX_POWER_OUT)
    {
        outputPower = MAX_POWER_OUT;
    }
    else if(outputPower < MIN_POWER_OUT)
    {
        outputPower = MIN_POWER_OUT;
    }

    return SIGN(power) * outputPower;
}

/**
 * Set the power of both left motors.
 * Positive is forwards, negative is reverse.
 */
void leftMotorsSet(int power) {
    motorSet(leftMotorF, power);
    motorSet(leftMotorR, power);
}

/**
 * Sets the power of both right motors.
 * Positive is forwards, negative is reverse.
 */
void rightMotorsSet(int power) {
    /* Sign of right motors is switched */
    motorSet(rightMotorF, -power);
    motorSet(rightMotorR, -power);
}

void stopChassis(void) {
    motorStop(rightMotorF);
    motorStop(rightMotorR);
    motorStop(leftMotorF);
    motorStop(leftMotorR);
}

