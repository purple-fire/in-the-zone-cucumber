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
    motorSet(rightMotorF, MIN_POWER_OUT);
    motorSet(rightMotorR, MIN_POWER_OUT);
    motorSet(leftMotorF,  MIN_POWER_OUT);
    motorSet(leftMotorR,  MIN_POWER_OUT);
}

bool motorSetSmooth(unsigned char channel, int power) {
    int curPower = motorGet(channel);
    if (ABS(curPower - power) <= 1) {
        /* Due to integer rounding, the normal calculation will not always
         * converge to power when two are close together
         */
        motorSet(channel, power);
        return true;
    } else {
        int nextPower = curPower * (1.0 - SMOOTH_FACTOR)
                      + power * SMOOTH_FACTOR;
        motorSet(channel, nextPower);
        return nextPower == power;
    }
}

bool leftMotorsSetSmooth(int power) {
    return motorSetSmooth(leftMotorF, power)
        && motorSetSmooth(leftMotorR, power);
}

bool rightMotorsSetSmooth(int power) {
    return motorSetSmooth(rightMotorF, power)
        && motorSetSmooth(rightMotorR, power);
}

void stopChassisSmooth(void) {
    /* TODO Should MIN_POWER_OUT be used instead, like stopChassis()? */
    while (   !motorSetSmooth(rightMotorF, 0)
            || !motorSetSmooth(rightMotorR, 0)
            || !motorSetSmooth(leftMotorF, 0)
            || !motorSetSmooth(leftMotorR, 0)) {
        /* Just loop until all succede */
        delay(20);
    }
}

