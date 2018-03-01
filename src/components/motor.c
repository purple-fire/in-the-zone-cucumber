/**
 * Wrapper structure for motors.
 */

#include "API.h"
#include "motor.h"
#include "utilities.h"

int motorPowerLimit(int power)
{
    int	outputPower = ABS(power);

    if(outputPower > MOTOR_POWER_MAX)
    {
        outputPower = MOTOR_POWER_MAX;
    }
    else if(outputPower < MOTOR_POWER_MIN)
    {
        outputPower = MOTOR_POWER_MIN;
    }

    return SIGN(power) * outputPower;
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
        int nextPower = curPower * (1.0 - MOTOR_SMOOTH_FACTOR)
            + power * MOTOR_SMOOTH_FACTOR;
        motorSet(channel, nextPower);
        return nextPower == power;
    }
}

void motorBrake(unsigned char channel) {
    motorSet(channel, -SIGN(motorGet(channel)));
}

void motorBrakeSmooth(unsigned char channel) {
    while (!motorSetSmooth(channel, -SIGN(motorGet(channel)))) {
        delay(20);
    }
}

void leftMotorsSet(int power) {
    motorSet(leftMotorF, power);
    motorSet(leftMotorR, power);
}

void rightMotorsSet(int power) {
    /* Sign of right motors is switched */
    motorSet(rightMotorF, -power);
    motorSet(rightMotorR, -power);
}

void chassisStop(void) {
    motorStop(rightMotorF);
    motorStop(rightMotorR);
    motorStop(leftMotorF);
    motorStop(leftMotorR);
}

void rightMotorsBrake(void) {
    motorBrake(rightMotorF);
    motorBrake(rightMotorR);
}

void leftMotorsBrake(void) {
    motorBrake(leftMotorF);
    motorBrake(leftMotorR);
}

void chassisBrake(void) {
    motorBrake(rightMotorF);
    motorBrake(rightMotorR);
    motorBrake(leftMotorF);
    motorBrake(leftMotorR);
}

bool leftMotorsSetSmooth(int power) {
    return motorSetSmooth(leftMotorF, power)
        && motorSetSmooth(leftMotorR, power);
}

bool rightMotorsSetSmooth(int power) {
    return motorSetSmooth(rightMotorF, -power)
        && motorSetSmooth(rightMotorR, -power);
}

void chassisStopSmooth(void) {
    /* TODO Should MOTOR_POWER_MIN be used instead, like chassisStop()? */
    while (    !motorSetSmooth(rightMotorF, 0)
            || !motorSetSmooth(rightMotorR, 0)
            || !motorSetSmooth(leftMotorF, 0)
            || !motorSetSmooth(leftMotorR, 0)) {
        /* Just loop until all succede */
        delay(20);
    }
}

void rightMotorsStopSmooth(void) {
    while (!rightMotorsSetSmooth(0)) {
        delay(20);
    }
}

void leftMotorsStopSmooth(void) {
    while (!leftMotorsSetSmooth(0)) {
        delay(20);
    }
}

/* vim: set tw=80 ft=c: */
