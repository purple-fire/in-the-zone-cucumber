/**
 * Line following functions.
 */

#include "API.h"
#include "main.h"
#include "utilities.h"
#include "lineFollower.h"
#include "motor.h"

int lineThreshold = 0;

void driveToLine(int power, float timeOut)
{
    long T1;
    T1 = millis();

    timeOut = timeOut*1000;

    while (T1 > (millis() - timeOut) && !IS_LINE(LINE_CENTER_PORT)) {
        rightMotorsSet(power);
        leftMotorsSet(power);
        delay(20);
    }

    chassisBrake();
}

void followLineTime(int power, float time) {
    long T1;
    T1 = millis();

    time *= 1000;

    while (T1 > millis() - time) {
        int rightPower = power;
        int leftPower = power;

        bool rightOn = analogReadCalibrated(LINE_RIGHT_PORT);
        bool centerOn = analogReadCalibrated(LINE_CENTER_PORT);
        bool leftOn = analogReadCalibrated(LINE_LEFT_PORT);

        if (!centerOn) {
            if (rightOn) {
                leftPower *= 1.1;
                rightPower *= 1.0 / 1.1;
            } else if (leftOn) {
                leftPower *= 1.0 / 1.1;
                rightPower *= 1.0;
            }
        }

        delay(20);
    }
}

void followLine(int inches, float timeOut) {

}

/* vim: set tw=80 ft=c: */
