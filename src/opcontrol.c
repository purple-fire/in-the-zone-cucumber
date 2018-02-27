/**
 * File for operator control code
 *
 * This file should contain the user operatorControl() function and any
 * functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "utilities.h"
#include "pid.h"
#include "liftControl.h"

/**
 * Driver control with tank-style controls.
 */
static void driveTank(void *parameter) {
    while (true) {
        /* TODO limitMotorPower() is used so that the robot will brake when the
         * joy sticks are not pushed. This means that power will always be
         * applied to the motors and the robot will make the really annoying
         * humming sound. Something should probably be done so that if the robot
         * is already stationary, or the joystick has been stationary for a
         * certain amount of time the motors will be sent a power of 0.
         */
        rightMotorsSetSmooth(joystickGetAnalog(1, CRY) * MAX_POWER_OUT / 127);
        leftMotorsSetSmooth( joystickGetAnalog(1, CLY) * MAX_POWER_OUT / 127);
        // Motor values can only be updated every 20ms
        delay(20);
    }
}

/**
 * Driver control with archade-style controls.
 * /
static void driveArchade(void *parameter) {
    while (true) {
        rightMotorsSetSmooth(
                (joystickGetAnalog(1, CLY) + joystickGetAnalog(1, CRX))
                * MAX_POWER_OUT / 127);
        leftMotorsSetSmooth(
                (joystickGetAnalog(1, CLY) - joystickGetAnalog(1, CRX))
                * MAX_POWER_OUT / 127);

        delay(20);
    }
}
*/

static void startAutoPilot(void *parameter) {
    stopChassisSmooth();
    delay(500);
    autonomous();
}

void operatorControl() {
    TaskHandle autoPilotHandle;
    TaskHandle driveTrainHandle = taskCreate(driveTank,
            TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
    setLiftAngle(LIFT_UP);
    liftToggle = 1;

    int autoPilot = 0;
    while (true) {

        if((joystickGetDigital(1, 8, JOY_RIGHT) == 1)&&(autoPilot==0)){
            autoPilot = 1;
            taskDelete(driveTrainHandle);
            autoPilotHandle = taskCreate(startAutoPilot,
                    TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
        }
        else if((joystickGetDigital(1, 8, JOY_LEFT) == 1)&&(autoPilot==1)){
            autoPilot = 0;
            taskDelete(autoPilotHandle);
            driveTrainHandle = taskCreate(driveTank,
                    TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
        }
        if(autoPilot==0){
            if (joystickGetDigital(1, 6, JOY_UP) == 1) {
                setLiftAngle(LIFT_UP);
            } else if (joystickGetDigital(1, 6, JOY_DOWN) == 1) {
                setLiftAngle(LIFT_DOWN);
            } else if (joystickGetDigital(1, 5, JOY_DOWN) == 1) {
                setLiftAngle(LIFT_HALF);
            }
        }

        // Motor values can only be updated every 20ms
        delay(20);
    }
    taskDelete(driveTrainHandle);
}

