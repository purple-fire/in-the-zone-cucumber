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

typedef enum {
    DRIVE_AUTO,
    DRIVE_TANK,
    DRIVE_ARCHADE
} DriveMode;

DriveMode driveMode;

/**
 * Driver control with tank-style controls.
 */
static void driveTank(void *parameter) {
    stopChassis();

    while (true) {
        if (driveMode == DRIVE_TANK) {
            int joyRight = joystickGetAnalog(1, CRY);
            int joyLeft = joystickGetAnalog(1, CLY);

            if (ABS(joyRight) <= 8) {
                rightMotorsBrake();
            } else {
                rightMotorsSet(joyRight * MAX_POWER_OUT / 127);
            }

            if (ABS(joyLeft) <= 8) {
                leftMotorsBrake();
            } else {
                leftMotorsSet(joyLeft * MAX_POWER_OUT / 127);
            }

        } else if (driveMode == DRIVE_ARCHADE) {
            int joyRight = joystickGetAnalog(1, CRX);
            int joyLeft = joystickGetAnalog(1, CLY);

            int leftPower  = joyLeft + joyRight;
            int rightPower = joyLeft - joyRight;

            if (ABS(leftPower) <= 8) {
                leftMotorsBrake();
            } else {
                leftMotorsSet(leftPower * MAX_POWER_OUT / 127);
            }

            if (ABS(rightPower) <= 8) {
                rightMotorsBrake();
            } else {
                rightMotorsSet(rightPower * MAX_POWER_OUT / 127);
            }

        } else {
            /* Stop for invalid modes */
            stopChassis();
        }

        // Motor values can only be updated every 20ms
        delay(20);
    }
}

static void startAutoPilot(void *parameter) {
    stopChassisSmooth();
    autonomous();
}

void operatorControl() {
    TaskHandle autoPilotHandle = NULL;
    TaskHandle driverControlHandle = taskCreate(driveTank,
            TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
    setLiftAngle(LIFT_UP);
    liftToggle = 1;

    driveMode = DRIVE_TANK;

    while (true) {

        if (driveMode != DRIVE_AUTO
                && joystickGetDigital(1, 8, JOY_RIGHT)){
            driveMode = DRIVE_AUTO;
            taskDelete(driverControlHandle);
            driverControlHandle = NULL;
            autoPilotHandle = taskCreate(startAutoPilot,
                    TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
        } else if (joystickGetDigital(1, 8, JOY_UP)) {
            if (driveMode == DRIVE_AUTO) {
                taskDelete(autoPilotHandle);
                autoPilotHandle = NULL;
                driverControlHandle = taskCreate(driveTank,
                        TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
            }

            driveMode = DRIVE_TANK;
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            if (driveMode == DRIVE_AUTO) {
                taskDelete(autoPilotHandle);
                autoPilotHandle = NULL;
                driverControlHandle = taskCreate(driveTank,
                        TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
            }

            driveMode = DRIVE_ARCHADE;
        }

        if (driveMode != DRIVE_AUTO) {
            if (joystickGetDigital(1, 6, JOY_UP)) {
                setLiftAngle(LIFT_UP);
            } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
                setLiftAngle(LIFT_DOWN);
            } else if (joystickGetDigital(1, 5, JOY_DOWN)) {
                setLiftAngle(LIFT_HALF);
            }
        }

        delay(50);
    }
}

