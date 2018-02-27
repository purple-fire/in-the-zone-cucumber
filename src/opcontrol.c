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

// H-Drive using 4 Inputs
void driveTrain(void *parameter) {
    while (true) {
        /* TODO limitMotorPower() is used so that the robot will brake when the
         * joy sticks are not pushed. This means that power will always be
         * applied to the motors and the robot will make the really annoying
         * humming sound. Something should probably be done so that if the robot
         * is already stationary, or the joystick has been stationary for a
         * certain amount of time the motors will be sent a power of 0.
         */
        rightMotorsSet(limitMotorPower(joystickGetAnalog(1, CRY) * 0.8));
        leftMotorsSet(limitMotorPower(joystickGetAnalog(1, CLY) * 0.8));
        // Motor values can only be updated every 20ms
        delay(20);
    }
}

void startAutoPilot(void *parameter) {
    stopChassis();
    delay(500);
    motorStopAll(); /* stopChassis() brakes all motors, this lets them rest */
    delay(20);
    autonomous();
}

void operatorControl() {
    TaskHandle autoPilotHandle;
    TaskHandle driveTrainHandle = taskCreate(driveTrain,
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
            driveTrainHandle = taskCreate(driveTrain,
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

