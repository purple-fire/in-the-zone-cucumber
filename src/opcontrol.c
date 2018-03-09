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

#include "liftControl.h"
#include "main.h"
#include "pid.h"
#include "utilities.h"
#include "motor.h"

typedef enum { DRIVE_AUTO, DRIVE_TANK, DRIVE_ARCADE } DriveMode;

DriveMode driveMode;

/**
 * Driver control with tank-style controls.
 */
static void driverControl(void *parameter) {
  chassisStop();

  while (true) {
    if (driveMode == DRIVE_TANK) {
      int joyRight = joystickGetAnalog(1, CRY);
      int joyLeft = joystickGetAnalog(1, CLY);

      if (ABS(joyRight) <= 10) {
        rightMotorsBrake();
      } else {
        rightMotorsSet(joyRight * MOTOR_POWER_MAX / 127);
      }

      if (ABS(joyLeft) <= 10) {
        leftMotorsBrake();
      } else {
        leftMotorsSet(joyLeft * MOTOR_POWER_MAX / 127);
      }

    } else if (driveMode == DRIVE_ARCADE) {
      int joyRight = joystickGetAnalog(1, CRX);
      int joyLeft = joystickGetAnalog(1, CLY);

      int leftPower = joyLeft + joyRight;
      int rightPower = joyLeft - joyRight;

      if (ABS(leftPower) <= 8) {
        leftMotorsBrake();
      } else {
        leftMotorsSet(leftPower * MOTOR_POWER_MAX / 127);
      }

      if (ABS(rightPower) <= 8) {
        rightMotorsBrake();
      } else {
        rightMotorsSet(rightPower * MOTOR_POWER_MAX / 127);
      }

    } else {
      /* Stop for invalid modes */
      chassisStop();
    }

    // Motor values can only be updated every 20ms
    delay(20);
  }
}

static void startAutoPilot(void *parameter) {
  chassisStopSmooth();
  autonomous();
}

void operatorControl() {
  TaskHandle autoPilotHandle, driverControlHandle;
  driveMode = DRIVE_ARCADE;
  driverControlHandle = taskCreate(driverControl, TASK_DEFAULT_STACK_SIZE, NULL,
                                   TASK_PRIORITY_DEFAULT);

  setLiftAngle(LIFT_UP);
  liftToggle = 1;

  while (true) {

    if (driveMode != DRIVE_AUTO && (joystickGetDigital(1, 8, JOY_RIGHT) == 1)) {
      taskDelete(driverControlHandle);
      autoPilotHandle = taskCreate(startAutoPilot, TASK_DEFAULT_STACK_SIZE,
                                   NULL, TASK_PRIORITY_DEFAULT);
      driveMode = DRIVE_AUTO;
    } else if (joystickGetDigital(1, 8, JOY_UP) == 1) {
      if (driveMode == DRIVE_AUTO) {
        taskDelete(autoPilotHandle);
        driverControlHandle = taskCreate(driverControl, TASK_DEFAULT_STACK_SIZE,
                                         NULL, TASK_PRIORITY_DEFAULT);
      }
      driveMode = DRIVE_TANK;
    } else if (joystickGetDigital(1, 8, JOY_DOWN) == 1) {
      if (driveMode == DRIVE_AUTO) {
        taskDelete(autoPilotHandle);
        driverControlHandle = taskCreate(driverControl, TASK_DEFAULT_STACK_SIZE,
                                         NULL, TASK_PRIORITY_DEFAULT);
      }
      driveMode = DRIVE_ARCADE;
    }

    if (driveMode != DRIVE_AUTO) {
      if (joystickGetDigital(1, 6, JOY_UP) == 1) {
        setLiftAngle(LIFT_UP);
      } else if (joystickGetDigital(1, 6, JOY_DOWN) == 1) {
        setLiftAngle(LIFT_DOWN);
      } else if (joystickGetDigital(1, 5, JOY_DOWN) == 1 || joystickGetDigital(1, 5, JOY_UP) == 1) {
        setLiftAngle(LIFT_HALF);
      }
    }

    delay(50);
  }
  taskDelete(autoPilotHandle);
  taskDelete(driverControlHandle);
}
