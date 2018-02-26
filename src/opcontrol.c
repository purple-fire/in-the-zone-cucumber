/** @file opcontrol.c
 * @brief File for operator control code
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

/*
 * Runs the user operator control code. This function will be started in its own
 * task with the
 * default priority and stack size whenever the robot is enabled via the Field
 * Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is
 * disabled or
 * communications is lost, the operator control task will be stopped by the
 * kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the
 * VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX
 * Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX
 * Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is
 * available and
 * the scheduler is operational. However, proper use of delay() or
 * taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating
 * LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop,
 * even if empty.
 */
#include "utilities.h"
#include "pid.h"
#include "liftControl.h"

// ROBOT CONFIG //
// Define Motors
#define rightMotorR 2
#define rightMotorF 3
#define liftMotor 4
#define leftMotorR 7
#define leftMotorF 8

// Define Sensors
#define GYRO_PORT 8
Gyro gyro;

#define QUAD_TOP_PORT_LEFT 1
#define QUAD_BOTTOM_PORT_LEFT 2
#define QUAD_TOP_PORT_RIGHT 3
#define QUAD_BOTTOM_PORT_RIGHT 4
Encoder BLEncoder;
Encoder BREncoder;

#define POTENTIOMETER_PORT 2
//////////////////

// H-Drive using 4 Inputs
void driveTrain(void *parameter) {
  while (true) {
    motorSet(rightMotorF, -(joystickGetAnalog(1, CRY)) * 0.8);
    motorSet(leftMotorF, (joystickGetAnalog(1, CLY)) * 0.8);
    motorSet(rightMotorR, -(joystickGetAnalog(1, CRY)) * 0.8);
    motorSet(leftMotorR, (joystickGetAnalog(1, CLY)) * 0.8);
    // Motor values can only be updated every 20ms
    delay(20);
  }
}

void stopChassis() {
    motorSet(rightMotorF, 10);
    motorSet(leftMotorF, 10);
    motorSet(rightMotorR, 10);
    motorSet(leftMotorR, 10);
}

void startAutoPilot(void *parameter) {
  stopChassis();
  delay(500);
  autonomous();
}

void operatorControl() {
  TaskHandle autoPilotHandle;
  TaskHandle driveTrainHandle = taskCreate(driveTrain, TASK_DEFAULT_STACK_SIZE,
                                           NULL, TASK_PRIORITY_DEFAULT);
  setLiftAngle(liftUp);
  liftToggle = 1;

  int autoPilot = 0;
  while (1) {

    if((joystickGetDigital(1, 8, JOY_RIGHT) == 1)&&(autoPilot==0)){
      autoPilot = 1;
      taskDelete(driveTrainHandle);
      autoPilotHandle = taskCreate(startAutoPilot, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
    }
    else if((joystickGetDigital(1, 8, JOY_LEFT) == 1)&&(autoPilot==1)){
      autoPilot = 0;
      taskDelete(autoPilotHandle);
      driveTrainHandle = taskCreate(driveTrain, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
    }
    if(autoPilot==0){
    if (joystickGetDigital(1, 6, JOY_UP) == 1) {
      setLiftAngle(liftUp);
    } else if (joystickGetDigital(1, 6, JOY_DOWN) == 1) {
      setLiftAngle(liftDown);
    } else if (joystickGetDigital(1, 5, JOY_DOWN) == 1) {
      setLiftAngle(liftHalf);
    }
  }


    // Motor values can only be updated every 20ms
    delay(20);
  }
    taskDelete(driveTrainHandle);
}
