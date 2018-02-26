/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions
 * related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user autonomous code. This function will be started in its own task
 * with the default
 * priority and stack size whenever the robot is enabled via the Field
 * Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or
 * communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the
 * robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX
 * Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition
 * Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never
 * exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
#include "utilities.h"
#include "liftControl.h"
#include "pid.h"
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

int tickGoal = 0;

int rightError;
int rightPower;

int leftError;
int leftPower;

int turnError;
int turnPower;

int gyroValue;

#define MAX_POWER_OUT 127
#define MIN_POWER_OUT 10

void stopMotors() {
  motorSet(-rightMotorF, 10);
  motorSet(-rightMotorR, 10);
  motorSet(leftMotorF, 10);
  motorSet(leftMotorR, 10);
}

int inchToTicks(float inch) {
  float wheelCircumference = wheelDiameter * Pi;
  int ticks = 360 / wheelCircumference;
  tickGoal = ticks * inch;
  return tickGoal;
}

//>>>>>>>>>>>>>>>>>>>>>>>>------------------0-----------------<<<<<<<<<<<<<<<<<<<<<<<<<

void baseControl(float target, float power, float integralRange,
                 float timeOut) {

  // Do PID on each side separately to help account for physical inaccuracies
  PIDData rightData;
  PIDData leftData;

  float kp = 0.25;
  float ki = 0.02;
  float kd = 0.9;
  float maxPower = power;

  // For 12 inches
  // P 0.2
  // I 0.001

  pidDataInit(&leftData, kp, ki, kd, maxPower, 32767, integralRange);
  pidDataInit(&rightData, kp, ki, kd, maxPower, 32767, integralRange);

  encoderReset(BLEncoder);
  encoderReset(BREncoder);

  target = inchToTicks(target);

  long T1, T2;
  T1 = millis();
  T2 = millis();

  timeOut = timeOut * 1000;

  while ((T1 > (millis() - 350)) && (T2 > (millis() - timeOut))) {

    rightError = encoderGet(BREncoder) - target;
    rightPower = limitMotorPower(pidNextIteration(&rightData, rightError));

    leftError = encoderGet(BLEncoder) - target;
    leftPower = limitMotorPower(pidNextIteration(&leftData, leftError));

    motorSet(rightMotorF, rightPower);
    motorSet(rightMotorR, rightPower);
    motorSet(leftMotorF, -leftPower);
    motorSet(leftMotorR, -leftPower);

    if ((ABS(leftError) > 60) || (ABS(rightError) > 60)) {
      T1 = millis();
    }

    delay(20);
  }
  stopMotors();
}

void baseTurn(float target, float power, float integralRange, bool leftToggle,
              bool rightToggle, float timeOut) {

  float kp = 0.25;
  float ki = 0.02;
  float kd = 0.9;
  float maxPower = power;

  PIDData data;
  pidDataInit(&data, kp, ki, kd, maxPower, 3600, integralRange);

  long T1, T2;
  T1 = millis();
  T2 = millis();

  timeOut = timeOut * 1000;

  while ((T1 > (millis() - 350)) && (T2 > (millis() - timeOut))) {
    gyroValue = gyroGet(gyro);

    turnError = gyroValue - target;

    // turnPower =  limitMotorPower(pidNextIteration(&data, turnError));
    turnPower = pidNextIteration(&data, turnError);

    // TODO Should we check that each side moves the same amount and adjust them
    // afterwards if not?
    if (rightToggle) {
      motorSet(rightMotorF, turnPower);
      motorSet(rightMotorR, turnPower);
    }
    if (leftToggle) {
      motorSet(leftMotorF, turnPower);
      motorSet(leftMotorR, turnPower);
    }

    if ((ABS(turnError) > 10)) {
      T1 = millis();
    }
    delay(20);
  }
  stopMotors();
}

void wallTurn(float target, float power, bool leftToggle, bool rightToggle) {
  gyroReset(gyro);
  gyroValue = gyroGet(gyro);

  while (ABS(gyroValue) < target) {
    turnError = gyroValue - target;
    gyroValue = gyroGet(gyro);

    if (rightToggle) {
      motorSet(rightMotorF, -power);
      motorSet(rightMotorR, -power);
    }
    if (leftToggle) {
      motorSet(leftMotorF, power);
      motorSet(leftMotorR, power);
    }
  }
  if (rightToggle) {
    motorSet(rightMotorF, 10);
    motorSet(rightMotorR, -10);
  }
  if (leftToggle) {
    motorSet(leftMotorF, -10);
    motorSet(leftMotorR, 10);
  }
  delay(200);
}

void driveTime(float powerL, float powerR, float timeOut) {
  long T1;
  T1 = millis();

  timeOut = timeOut * 1000;

  while (T1 > (millis() - timeOut)) {
    motorSet(rightMotorF, -powerR);
    motorSet(rightMotorR, -powerR);
    motorSet(leftMotorF, powerL);
    motorSet(leftMotorR, powerL);
  }
  stopMotors();
}

void autonomous() {
  // Initialize the gryo
  gyro = gyroInit(GYRO_PORT, 102);
  delay(2000);

  // Init Encoders
  BLEncoder = encoderInit(QUAD_TOP_PORT_LEFT, QUAD_BOTTOM_PORT_LEFT, false);
  BREncoder = encoderInit(QUAD_TOP_PORT_RIGHT, QUAD_BOTTOM_PORT_RIGHT, false);

  setLiftAngle(liftDown);
  liftToggle = 1;
  delay(700);

  // NOW START!

  // FIRST BASE
  baseControl(56, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftUp);
  delay(1000);
  baseTurn(-25, 90, 300, true, true, 1);
  delay(200);
  baseControl(-70, 80, 100, 2.5);
  delay(200);
  baseTurn(-135, 100, 300, true, true, 3);
  delay(200);
  driveTime(127, 127, 1.2);
  delay(200);
  setLiftAngle(liftHalf - 100);
  delay(200);
  driveTime(-127, -127, 0.5);
  delay(200);

  // SECOND BASE
  baseTurn(-45, 100, 300, true, true, 3);
  delay(200);
  baseControl(-15, 80, 100, 2.5);
  delay(200);
  baseTurn(45, 100, 300, true, true, 3);
  delay(200);
  setLiftAngle(liftDown);
  delay(200);
  baseControl(40, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftUp);
  delay(200);
  baseTurn(-132.5, 100, 300, true, true, 3);
  delay(200);
  baseControl(45, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftHalf);
  delay(200);
  baseControl(-8, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftUp);
  delay(200);

  // THIRD BASE
  baseTurn(-45, 100, 300, true, true, 3);
  delay(200);
  baseControl(33, 80, 100, 2.5);
  delay(200);
  baseTurn(45, 100, 300, true, true, 3);
  delay(200);
  setLiftAngle(liftDown);
  delay(600);
  baseControl(33, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftUp);
  delay(500);
  baseTurn(-135, 100, 300, true, true, 3);
  delay(200);
  baseControl(45, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftHalf);
  delay(200);
  baseControl(-25, 80, 100, 2.5);
  delay(200);
  setLiftAngle(liftUp);
  delay(200);

  // FOURTH BASE
  baseTurn(45, 100, 300, true, true, 4);
  delay(200);
  setLiftAngle(liftDown);
  delay(200);
  baseControl(55, 100, 100,
              2.5); // Problem child; need to tune to properly grab blue base
  delay(200);
  setLiftAngle(liftUp);
  delay(200);
  baseTurn(10, 80, 300, true, true,
           0.5); // Cannot test these until problem child is fixed
  delay(200);
  baseControl(30, 100, 100, 2.5);
  delay(200);
  setLiftAngle(liftHalf - 100);
  delay(200);
  baseControl(-10, 100, 100, 2.5);
  delay(200);

  delay(2000);
}
