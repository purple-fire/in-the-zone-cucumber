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

#include <limits.h>

#include "main.h"
#include "utilities.h"
#include "pid.h"
#include "motor.h"
#include "liftControl.h"
#include "lineFollower.h"

int tickGoal=0;

int rightError;
int rightPower;

int leftError;
int leftPower;

int turnError;
int turnPower;

int gyroValue;

/* TODO Figure out a way to update these in a thread-safe fashion when
 * baseControl() is not running
 */
int rightEncoderValue;
int leftEncoderValue;

void baseControl(float target, float power, float integralRange, float timeOut)
{

    // Do PID on each side separately to help account for physical inaccuracies
    PIDData rightData;
    PIDData leftData;

    float kp = 0.25;
    float ki = 0.00; // 0.02
    float kd = 0.9;
    float maxPower = power;

    pidDataInit(&leftData, kp, ki, kd, maxPower, 32767, integralRange);
    pidDataInit(&rightData, kp, ki, kd, maxPower, 32767, integralRange);

    encoderReset(BLEncoder);
    encoderReset(BREncoder);

    target = INCHES_TO_TICKS(target);

    long T1,T2;
    T1 = millis();
    T2 = millis();

    timeOut = timeOut*1000;

    while ((T1 > (millis() - 350))&&(T2 > (millis() - timeOut))) {

        rightEncoderValue = encoderGet(BREncoder);
        leftEncoderValue = encoderGet(BLEncoder);

        rightError = target - rightEncoderValue;
        rightPower =  motorPowerLimit(pidNextIteration(&rightData, rightError));

        leftError = target - leftEncoderValue;
        leftPower =  motorPowerLimit(pidNextIteration(&leftData, leftError));

        rightMotorsSet(rightPower);
        leftMotorsSet(leftPower);

        if((ABS(leftError)>60)||(ABS(rightError)>60)){
            T1 = millis();
        }

        delay(20);
    }

    chassisStop();
}

void baseTurn(float target, float power, float integralRange,
        bool leftToggle, bool rightToggle, float timeOut)
{
    const float kp = 2.5;
    const float ki = 0.02; //0.02;
    const float kd = 9.0;
    /* const float kp = 0.5; */
    /* const float ki = 0.0; */
    /* const float kd = 0.0; */
    const float maxPower = power;

    PIDData data;
    pidDataInit(&data, kp, ki, kd, maxPower, 360, integralRange);

    int oldValue = INT_MAX;
    gyroValue = devgyroGet(&gyroDev);
    while (oldValue != gyroValue) {
        delay(50);
        oldValue = gyroValue;
        gyroValue = devgyroGet(&gyroDev);
    }

    long T1,T2;
    T1 = millis();
    T2 = millis();

    timeOut = timeOut*1000;

    while ((T1 > (millis() - 350))&&(T2 > (millis() - timeOut))) {
        gyroValue = devgyroGet(&gyroDev);

        turnError = target - gyroValue;
        turnPower = motorPowerLimit(pidNextIteration(&data, turnError));

        // TODO Should we check that each side moves the same amount and adjust
        // them afterwards if not?
        if(rightToggle){
            rightMotorsSet(turnPower);
        } else {
            rightMotorsSet(0);
        }

        if(leftToggle){
            leftMotorsSet(-turnPower);
        } else {
            leftMotorsSet(0);
        }

        if(ABS(turnError)>2){
            T1 = millis();
        }

        delay(20);
    }

    chassisStop ();
}

/**
 * Time-based drive function for when we don't care too much about distance
 * (when going over the bump and aligning along walls/pipes).
 */
void driveTime(float powerL, float powerR, bool coast, float timeOut)
{
    long T1;
    T1 = millis();

    timeOut = timeOut*1000;

    while (T1 > (millis() - timeOut)) {
        rightMotorsSet(powerR);
        leftMotorsSet(powerL);
    }

    if (coast) {
        rightMotorsSet(0);
        leftMotorsSet(0);
    } else {
        rightMotorsSet(-SIGN(powerR));
        leftMotorsSet(-SIGN(powerL));
    }
}

/**
 * Drives until it senses a obstacle
 */

void wallBump(int threshold, float power, float timeOut, int angle)
{
    long T1;
    T1 = millis();
    int distance = ULTRA_BAD_RESPONSE;
    timeOut = timeOut*1000;

    while (T1 > (millis() - timeOut)
            && (distance > threshold || distance == ULTRA_BAD_RESPONSE)) {
      distance = ultrasonicGet(sonar);
      rightMotorsSet(power);
      leftMotorsSet(power);
    }

    chassisStop();

    devgyroResetTo(&gyroDev, angle);
    delay(100);
}

void autonomous ()
{
    devgyroReset(&gyroDev);

    //NOW START!


    setLiftAngle(LIFT_UP);
    liftToggle = 1;
    delay(700);

/*
    baseTurn(90, 80, 10, true, true, 5.0);
    delay(2000);

    baseTurn(45, 80, 10, true, true, 5.0);
    delay(2000);

    baseTurn(-20, 80, 10, true, true, 5.0);
    delay(2000);

    baseTurn(-135, 80, 10, true, true, 5.0);
    delay(2000);

    baseTurn(0, 80, 10, true, true, 5.0);
    return;
    */

    setLiftAngle(LIFT_DOWN);
    liftToggle = 1;
    delay(700);

    //FIRST BASE
    /*
    baseControl(56,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(1000);
    baseTurn(-26,90,10,true,true,1);
    delay(100);
    baseControl(-60,80,10,2.5);
    delay(200);
    baseTurn(-135,10,10,true,true,2);
    delay(100);
    driveTime(127,127,true,1.0);
    setLiftAngle(LIFT_HALF-200);
    delay(100);
    driveTime(-127,-127,false,0.4);
    delay(100);
    */

    baseControl(56, 80, 10, 2.0);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(500);
    baseControl(-46, 80, 10, 2.0);
    delay(100);
    baseTurn(135, 10, 10, true, true, 2);
    delay(100);
    baseControl(18, 80, 10, 2.0);
    baseTurn(360 - 135, 10, 10, true, true, 2);
    devgyroOffset(&gyroDev, -360);
    delay(100);
    driveTime(127,127,true,1.0);
    setLiftAngle(LIFT_HALF-200);
    delay(500);
    driveTime(-127,-127,false,0.5);
    delay(100);

    //Bump Bar
    setLiftAngle(LIFT_UP);
    delay(500);
    wallBump(10,25,10,-135);
    delay(100);

    //SECOND BASE
    baseControl(-6,80,10,2.5);
    delay(200);
    baseTurn(-45,10,10,true,true,2);
    delay(100);
    baseControl(-23,80,10,2.5);
    delay(200);
    baseTurn(45,10,10,true,true,2);
    delay(100);
    setLiftAngle(LIFT_DOWN);
    delay(500);
    baseControl(19,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(500);
    baseTurn(360-135,10,10,true,true,2);
    devgyroOffset(&gyroDev, -360);
    delay(100);

    baseControl(19, 80, 10, 2.5);
    delay(100);

    wallBump(10, 40, 10, -135);
    delay(100);

    /*
    baseControl(25,80,10,2.5);
    delay(100);
    */
    setLiftAngle(LIFT_HALF);
    delay(100);
    baseControl(-8,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);

    //THIRD BASE
    baseTurn(-45,10,10,true,true,3);
    delay(100);
    baseControl(32,80,10,2.5);
    delay(100);
    baseTurn(45,10,10,true,true,3);
    delay(100);
    setLiftAngle(LIFT_DOWN);
    delay(500);
    baseControl(32,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(500);
    baseTurn(-135,10,10,true,true,3);
    delay(100);
    baseControl(23,80,10,2.5);
    delay(100);
    wallBump(10, 25, 10, -135);
    delay(100);
    setLiftAngle(LIFT_HALF);
    delay(100);
    baseControl(-25,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);

    //FOURTH BASE
    baseTurn(45,10,10,true,true,4);
    delay(100);
    setLiftAngle(LIFT_DOWN);
    delay(100);
    baseControl(62,10,10,2.5); // Problem child; need to tune to properly grab blue base
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);

    baseTurn(90, 30, 10, true, true, 2.0);
    delay(100);
    baseControl(20, 10, 10, 2.0);
    delay(100);
    baseTurn(45, 30, 10, true, true, 2.0);
    delay(100);

    driveTime(127,127,true,1.0);
    setLiftAngle(LIFT_HALF-200);
    delay(500);
    driveTime(-127,-127,false,0.5);
    delay(100);

    setLiftAngle(LIFT_UP);
    delay(500);

    wallBump(10, 25, 5.0, 45);
    delay(100);

    // FIFTH BASE
    baseControl(-6,80,10,2.5);
    delay(200);
    baseTurn(-45,10,10,true,true,2);
    delay(100);
    baseControl(-24,80,10,2.5);
    delay(100);
    baseTurn(-135,10,10,true,true,2);
    delay(100);
    setLiftAngle(LIFT_DOWN);
    delay(300);
    baseControl(19,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);
    baseTurn(45-360,10,10,true,true,2);
    devgyroOffset(&gyroDev, 360);
    delay(100);
    baseControl(20, 80, 10, 2.0);
    delay(100);
    wallBump(10, 30, 10, 45);
    delay(100);

    setLiftAngle(LIFT_HALF);
    delay(200);
    baseControl(-8,80,10,2.5);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);

    // SIXTH BASE
    baseTurn(135, 10, 10, true, true, 2);
    delay(100);
    baseControl(-46, 80, 10, 2.0);
    delay(200);

    baseTurn(360-90, 10, 10, true, true, 2.0);
    devgyroOffset(&gyroDev, -360);
    delay(100);
    setLiftAngle(LIFT_DOWN);
    baseControl(50, 80, 10, 2.0);
    setLiftAngle(LIFT_UP);
    delay(100);
    baseControl(-50, 80, 10, 2.0);
    delay(200);
    baseTurn(45, 10, 10, true, false, 2.0);
    delay(100);

    wallBump(10, 25, 10, 45);

    //Bump Bar
    setLiftAngle(LIFT_HALF);
    delay(200);
    baseControl(-10, 80, 10, 2.0);
    delay(100);
    setLiftAngle(LIFT_UP);
    delay(100);
    baseTurn(95, 10, 10, true, true, 2.0);
    delay(200);
    baseControl(-50, 80, 10, 2.0);

    delay(2000);
}

