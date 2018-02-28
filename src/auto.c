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
#include "utilities.h"
#include "pid.h"
#include "liftControl.h"

int tickGoal=0;

int rightError;
int rightPower;

int leftError;
int leftPower;

int turnError;
int turnPower;

int gyroValue;
int gyroOffset = 0;

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
    float ki = 0.02;
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
        rightPower =  limitMotorPower(pidNextIteration(&rightData, rightError));

        leftError = target - leftEncoderValue;
        leftPower =  limitMotorPower(pidNextIteration(&leftData, leftError));

        rightMotorsSet(rightPower);
        leftMotorsSet(leftPower);

        if((ABS(leftError)>60)||(ABS(rightError)>60)){
            T1 = millis();
        }

        delay(20);
    }
    stopChassis();
}

void baseTurn(float target, float power, float integralRange,
        bool leftToggle, bool rightToggle, float timeOut)
{
    const float kp = 2.5;
    const float ki = 0.2;
    const float kd = 9.0;
    /* const float kp = 0.5; */
    /* const float ki = 0.0; */
    /* const float kd = 0.0; */
    const float maxPower = power;

    PIDData data;
    pidDataInit(&data, kp, ki, kd, maxPower, 3600, integralRange);

    long T1,T2;
    T1 = millis();
    T2 = millis();

    timeOut = timeOut*1000;

    while ((T1 > (millis() - 350))&&(T2 > (millis() - timeOut))) {
        gyroValue = gyroGet(gyro) + gyroOffset;

        turnError = target - gyroValue;
        turnPower = limitMotorPower(pidNextIteration(&data, turnError));

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
    stopChassis ();
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
 * Drives until it senses a line
 */
/*
void driveLine(float power, float timeOut)
{
    long T1;
    T1 = millis();

    timeOut = timeOut*1000;

    while (T1 > (millis() - timeOut) && analogRead(LINE_CENTER_PORT) > 2000) {
        rightMotorsSet(power);
        leftMotorsSet(power);
    }

    rightMotorsSet(-SIGN(power));
    leftMotorsSet(-SIGN(power));
}
*/

void autonomous ()
{
    gyroReset(gyro);

    //NOW START!

    /*
    setLiftAngle(LIFT_UP);
    liftToggle = 1;
    delay(700);

    baseTurn(90, 80, 100, true, true, 5.0);
    delay(2000);

    baseTurn(45, 80, 100, true, true, 5.0);
    delay(2000);

    baseTurn(-20, 80, 100, true, true, 5.0);
    delay(2000);

    baseTurn(-135, 80, 100, true, true, 5.0);
    delay(2000);

    baseTurn(0, 80, 100, true, true, 5.0);

    return;
    */

    setLiftAngle(LIFT_DOWN);
    liftToggle = 1;
    delay(700);

    //FIRST BASE
    baseControl(56,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(1000);
    baseTurn(-26,90,300,true,true,1);
    delay(200);
    baseControl(-60,80,100,2.5);
    delay(200);
    baseTurn(-135,100,300,true,true,3);
    delay(200);
    driveTime(127,127,true,1.0);
    setLiftAngle(LIFT_HALF-200);
    delay(200);
    driveTime(-127,-127,false,0.4);
    delay(200);

/*     baseTurn(-135, 80, 300, true, true, 1.0); */
/*     delay(200); */
/*     driveTime(20, 20, true, 2.0); */
/*     gyroReset(gyro); */
/*     gyroOffset = -135; */

    /* baseControl(-8, 80, 30, 2.0); */

    //SECOND BASE
    delay(200);
    baseTurn(-45,100,300,true,true,3);
    delay(200);
    baseControl(-38,80,100,2.5);
    delay(200);
    driveTime(-100, -100, true, 0.75);

    gyroReset(gyro);

    return;

    baseTurn(45,100,300,true,true,3);
    delay(200);
    setLiftAngle(LIFT_DOWN);
    delay(500);
    baseControl(25,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(500);
    baseTurn(360 - 135,100,300,true,true,3);
    gyroOffset = (gyroOffset - 360) % 360;
    delay(200);
    baseControl(27,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_HALF);
    delay(200);
    baseControl(-8,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(200);

    //THIRD BASE
    baseTurn(-45,100,300,true,true,3);
    delay(200);
    baseControl(33,80,100,2.5);
    delay(200);
    baseTurn(45,100,300,true,true,3);
    delay(200);
    setLiftAngle(LIFT_DOWN);
    delay(600);
    baseControl(32,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(500);
    baseTurn(-135,100,300,true,true,3);
    delay(200);
    baseControl(28,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_HALF);
    delay(200);
    baseControl(-25,80,100,2.5);
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(200);

    //FOURTH BASE
    baseTurn(45,100,300,true,true,4);
    delay(200);
    setLiftAngle(LIFT_DOWN);
    delay(200);
    baseControl(55,100,100,2.5); // Problem child; need to tune to properly grab blue base
    delay(200);
    setLiftAngle(LIFT_UP);
    delay(200);
    baseTurn(10,80,300,true, true,0.5); // Cannot test these until problem child is fixed
    delay(200);
    baseControl(30,100,100,2.5);
    delay(200);
    setLiftAngle(LIFT_HALF-100);
    delay(200);
    baseControl(-10,100,100,2.5);
    delay(200);

    delay(2000);
}

