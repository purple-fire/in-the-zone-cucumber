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

/* Globals so that values can be seen from outside functions.
 * These may be obsolete, sincne PROS does not have a built-in monitor of
 * globals.
 */

int tickGoal=0;

int rightError;
int rightPower;

int leftError;
int leftPower;

int turnError;
int turnPower;

int gyroValue;

void stopMotors ()
{
    motorStop(rightMotorF);
    motorStop(rightMotorR);
    motorStop(leftMotorF);
    motorStop(leftMotorR);
}

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

        rightError = encoderGet(BREncoder) - target;
        rightPower =  limitMotorPower(pidNextIteration(&rightData, rightError));

        leftError = encoderGet(BLEncoder) - target;
        leftPower =  limitMotorPower(pidNextIteration(&leftData, leftError));

        motorSet (rightMotorF,-rightPower);
        motorSet (rightMotorR,-rightPower);
        motorSet (leftMotorF,-leftPower);
        motorSet (leftMotorR,-leftPower);

        if((ABS(leftError)>60)||(ABS(rightError)>60)){
            T1 = millis();
        }

        delay(20);
    }
    stopMotors();
}

void baseTurn(float target, float power, float integralRange,
        bool leftToggle, bool rightToggle, float timeOut)
{

    float kp = 0.25;
    float ki = 0.02;
    float kd = 0.9;
    float maxPower = power;

    PIDData data;
    pidDataInit(&data, kp, ki, kd, maxPower, 3600, integralRange);

    long T1,T2;
    T1 = millis();
    T2 = millis();

    timeOut = timeOut*1000;

    while ((T1 > (millis() - 350))&&(T2 > (millis() - timeOut))) {
        gyroValue = gyroGet(gyro);

        turnError = gyroValue - target;

        turnPower = limitMotorPower(pidNextIteration(&data, turnError));

        // TODO Should we check that each side moves the same amount and adjust
        // them afterwards if not?
        if(rightToggle){
            motorSet (rightMotorF,-turnPower);
            motorSet (rightMotorR,-turnPower);
        }

        if(leftToggle){
            motorSet (leftMotorF,turnPower);
            motorSet (leftMotorR,turnPower);
        }

        if((ABS(turnError)>10)){
            T1 = millis();
        }

        delay(20);
    }
    stopMotors ();
}

/**
 * Obsolete time-based turn function
 * TODO Remove
 */
void wallTurn(float target, float power, bool leftToggle, bool rightToggle)
{
    gyroReset (gyro);
    gyroValue = gyroGet(gyro);

    while (ABS(gyroValue)<target) {
        turnError = gyroValue - target;
        gyroValue = gyroGet(gyro);

        if(rightToggle){
            motorSet (rightMotorF,-power);
            motorSet (rightMotorR,-power);
        }
        if(leftToggle){
            motorSet (leftMotorF,power);
            motorSet (leftMotorR,power);
        }
    }
    if(rightToggle){
        motorSet (rightMotorF,10);
        motorSet (rightMotorR,10);
    }
    if(leftToggle){
        motorSet (leftMotorF,-10);
        motorSet (leftMotorR,-10);
    }
    delay(200);
}

/**
 * Time-based drive function for when we don't care too much about distance
 * (when going over the bump and aligning along walls/pipes).
 */
void driveTime(float powerL, float powerR, float timeOut)
{
    long T1;
    T1 = millis();

    timeOut = timeOut*1000;

    while (T1 > (millis() - timeOut)) {
        motorSet (rightMotorF,powerR);
        motorSet (rightMotorR,powerR);
        motorSet (leftMotorF,powerL);
        motorSet (leftMotorR,powerL);
    }
    stopMotors ();
}

void autonomous ()
{
    setLiftAngle(liftDown);
    liftToggle = 1;
    delay(700);

    //NOW START!

    //FIRST BASE
    baseControl(56,80,100,2.5);
    delay(200);
    setLiftAngle(liftUp);
    delay(1000);
    baseTurn(-25,90,300,true,true,1);
    delay(200);
    baseControl(-70,80,100,2.5);
    delay(200);
    baseTurn(-135,100,300,true,true,3);
    delay(200);
    driveTime(127,127,1.2);
    delay(200);
    setLiftAngle(liftHalf-100);
    delay(200);
    driveTime(-127,-127,0.5);
    delay(200);

    //SECOND BASE
    baseTurn(-45,100,300,true,true,3);
    delay(200);
    baseControl(-15,80,100,2.5);
    delay(200);
    baseTurn(45,100,300,true,true,3);
    delay(200);
    setLiftAngle(liftDown);
    delay(200);
    baseControl(40,80,100,2.5);
    delay(200);
    setLiftAngle(liftUp);
    delay(200);
    baseTurn(-132.5,100,300,true,true,3);
    delay(200);
    baseControl(45,80,100,2.5);
    delay(200);
    setLiftAngle(liftHalf);
    delay(200);
    baseControl(-8,80,100,2.5);
    delay(200);
    setLiftAngle(liftUp);
    delay(200);

    //THIRD BASE
    baseTurn(-45,100,300,true,true,3);
    delay(200);
    baseControl(33,80,100,2.5);
    delay(200);
    baseTurn(45,100,300,true,true,3);
    delay(200);
    setLiftAngle(liftDown);
    delay(600);
    baseControl(33,80,100,2.5);
    delay(200);
    setLiftAngle(liftUp);
    delay(500);
    baseTurn(-135,100,300,true,true,3);
    delay(200);
    baseControl(45,80,100,2.5);
    delay(200);
    setLiftAngle(liftHalf);
    delay(200);
    baseControl(-25,80,100,2.5);
    delay(200);
    setLiftAngle(liftUp);
    delay(200);

    //FOURTH BASE
    baseTurn(45,100,300,true,true,4);
    delay(200);
    setLiftAngle(liftDown);
    delay(200);
    baseControl(55,100,100,2.5); // Problem child; need to tune to properly grab blue base
    delay(200);
    setLiftAngle(liftUp);
    delay(200);
    baseTurn(10,80,300,true, true,0.5); // Cannot test these until problem child is fixed
    delay(200);
    baseControl(30,100,100,2.5);
    delay(200);
    setLiftAngle(liftHalf-100);
    delay(200);
    baseControl(-10,100,100,2.5);
    delay(200);

    delay(2000);
}

