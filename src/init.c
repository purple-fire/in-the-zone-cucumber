/**
 * File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "liftControl.h"

TaskHandle liftControlHandle;

/*
 * Runs pre-initialization code. This function will be started in kernel mode
 * one time while the VEX Cortex is starting up. As the scheduler is still
 * paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes
 * (pinMode()) and port states (digitalWrite()) of limit switches, push buttons,
 * and solenoids. It can also safely configure a UART port (usartOpen()) but
 * cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
}

/*
 * Runs user initialization code. This function will be started in its own task
 * with the default priority and stack size once when the robot is starting up.
 * It is possible that the VEXnet communication link may not be fully
 * established at this time, so reading from the VEX Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics),
 * LCDs, global variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and
 * autonomous() tasks will not start. An autonomous mode selection menu like the
 * pre_auton() in other environments can be implemented in this task if desired.
 */
void initialize() {
    /* Initialize the gryo */
    gyro = gyroInit(GYRO_PORT, 102);
    delay(2000); /* TODO Does gyroInit() do this */

    /* Initialize encoders
     * TODO Should one of these be reversed?
     */
    BLEncoder = encoderInit(QUAD_TOP_PORT_LEFT, QUAD_BOTTOM_PORT_LEFT, false);
    BREncoder = encoderInit(QUAD_TOP_PORT_RIGHT, QUAD_BOTTOM_PORT_RIGHT, false);

    /* Calibrate the potentiometer */
    analogCalibrate(POTENTIOMETER_PORT);

    /* Start the lift task here since it is needed in both autonomous and driver
     * control.
     */
    liftControlHandle = taskCreate(
            liftControl, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
}

