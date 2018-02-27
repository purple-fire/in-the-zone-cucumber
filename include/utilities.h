/**
 * Assorted macros and utilities.
 */

#include <stdbool.h>

//Motor Power Bounds
#define MAX_POWER_OUT  127
#define MIN_POWER_OUT  10

//Motor power smoothing factor
#define SMOOTH_FACTOR 0.1

//Storing Constants for wheel diameter and Pi
#define PI 3.14159
#define WHEEL_DIAMETER 4.0
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

// Stick L/R, Axis X/Y
#define CRX 1
#define CRY	2
#define CLY 3
#define CLX 4

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define SIGN(x) ((x) >= 0 ? 1 : -1)

#define INCHES_TO_TICKS(inches) ((inches) * 360.0 / WHEEL_CIRCUMFERENCE)

/**
 * Limits the magnitude of a motor power between MAX_POWER_OUT and
 * MIN_POWER_OUT.
 * This sets a motor power of 0 to MIN_POWER_OUT (non-0) so that the motor will
 * brake instead of coast. To have a motor coast, motorStop() should be used.
 */
int limitMotorPower(int power);

/**
 * Set the power of both left motors.
 * Positive is forwards, negative is reverse.
 */
void leftMotorsSet(int power);

/**
 * Sets the power of both right motors.
 * Positive is forwards, negative is reverse.
 */
void rightMotorsSet(int power);

/**
 * Bring the chassis to a complete stop.
 */
void stopChassis(void);

void motorBrake(unsigned char channel);

void rightMotorsBrake(void);

void leftMotorsBrake(void);

/**
 * Sets the power of a motor towards a specified power based on the smoothing
 * factor SMOOTH_FACTOR.
 *
 * This is used to smooth motor transitions during non-PID motor control (i.e.
 * opcontrol).
 *
 * The value sent to the motor is
 *  cur_power * (1 - SMOOTH_FACTOR) + power * SMOOTH_FACTOR
 * where cur_power is the current power of the motor (the last value sent to
 * it).
 *
 * If this is called multiple times in succession with the same power value, the
 * motor's power will converge to it after about
 *  log_{1 - SMOOTH_FACTOR} (1 / |power - cur_power|)
 * iterations. With a smoothing factor of 0.1 and iterations spaced 20 ms
 * appart, this is about 1 s to go from the maximum value to 0.
 *
 * @param channel Motor channel to set (1 - 10).
 * @param power Power goal.
 * @return true (1) if the motor value is set to @p power; false (0) if not.
 */
bool motorSetSmooth(unsigned char channel, int power);

/**
 * Sets the power of both left motors using motorSetSmooth().
 *
 * @param power Power goal.
 * @return true (1) if both motor values are set to @p power; false (0) if not.
 */
bool leftMotorsSetSmooth(int power);

/**
 * Sets the power of both right motors using motorSetSmooth().
 *
 * @param power Power goal.
 * @return true (1) if both motor values are set to @p power; false (0) if not.
 */
bool rightMotorsSetSmooth(int power);

/**
 * Brings the chassis to a complete stop using motorSetSmooth().
 * This funciton blocks until all motor values are set to 0 by motorSetSmooth().
 */
void stopChassisSmooth(void);

void motorBrakeSmooth(unsigned char channel);

void rightMotorsBrakeSmooth(void);

void leftMotorsBrakeSmooth(void);


