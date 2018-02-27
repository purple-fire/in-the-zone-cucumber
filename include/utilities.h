/**
 * Assorted macros and utilities.
 */

//Motor Power Bounds
#define MAX_POWER_OUT  127
#define MIN_POWER_OUT  10

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

#define INCHES_TO_TICKS(inches) ((inches) * 360 / WHEEL_CIRCUMFERENCE)

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

