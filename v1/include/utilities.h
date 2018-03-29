/**
 * Assorted macros and utilities.
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

// Stick L/R, Axis X/Y
#define CRX 1
#define CRY	2
#define CLY 3
#define CLX 4

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define SIGN(x) ((x) == 0 ? 0 : ((x) > 0 ? 1 : -1))

//Storing Constants for wheel diameter and Pi
#define PI 3.14159
#define WHEEL_DIAMETER 4.0
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

#define INCHES_TO_TICKS(inches) ((inches) * 360.0 / WHEEL_CIRCUMFERENCE)

#endif

