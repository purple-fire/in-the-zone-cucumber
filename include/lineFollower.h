/**
 * Line following functions/macros.
 */

#ifndef LINEFOLLOWER_H_
#define LINEFOLLOWER_H_

#define IS_LINE(port) (analogReadCalibrated(port) < -200)

/**
 * Drives foreward with constant power until the center line sensor reaches a
 * line.
 */
void driveToLine(int power, float timeOut);

/**
 * Follows a line with constant power for an amount of time.
 * It is assumed that the robot is currently on a line (from a previous call to
 * driveToLine()).
 */
void followLineTime(int power, float time);

/**
 * Follows a line for a certain distance using PID control.
 * It is assumed that the robot is currently on a line (from a previous call to
 * driveToLine()).
 */
void followLine(int inches, float timeOut);

#endif /* LINEFOLLOWER_H_ */

/* vim: set tw=80 ft=c: */
