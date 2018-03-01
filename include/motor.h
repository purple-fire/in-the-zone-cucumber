/**
 * Wrapper for motors.
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>

/* Motors */
#define rightMotorR 2
#define rightMotorF 3
#define liftMotor 4
#define leftMotorR 7
#define leftMotorF 8

#define MOTOR_SMOOTH_FACTOR 0.1

//Motor Power Bounds
#define MOTOR_POWER_MAX  127
#define MOTOR_POWER_MIN  10

/**
 * Limits the magnitude of a motor power between MOTOR_POWER_MAX and
 * MOTOR_POWER_MIN.
 * This sets a motor power of 0 to MOTOR_POWER_MIN (non-0) so that the motor will
 * brake instead of coast. To have a motor coast, motorStop() should be used.
 */
int motorPowerLimit(int power);

/**
 * Sets the power of a motor towards a new power using a smoothing function.
 *
 * The actual power sent to the motor is
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
 * @param channel Motor to set.
 * @param power Power goal to set. This should be from -128 to +127. Sending a
 * power of 0 turns off the motor, letting it coast.
 * @return true if the motor's power was set to power; false otherwise.
 */
bool motorSetSmooth(unsigned char channel, int power);

/**
 * Sends braking power to a motor.
 *
 * Power sent is a low value in the opposite direction of the current power.
 *
 * @param channel Motor to brake.
 */
void motorBrake(unsigned char channel);

/**
 * Brakes a motor using motorSetSmooth().
 *
 * This functions blocks until the motor's power is set to 0 (coasting).
 * @see motorBrake()
 * @see motorSetSmooth()
 *
 * @param channel Motor to brake.
 */
void motorStopSmooth(unsigned char channel);

/**
 * Set the power of both left motors.
 *
 * @param power Power to set.
 */
void leftMotorsSet(int power);

/**
 * Sets the power of both right motors.
 *
 * @param power Power to set.
 */
void rightMotorsSet(int power);

/**
 * Turns off all chassis motors, letting them coast.
 */
void chassisStop(void);

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
 * Brakes both left motors using motorBrake().
 */
void leftMotorsBrake(void);

/**
 * Brakes both right motors using motorBrake().
 */
void rightMotorsBrake(void);

/**
 * Brakes all motors using motorBrake().
 */
void chassisBrake(void);

/**
 * Brakes both left motors using motorStopSmooth().
 */
void leftMotorsStopSmooth(void);

/**
 * Brakes both right motors using motorStopSmooth().
 */
void rightMotorsStopSmooth(void);

/**
 * Brakes all motors using motorStopSmooth().
 */
void chassisStopSmooth(void);

#endif /* MOTOR_H_ */

/* vim: set tw=80 ft=c: */
