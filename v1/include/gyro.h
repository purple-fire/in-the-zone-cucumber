/**
 * Wrapper for the PROS Gyro object to support offsets.
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "API.h"

/**
 * Wrapper for the Gyro object.
 * Since Gyro is typedef'd to void *, the compiler will not complain if you try
 * to pass this where a PROS Gyro object is expected, so be careful.
 */
typedef struct DevGyro {
    Gyro base;
    int offset;
} DevGyro;

/**
 * Initializes gyro data.
 *
 * @param pGyro Gyro data to initialize.
 * @param port (Analog) Port of the gyro.
 * @param multiplier Multiplier of the gyro to tune the value returned. Use 0
 * for the default value.
 */
void devgyroInit(DevGyro *pGyro, unsigned char port, unsigned short multiplier);

/**
 * Destroys gyro data and de-initializes the gyro.
 *
 * @param pGyro Gyro data to destroy.
 */
void devgyroDestroy(DevGyro *pGyro);

/**
 * Gets a gyro value.
 *
 * @param pGyro Gyro to get from.
 * @return Angle of the gyro, in degrees.
 */
int devgyroGet(DevGyro *pGyro);

/**
 * Resests a gyro to an angle of 0.
 *
 * @param pGyro Gyro to reset.
 */
void devgyroReset(DevGyro *pGyro);

/**
 * Resets a gyro to a specified angle.
 *
 * @param pGyro Gyro to reset.
 * @param angle Angle to reset to.
 */
void devgyroResetTo(DevGyro *pGyro, int angle);

/**
 * Adds an offset to the gyro's value.
 *
 * @param pGyro Gyro to offset.
 * @param offset Offset to add in degrees (can be negative).
 */
void devgyroOffset(DevGyro *pGyro, int offset);

#endif /* GYRO_H_ */

/* vim: set tw=80 ft=c: */
