/**
 * Assorted utilities
 */

#include "utilities.h"

int limitMotorPower(int power)
{
    int	outputPower = ABS(power);

    if(outputPower > MAX_POWER_OUT)
    {
        outputPower = MAX_POWER_OUT;
    }
    else if(outputPower < MIN_POWER_OUT)
    {
        outputPower = MIN_POWER_OUT;
    }

    return SIGN(power) * outputPower;
}

