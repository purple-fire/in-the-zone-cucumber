#include "utilities.h"

int limitMotorPower(int power) {
  int sign = power >= 0 ? 1 : -1;

  int outputPower;
  outputPower = ABS(power);

  if (outputPower > maxOutput) {
    outputPower = maxOutput;
  } else if (outputPower < minOutput) {
    outputPower = minOutput;
  }

  return (outputPower * sign);
}
