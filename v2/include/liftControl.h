/**
 * Functions for controlling the lift
 */

#include "main.h"

#define LIFT_DOWN 1030
#define LIFT_HALF 1530
#define LIFT_UP 2612

void liftControl(void *parameter);
void setLiftAngle(int liftAngle);
