/**
 * Functions for controlling the lift
 */

#include "main.h"

#define liftDown 1485
#define liftHalf 1280
#define liftUp 9

int liftToggle;
int desiredLiftAngle;

void liftControl(void *parameter);
void setLiftAngle(int liftAngle);

