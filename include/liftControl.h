#define ABS(x)          ( (x)>=0?(x):-(x) )

#define liftDown 1420
#define liftHalf 1210
#define liftUp 4

#define liftMotor 4
#define POTENTIOMETER_PORT 2

int liftToggle;
int desiredLiftAngle;

void liftControl(void * parameter);
void setLiftAngle(int liftAngle);
