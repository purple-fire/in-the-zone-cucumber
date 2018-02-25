#define ABS(x)          ( (x)>=0?(x):-(x) )
#define MAX_POWER_OUT										127
#define MIN_POWER_OUT								  	10

#define liftDown 1420
#define liftHalf 1210
#define liftUp 4

#define liftMotor 4
#define POTENTIOMETER_PORT 2

int liftToggle;
int desiredLiftAngle;

int limitLiftPower(int power);
void liftControl(void * parameter);
void setLiftAngle(int liftAngle);
