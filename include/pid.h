#include <main.h>
#define PIDABS(x) ( (x)>=0?(x):-(x) )

typedef struct PIDData
{
	float kp, ki, kd;

	float lastError;
	float lastIntegral;

	float maxPower;
	float integralRange;
	float minimumInput,maximumInput;
} PIDData;

void pidDataReset(PIDData *data);
void pidDataInit(PIDData *data, float kp, float ki, float kd, float maxPower,float inputRange,float integralRange);
float pidNextIteration(PIDData *data, float error);
