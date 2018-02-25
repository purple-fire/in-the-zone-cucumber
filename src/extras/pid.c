#include "pid.h"
#include "utilities.h"

void pidDataReset(PIDData *data)
{
	data->lastError = 0.0;
	data->lastIntegral = 0.0;
}

void pidDataInit(PIDData *data, float kp, float ki, float kd, float maxPower,float inputRange,float integralRange)
{
	data->kp = kp;
	data->ki = ki;
	data->kd = kd;
	data->maxPower = maxPower;
	data->integralRange= integralRange;

	data->maximumInput= inputRange;
	data->minimumInput= -inputRange;

	data->lastError = 0.0;
	data->lastIntegral = 0.0;

	pidDataReset(data);
}

float pidNextIteration(PIDData *data, float error)
{
	float proportional, integral, derivative;
	int result;


	if (ABS(error) > (data->maximumInput - data->minimumInput) / 2) {
		if (error > 0) {
			error = error - data->maximumInput + data->minimumInput;
			} else {
			error = error + data->maximumInput - data->minimumInput;
		}
	}



/*
	if ((error * data->kp < data->maxPower) && (error * data->kp > -(data->maxPower))) {
		data->lastIntegral += error;
		} else {
		data->lastIntegral = 0;
	}

	if( ABS(error) < data->integralRange ){
	data->lastIntegral += error;
	}
	else{
	data->lastIntegral = 0;
	}
*/

	proportional = error;
	integral = data->lastIntegral;
	derivative = error - data->lastError;

	result = ((data->kp * proportional) + (data->ki * integral) + (data->kd * derivative));
	data->lastError = error;

	return result;
}

/*
float pidNextIteration(PIDData *data, float error)
{
float proportional, integral, derivative;
int result;
proportional = error;

// Bound integral
if( ABS(error) < data->integralRange ){
data->lastIntegral += error;
}
else{
data->lastIntegral = 0;
}
integral = data->lastIntegral;

derivative = error - data->lastError;
data->lastError = error;

result = (data->kp * proportional) + (data->ki * integral) + (data->kd * derivative);

if (result > data->maxPower) {
result = data->maxPower;
} else if (result < -(data->maxPower)) {
result = -(data->maxPower);
}

return result;
}
*/
