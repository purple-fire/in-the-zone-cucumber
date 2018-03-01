/**
 * PID algorithm functions
 */

#include "pid.h"
#include "utilities.h"

void pidDataReset(PIDData *data)
{
    data->lastError = 0.0;
    data->integral = 0.0;
    data->errorCount = 0;
}

void pidDataInit(PIDData *data,
        float kp, float ki, float kd, float maxPower,
        float inputRange, float integralRange)
{
    data->kp = kp;
    data->ki = ki;
    data->kd = kd;
    data->maxPower = maxPower;
    data->integralRange= integralRange;

    data->maximumInput= inputRange;
    data->minimumInput= -inputRange;

    data->lastError = 0.0;
    data->integral = 0.0;
    data->errorCount = 0.0;

    pidDataReset(data);
}

float pidNextIteration(PIDData *data, float error)
{
    float proportional, integral, derivative;
    int result;

    /* Wrap around error */
    if (ABS(error) > (data->maximumInput - data->minimumInput) / 2) {
        if (error > 0) {
            error = error - data->maximumInput + data->minimumInput;
        } else {
            error = error + data->maximumInput - data->minimumInput;
        }
    }

    /* Limit integral
    if ((error * data->kp < data->maxPower) && (error * data->kp > -(data->maxPower))) {
        data->lastIntegral += error;
    } else {
        data->lastIntegral = 0;
    }
    */

    /* Limit integral (by another method)
    if( ABS(error) < data->integralRange ){
        data->lastIntegral += error;
    }
    else{
        data->lastIntegral = 0;
    }
    */

    /*Limit integral by 100 last samples
    if(data->errorCount<100 ){
        data->integral += error;
        data->errorCount+=1;
    }
    else{
        data->integral = 0;
        data->errorCount=0;

    }
    */

    proportional = error;
    integral = data->integral;
    derivative = error - data->lastError;

    result = ((data->kp * proportional) + (data->ki * integral) + (data->kd * derivative));
    data->lastError = error;

    return result;
}
