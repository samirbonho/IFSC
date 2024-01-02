#include "pid.h"

void pid_init(PIDController_t *target) {
    
    /* clear controller variables */
    target->integrator = 0.0F;
    target->prevError = 0.0F;

    target->differentiator = 0.0F;
    target->prevMeasurement = 0.0F;

    target->out = 0.0F;

}

float pid_update(PIDController_t *target, float setPoint, float measurement) {

    /* error signal */
    float error = setPoint - measurement;



    /* proportional */
    float proportional = target->Kp * error;



    /* integral */
    target->integrator = target->integrator + 0.5F * target->Ki * target->T * (error + target->prevError);

    /* anti-wind-up via dynamic integrator clamping */
    float limMinInt, limMaxInt;

    /* compute integrator limits */
    limMaxInt = (target->limMax > proportional) ? target->limMax - proportional : 0.0F;
    limMinInt = (target->limMin < proportional) ? target->limMin - proportional : 0.0F;

    /* clamp integrator */
    if (target->integrator > limMaxInt) {

        target->integrator = limMaxInt;

    } else if (target ->integrator < limMinInt) {
        
        target->integrator = limMinInt;

    }



    /* derivative (band-limited differentiator) */
    target->differentiator = (2.0F * target->Kd * (measurement - target->prevMeasurement)
                            + (2.0F * target->tau - target->T) * target->differentiator)
                            / (2.0F * target->tau + target->T);

    /* add all terms and apply limits */  
    target->out = proportional + target->integrator + target->differentiator;
    if (target->out > target->limMax) {

        target->out = target->limMax;

    } else if (target->out < target->limMin) {

        target->out = target->limMin;

    }

    #ifdef DEBUG
    UARTprintf("PID...error= %d\n",(int)error);
    UARTprintf("PID...proportional= %d\n",(int)proportional);
    UARTprintf("PID...integrator= %d\n",(int)target->integrator);
    UARTprintf("PID...differentiator= %d\n",(int)target->differentiator);
    UARTprintf("PID...total out= %d\n",(int)target->out);
    #endif

    /* update prev variables */
    target->prevError = error;
    target->prevMeasurement = measurement;

    /* return output of controller */
    return target->out;

}
