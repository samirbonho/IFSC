#ifndef PID_H
#define PID_H


/* Defines */
#define SENSOR_SAMPLE_PERIOD_TICKS  3125 // clock divider to 16 for 50ms sampling period
#define PWM_PERIOD_TICKS            500  // clock divider to 1 for 1ms period (updown)

#define SAMPLER_TIMER_BASE          TIMER_B0_BASE




/* Constructs */
typedef struct {

    /* controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* derivative low pass */
    float tau;

    /* output limits */
    float limMin;
    float limMax;

    /* sample time (in seconds) */
    float T;

    /* controller memories */
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    /* controller output */
    float out;
    
} PIDController_t;
/* ---------- */

/* External Functions */
void pid_init(PIDController_t *target);
float pid_update(PIDController_t *target, float setPoint, float measurement);
/* ------------------ */

#endif

