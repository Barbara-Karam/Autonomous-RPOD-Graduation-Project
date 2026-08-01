#include "pid.h"
#include <stdbool.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_limit, float int_limit)
{
    pid->kp               = kp;
    pid->ki               = ki;
    pid->kd               = kd;
    pid->output_limit     = out_limit;
    pid->integrator_limit = int_limit;
    PID_Reset(pid);
}

float PID_Update(PID_t *pid, float setpoint,
                 float measurement, float dt)
{
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->kp * error;

    // Derivative on measurement -- avoids kick on setpoint change
    float derivative = -pid->kd *
                       (measurement - pid->prev_measurement) / dt;
    pid->prev_measurement = measurement;

    /* ── Conditional-integration anti-windup ──────────────────────────
       Previously the integrator was hard-clamped to +/-integrator_limit
       (800, vs an output_limit of 4199). That clamp was the actual ceiling
       on sustained output -- P alone maxes out around 300 at full RPM
       error, so P+I together topped out near 1100/4199 (~26% duty). Any
       commanded speed that needed more authority than that just saturated
       there, which is why raising/lowering the target barely changed
       actual motor speed: you were pinned against the integrator clamp,
       not tracking the setpoint.

       Conditional integration removes that artificial ceiling while still
       preventing classic windup: on each tick, compute what the output
       would be with the integrator as it currently stands (no new
       contribution added yet). If that pre-update output is already
       saturated AND the pending integral contribution would push it
       further into saturation, skip accumulating this tick. If the error
       has reversed (i.e. integrating would pull the output back out of
       saturation), let it accumulate normally. This gives the full
       output_limit range as real control authority instead of reserving
       most of it as unused headroom, while still stopping the integrator
       from winding up past what the actuator can ever use.

       The zero-target stall cutout in control_loop.c and the immediate
       PID_Reset() on a zero-target command in ControlLoop_SetTarget()
       remain in place and are what actually prevent the original
       "motor keeps spinning after a stop command" bug -- that fix did not
       depend on the integrator being pre-clamped, so lifting the clamp
       here does not reopen it.                                            */
    float integrator_delta = pid->ki * error * dt;

    float output_pre_integration = proportional + pid->integrator + derivative;

    bool saturated_high = output_pre_integration >  pid->output_limit;
    bool saturated_low  = output_pre_integration < -pid->output_limit;

    bool would_worsen_high = saturated_high && (integrator_delta > 0.0f);
    bool would_worsen_low  = saturated_low  && (integrator_delta < 0.0f);

    if (!would_worsen_high && !would_worsen_low) {
        pid->integrator += integrator_delta;
    }

    /* Hard safety backstop only -- should not normally be reached given the
       conditional-integration logic above, but guards against edge cases
       (e.g. dt anomalies, gain retuning) leaving the integrator to grow
       unbounded. integrator_limit is set well above what a legitimate
       control action would need (see control_loop.h), so this does not
       function as the everyday ceiling anymore. */
    if (pid->integrator >  pid->integrator_limit)
        pid->integrator =  pid->integrator_limit;
    if (pid->integrator < -pid->integrator_limit)
        pid->integrator = -pid->integrator_limit;

    float output = proportional + pid->integrator + derivative;

    // Output clamp
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    pid->output = output;
    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integrator       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output           = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_Reset(pid);
}
