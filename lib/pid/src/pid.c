#include "pid.h"

static inline q16 clamp(q16 x, q16 a, q16 b) {
    return x > a ? (x < b ? x : b) : a;
}

void pid_initialize(q16 Kp, q16 Ki, q16 Kd, q16 dt,
    q16 out_min, q16 out_max, pidsettings *settings, pidstate *state) {
    settings->Kp = Kp;
    settings->Ki = Ki;
    settings->Ki_times_dt = q16_mul(Ki, dt);
    settings->Kd = Kd;
    settings->Kd_over_dt = q16_div(Kd, dt);
    settings->dt = dt;
    settings->out_min = out_min;
    settings->out_max = out_max;
    state->i_term = 0;
    state->last_input = 0;
}

q16 pid_compute(q16 input, q16 setpoint,
    pidsettings *settings, pidstate *state) {
    q16 error = input - setpoint;
    q16 p_term = q16_mul(settings->Kp, error);
    state->i_term = clamp(state->i_term + q16_mul(settings->Ki_times_dt, error),
        settings->out_min, settings->out_max);
    q16 d_term = q16_mul(settings->Kd_over_dt, input - state->last_input);
    q16 output = p_term + state->i_term + d_term;
    state->last_input = input;
    return clamp(output, settings->out_min, settings->out_max);
}

void pid_reset(q16 input, q16 setpoint, q16 output,
    pidsettings *settings, pidstate *state) {
    state->i_term = clamp(output, settings->out_min,
        settings->out_max);
    state->last_input = input;
}
