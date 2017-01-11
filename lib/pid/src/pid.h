#pragma once

#include "q16.h"
#include <Arduino.h>

typedef struct {
    q16 Kp;
    q16 Ki;
    q16 Ki_times_dt;
    q16 Kd;
    q16 Kd_over_dt;
    q16 dt;
    q16 out_min;
    q16 out_max;
} pidsettings;

typedef struct {
    q16 i_term;
    q16 last_error;
} pidstate;

void pid_initialize(q16 Kp, q16 Ki, q16 Kd, q16 dt, q16 out_min,
    q16 out_max, pidsettings *settings, pidstate *state);
q16 pid_compute(q16 input, q16 setpoint,
    pidsettings *settings, pidstate *state);
void pid_reset(q16 input, q16 setpoint, q16 output,
    pidsettings *settings, pidstate *state);
