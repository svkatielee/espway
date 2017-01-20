#include <Arduino.h>

#include "motor.h"

static int32_t PERIOD = 2500;

void motorInit(uint32_t period) {
    PERIOD = period;

    // PWM setup
    uint32 io_info[][3] = {
        // MUX, FUNC, PIN
        {PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13},
        {PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}
    };

    // initial duty: all off
    uint32 pwm_duty_init[] = {0, 0};

    pwm_init(period, pwm_duty_init, 2, io_info);
    pwm_start();
}

void setMotorSpeed(int channel, int dirPin, q16 speed) {
    speed = constrain((PERIOD * speed) >> 16, -PERIOD, PERIOD);

    if (speed < 0) {
        digitalWrite(dirPin, HIGH);
        pwm_set_duty(PERIOD + speed, channel);
    } else {
        digitalWrite(dirPin, LOW);
        pwm_set_duty(speed, channel);
    }
}

void motorCommit() {
    pwm_start();
}

