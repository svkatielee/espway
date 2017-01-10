#include <Arduino.h>

#include "motor.h"


void setMotorSpeed(int pwmPin, int dirPin, q16 speed) {
    speed = constrain((PWMRANGE * speed) >> 16, -PWMRANGE, PWMRANGE);

    if (speed < 0) {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, PWMRANGE + speed);
    } else {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, speed);
    }
}
