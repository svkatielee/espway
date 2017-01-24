#pragma once

extern "C" {
#include "q16.h"
}

void setMotorSpeed(int channel, int dirPin, q16 speed);
