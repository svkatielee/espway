#pragma once


// Angle PID == the PID controller which regulates motor output signal to reach
// the target angle
const float ANGLE_KP = 5.0f, ANGLE_KI = 10.0f, ANGLE_KD = 0.1f;
const float ANGLE_HIGH_KP = 10.0f, ANGLE_HIGH_KI = ANGLE_KI, ANGLE_HIGH_KD = ANGLE_KD;
// Velocity PID == the PID controller which regulates target angle to reach
// the target velocity
const float VEL_KP = 2.0f, VEL_KI = 0.5f, VEL_KD = 0.002f;

const float STEERING_FACTOR = 0.13f,
            SPEED_CONTROL_FACTOR = 0.67f;

const float TRAVEL_SPEED_SMOOTHING = 0.002f,
            TARGET_SPEED_SMOOTHING = 0.001f;

// Madgwick beta == accelerometer correction factor. Should be fine-ish as is
const float MADGWICK_BETA = 0.1f;

const unsigned long ORIENTATION_STABILIZE_DURATION = 6000;

const float FALL_LIMIT = 0.75f,
            RECOVER_LIMIT = 0.4f;

const float STABLE_ANGLE = 0.2f;

const int16_t GYRO_OFFSETS[] = { -27, -89, 14 };

// Undervoltage cutoff check
const float BATTERY_THRESHOLD = 7.4f;
const unsigned long BATTERY_CHECK_INTERVAL = 500;
const bool ENABLE_BATTERY_CHECK = true;

