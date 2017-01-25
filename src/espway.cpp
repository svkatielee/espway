#include <Arduino.h>
#include <Hash.h>
#include <Wire.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESPAsyncWebServer.h>
#include <NeoPixelBus.h>

#include "motor.h"
#include "newpwm.h"
extern "C" {
#include "imu.h"
#include "pid.h"
}


enum logmode { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE, GYRO_CALIB };
enum state { STABILIZING_ORIENTATION, RUNNING, FALLEN, CUTOFF };

const float BETA = 0.1f;
const int MPU_RATE = 0;
const int MPU_ADDR = 0x68;
const logmode LOGMODE = LOG_NONE;
const int N_SAMPLES = 1000;
const int QUAT_DELAY = 50;

state myState = STABILIZING_ORIENTATION;

// Gyro calibration variables
const int N_GYRO_SAMPLES = 10000;
const int GYRO_OFFSETS[] = { 11, -7, 13 };
long int gyroOffsetAccum[] = { 0, 0, 0 };
int nGyroSamples = 0;

const q16 SMOOTHING_PARAM = Q16_ONE / 500;
const q16 TARGET_SMOOTHING_PARAM = Q16_ONE / 1000;

pidsettings anglePidSettings;
pidsettings motorPidSettings;
pidstate anglePidState;
pidstate motorPidState;

const q16 STABLE_ANGLE = (q16)(0.2 * Q16_ONE);
q16 targetAngle = STABLE_ANGLE;
q16 motorSpeed = 0;
q16 travelSpeed = 0;
q16 targetSpeed = 0;
q16 smoothedTargetSpeed = 0;
q16 steeringBias = 0;
unsigned long stageStarted = 0;
const unsigned long ORIENTATION_STABILIZE_DURATION = 12000;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
quaternion_fix quat = { Q16_MULTIPLIER, 0, 0, 0 };
q16 beta, gyroIntegrationFactor;

bool sendQuat = false;
AsyncWebSocketClient *wsclient = NULL;
unsigned long lastTime = 0;
int sampleCounter = 0;

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> eyes(2);
RgbColor RED(180, 0, 0);
RgbColor YELLOW(180, 180, 0);
RgbColor GREEN(0, 180, 0);
RgbColor BLUE(0, 0, 180);
RgbColor LILA(180, 0, 180);
RgbColor BLACK(0, 0, 0);

const unsigned long BATTERY_INTERVAL = 500;
const unsigned int BATTERY_THRESHOLD = 700;

bool motorsEnabled = false;
bool otaStarted = false;


void setBothEyes(RgbColor &color) {
    eyes.SetPixelColor(0, color);
    eyes.SetPixelColor(1, color);
    eyes.Show();
}


void initPID() {
    // Motor PID
    pid_initialize(
        Q16_ONE * 5,  // Kp
        Q16_ONE * 10,  // Ki
        Q16_ONE / 10,  // Kd
        Q16_ONE / 1000,  // dt
        -Q16_ONE,  // out_min
        Q16_ONE,  // out_max
        &motorPidSettings,
        &motorPidState);
    // Angle PID
    pid_initialize(
        Q16_ONE * 2,  // Kp
        Q16_ONE / 2,  // Ki
        Q16_ONE / 500,  // Kd
        Q16_ONE / 1000,  // dt
        -Q16_ONE * 3/4,  // out_min
        Q16_ONE * 3/4,  // out_max
        &anglePidSettings,
        &anglePidState);
}


void setMotors(q16 leftSpeed, q16 rightSpeed) {
    if (motorsEnabled) {
        setMotorSpeed(1, 12, -rightSpeed + steeringBias);
        setMotorSpeed(0, 15, -leftSpeed - steeringBias);
    } else {
        setMotorSpeed(1, 12, 0);
        setMotorSpeed(0, 15, 0);
    }
    pwmStart();
}


void wsCallback(AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void * arg, uint8_t *data, size_t len) {
    int8_t *signed_data = (int8_t *)data;
    wsclient = client;
    // Parse steering command
    if (len >= 2) {
        steeringBias = (Q16_ONE / 8 * signed_data[0]) / 128;
        targetSpeed = (Q16_ONE * 2/3 * signed_data[1]) / 128;
    }
}


void calculateIMUCoeffs() {
    float sampleTime = (1.0f + MPU_RATE) / 1000.0f;
    float gyroScale = 2.0f * M_PI / 180.0f * 2000.0f;
    gyroIntegrationFactor = float_to_q16(0.5f * gyroScale * sampleTime);
    beta = float_to_q16(BETA / (0.5f * gyroScale));
}


bool mpuInit() {
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setSleepEnabled(false);
    mpu.setRate(MPU_RATE);
    mpu.setTempSensorEnabled(false);
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setIntDataReadyEnabled(true);
    mpu.setIntEnabled(true);
    mpu.setXGyroOffset(GYRO_OFFSETS[0]);
    mpu.setYGyroOffset(GYRO_OFFSETS[1]);
    mpu.setZGyroOffset(GYRO_OFFSETS[2]);
    return mpu.getDeviceID() == 0x34;
}


// Adapted from I2Cdevlib: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp,
// adding a finite timeout
int getMotion6(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[14];
    int count = I2Cdev::readBytes(MPU_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14,
        buffer, 2);
    accel[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    accel[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    accel[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyro[0] = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyro[1] = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyro[2] = (((int16_t)buffer[12]) << 8) | buffer[13];
    return count;
}


// Adapted from I2Cdevlib: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp,
// adding a finite timeout
bool getIntDataReadyStatus() {
    uint8_t buffer;
    I2Cdev::readBit(MPU_ADDR, MPU6050_RA_INT_STATUS,
        MPU6050_INTERRUPT_DATA_RDY_BIT, &buffer, 2);
    return buffer;
}


void setup() {
    pinMode(A0, INPUT);

    if (LOGMODE != LOG_NONE) {
        Serial.begin(115200);
    }

    initPID();
    pinMode(12, OUTPUT);
    pinMode(15, OUTPUT);
    pwmAddChannel(13);
    pwmAddChannel(14);
    pwmInit();

    eyes.Begin();
    setBothEyes(BLACK);

    // I2C initialization
    Wire.begin(4, 5);
    Wire.setClock(400000);
    calculateIMUCoeffs();
    if (mpuInit()) {
        // NeoPixel eyes initialization
        setBothEyes(YELLOW);
    } else {
        setBothEyes(RED);
        return;
    }

    // WiFi soft AP init
    WiFi.persistent(false);
    WiFi.softAP("ESPway", NULL);
    // WiFi.softAP("ESPway", NULL, 1, 0, 1);  // Use this as soon as new Arduino framework is released

    ArduinoOTA.onStart([]() {
        otaStarted = true;
        motorsEnabled = false;
        setMotors(0, 0);
        setBothEyes(LILA);
    });
    // ArduinoOTA init
    ArduinoOTA.begin();

    SPIFFS.begin();

    ws.onEvent(wsCallback);
    server.addHandler(&ws);

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.onNotFound([](AsyncWebServerRequest *req) {
        req->send(404);
    });
    server.begin();

    myState = STABILIZING_ORIENTATION;
    stageStarted = millis();
}


void doLog(q16 spitch) {
    if (LOGMODE == LOG_RAW) {
        Serial.print(ax); Serial.print(',');
        Serial.print(ay); Serial.print(',');
        Serial.print(az); Serial.print(',');
        Serial.print(gx); Serial.print(',');
        Serial.print(gy); Serial.print(',');
        Serial.println(gz);
    } else if (LOGMODE == LOG_FREQ) {
        unsigned long ms = millis();
        if (++sampleCounter == N_SAMPLES) {
            Serial.println(N_SAMPLES * 1000 / (ms - lastTime));
            sampleCounter = 0;
            lastTime = ms;
        }
    } else if (LOGMODE == GYRO_CALIB && nGyroSamples != N_GYRO_SAMPLES) {
        gyroOffsetAccum[0] += gx;
        gyroOffsetAccum[1] += gy;
        gyroOffsetAccum[2] += gz;
        nGyroSamples += 1;

        if (nGyroSamples == N_GYRO_SAMPLES) {
            Serial.println("Gyro offsets:");
            Serial.print("X: "); Serial.println(gyroOffsetAccum[0] / N_GYRO_SAMPLES);
            Serial.print("Y: "); Serial.println(gyroOffsetAccum[1] / N_GYRO_SAMPLES);
            Serial.print("Z: "); Serial.println(gyroOffsetAccum[2] / N_GYRO_SAMPLES);
        }
    } else if (LOGMODE == LOG_PITCH) {
        Serial.println(spitch);
    }
}


void sendQuaternion() {
    int16_t qdata[4];
    qdata[0] = quat.q0 / 2;
    qdata[1] = quat.q1 / 2;
    qdata[2] = quat.q2 / 2;
    qdata[3] = quat.q3 / 2;
    wsclient->binary((uint8_t *)qdata, 8);
    sendQuat = false;
}


void loop() {
    if (otaStarted) {
        ArduinoOTA.handle();
        return;
    }

    while (!getIntDataReadyStatus());

    // Perform MPU quaternion update
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    getMotion6(rawAccel, rawGyro);
    // Update orientation estimate
    MadgwickAHRSupdateIMU_fix(beta, gyroIntegrationFactor, rawAccel, rawGyro,
        &quat);
    // Calculate sine of pitch angle from quaternion
    q16 spitch = sinPitch_gravX(&quat);

    // Exponential smoothing of target speed
    // https://en.wikipedia.org/wiki/Exponential_smoothing
    smoothedTargetSpeed = q16_mul(Q16_ONE - TARGET_SMOOTHING_PARAM,
        smoothedTargetSpeed) + q16_mul(TARGET_SMOOTHING_PARAM, targetSpeed);

    // Estimate travel speed by exponential smoothing
    travelSpeed = q16_mul(Q16_ONE - SMOOTHING_PARAM, travelSpeed) +
        q16_mul(SMOOTHING_PARAM, motorSpeed);

    unsigned long curTime = millis();
    if (myState == STABILIZING_ORIENTATION) {
        if (curTime - stageStarted > ORIENTATION_STABILIZE_DURATION) {
            myState = RUNNING;
            stageStarted = curTime;
        }
    } else if (myState == RUNNING) {
        if (spitch < Q16_ONE*3/4 && spitch > -Q16_ONE*3/4) {
            // Perform PID update
            targetAngle = pid_compute(travelSpeed, smoothedTargetSpeed,
                &anglePidSettings, &anglePidState);
            motorSpeed = -pid_compute(spitch, targetAngle,
                &motorPidSettings, &motorPidState);
        } else {
            myState = FALLEN;
            setBothEyes(BLUE);
            motorSpeed = 0;
            motorsEnabled = false;
        }
    } else if (myState == FALLEN) {
        if (spitch < Q16_ONE/2 && spitch > -Q16_ONE/2) {
            myState = RUNNING;
            setBothEyes(GREEN);
            pid_reset(spitch, STABLE_ANGLE, 0, &motorPidSettings,
                &motorPidState);
            pid_reset(0, 0, STABLE_ANGLE, &anglePidSettings, &anglePidState);
            motorsEnabled = true;
        }
    }

    setMotors(motorSpeed, motorSpeed);

    if (LOGMODE != LOG_NONE) {
        doLog(spitch);
    }

    static unsigned long lastSentQuat = 0;
    if (sendQuat && curTime - lastSentQuat > QUAT_DELAY) {
        sendQuaternion();
        lastSentQuat = curTime;
    }

    static unsigned long lastBatteryCheck = 0;
    if (curTime - lastBatteryCheck > BATTERY_INTERVAL) {
        lastBatteryCheck = curTime;
        if (analogRead(A0) < BATTERY_THRESHOLD) {
            myState = CUTOFF;
            setBothEyes(BLACK);
            mpu.setSleepEnabled(true);
            motorsEnabled = false;
            ESP.deepSleep(100000000UL);
        }
    }

    static unsigned long lastOtaHandled = 0;
    if (curTime - lastOtaHandled > 100) {
        ArduinoOTA.handle();
        lastOtaHandled = curTime;
    }
}

