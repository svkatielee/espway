#include <Arduino.h>
#include <Hash.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#include <ESPAsyncWebServer.h>

#include <MadgwickAHRS_fix.h>


MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

const bool logFrequency = true;
const int nSamples = 100;
unsigned long lastTime = 0;
int sampleCounter = 0;


volatile bool intFlag = false;
void mpuInterrupt() {
    intFlag = true;
}

void setup() {
    Serial.begin(115200);

    Wire.begin(0, 5);
    Wire.setClock(400000);

    mpu.initialize();
    mpu.setRate(1);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setIntEnabled(true);
    mpu.setInterruptMode(MPU6050_INTMODE_ACTIVELOW);
    mpu.setInterruptDrive(MPU6050_INTDRV_PUSHPULL);

    attachInterrupt(4, mpuInterrupt, RISING);
}


void loop() {
    while (!intFlag);
    intFlag = false;
    mpu.getIntStatus();

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (!logFrequency) {
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
    } else {
        unsigned long ms = millis();
        if (++sampleCounter == nSamples) {
            Serial.println(nSamples * 1000 / (ms - lastTime));
            sampleCounter = 0;
            lastTime = ms;
        }
    }
}
