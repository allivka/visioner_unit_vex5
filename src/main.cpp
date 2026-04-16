#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <Vex5.h>

#define IN_HZ 60
#define OUT_HZ 10

MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float filteredYaw = 0;
float filterAlpha = 0.05;

unsigned long lastResetTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastGetTime = 0;

volatile bool dataReady = false;

void interruptHandler() {
    dataReady = true;
}

Vex5_Motor m1, m2, m3, m4;

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.begin(115200);
    while (!Serial);
    
    mpu.initialize();
    pinMode(3, INPUT);
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while(1);
    }
    
    if (mpu.dmpInitialize() != 0) {
        Serial.println("DMP init failed");
        while(1);
    }
    
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(3), interruptHandler, RISING);
    
    Vex5.begin();
    m1.begin(VEX5_PORT_1);
    m1.begin(VEX5_PORT_2);
    m1.begin(VEX5_PORT_3);
    m1.begin(VEX5_PORT_4);
    
    Serial.println("Ready");
    lastResetTime = millis();
}



struct {
    double viewAngle = 0;
    double motionAngle = 0;
    double speed = 0;
    double speedK = 1;
    bool enableHeadSync = false;
    
    void update() {
        uint8_t buff[17];
        
        Serial.readBytesUntil('\n', buff, 17);
        
        viewAngle = *(buff);
        motionAngle = *(buff + 4);
        speed = *(buff + 8);
        speedK = *(buff + 12);
        enableHeadSync = buff[16];
    }
} packageBuff;


void go(double s1, double s2, double s3, double s4) {
    m1.setSpeed(s1);
    m2.setSpeed(-s2);
    m3.setSpeed(-s3);
    m4.setSpeed(s4);
}

void goRelative(double angle, double speed, double w) {
    go(
        speed * cos(angle / 180 * PI) + w,
        speed * sin(angle / 180 * PI) + w,
        speed * cos(angle / 180 * PI) + w,
        speed * sin(angle / 180 * PI) + w
    );
}

// void run(double viewAngle, double motionAngle, double yaw, double speed) {
//     static float errold = 0;
//     float err = viewAngle - yaw;
    
//     if(err < -180.0) err += 360.0;
//     if(err > 180.0) err += -360.0;
    
//     float u = err * 1 + (err - errold) * 0.2;
//     errold = err;
    
//     goRelative(motionAngle - viewAngle, speed, u);
    
// }

void loop() {
    
    goRelative(0, 1000, 0);
    delay(1000);
    goRelative(0, 0, 0);
    delay(1000);
    
    // if (!dataReady) return;
    
    // dataReady = false;
    // uint16_t fifoCount = mpu.getFIFOCount();
    // uint8_t intStatus = mpu.getIntStatus();
    
    // if ((intStatus & 0x10) || fifoCount >= 1024) {
    //     mpu.resetFIFO();
    // } else if (intStatus & 0x02 && fifoCount >= mpu.dmpGetFIFOPacketSize()) {
    //     mpu.getFIFOBytes(fifoBuffer, mpu.dmpGetFIFOPacketSize());
    //     fifoCount -= mpu.dmpGetFIFOPacketSize();
        
    //     mpu.dmpGetQuaternion(&q, fifoBuffer);
    //     mpu.dmpGetGravity(&gravity, &q);
    //     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
    //     float rawYaw = ypr[0] * 180 / M_PI;
    //     filteredYaw = filterAlpha * rawYaw + (1 - filterAlpha) * filteredYaw;
    // }
    
    // if (millis() - lastPrintTime >= 1000 / OUT_HZ) {
    //     Serial.println(filteredYaw - 60);
    //     lastPrintTime = millis();
    // }
    
    // if (millis() - lastResetTime >= 60000) {
    //     mpu.resetFIFO();
    //     lastResetTime = millis();
    // }
    
    // if (millis() - lastGetTime >= 1000 / IN_HZ) {
    //     packageBuff.update();
    //     lastGetTime = millis();
    // }
    
    // run(packageBuff.viewAngle, packageBuff.motionAngle, filteredYaw - 60, packageBuff.speed * packageBuff.speedK);
}