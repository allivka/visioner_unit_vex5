#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

float filteredYaw = 0;
float filterAlpha = 0.05;
unsigned long lastResetTime = 0;
unsigned long lastPrintTime = 0;
volatile bool dataReady = false;

void interruptHandler() {
    dataReady = true;
}

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
    
    Serial.println("Ready");
    lastResetTime = millis();
}

void loop() {
    if (!dataReady) return;
    
    dataReady = false;
    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t intStatus = mpu.getIntStatus();
    
    if ((intStatus & 0x10) || fifoCount >= 1024) {
        mpu.resetFIFO();
    } else if (intStatus & 0x02 && fifoCount >= mpu.dmpGetFIFOPacketSize()) {
        mpu.getFIFOBytes(fifoBuffer, mpu.dmpGetFIFOPacketSize());
        fifoCount -= mpu.dmpGetFIFOPacketSize();
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        float rawYaw = ypr[0] * 180 / M_PI;
        filteredYaw = filterAlpha * rawYaw + (1 - filterAlpha) * filteredYaw;
    }
    
    if (millis() - lastPrintTime >= 100) {
        Serial.println(filteredYaw - 60);
        lastPrintTime = millis();
    }
    
    if (millis() - lastResetTime >= 60000) {
        mpu.resetFIFO();
        lastResetTime = millis();
    }
}