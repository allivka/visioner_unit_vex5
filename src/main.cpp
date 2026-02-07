#include "visioner_unit_vex5.hpp"
#include <stdio.h>
#include <Wire.h>

using namespace vislib;

motor::SpeedRange rpmSpeedRange(-600, 600);
motor::SpeedRange motorUseSpeedRange(-1500, 1500);
motor::SpeedRange motorInterfaceAngularSpeedRange(
    motorUseSpeedRange.mapValueToRange(-1500, rpmSpeedRange) * 2 * PI, motorUseSpeedRange.mapValueToRange(1500, rpmSpeedRange) * 2 * PI);

constexpr double wheelR = 0.1;
constexpr double motorDistance = 0.3;

platform::PlatformMotorConfig config({
    motor::MotorInfo(90, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
    motor::MotorInfo(180, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
    motor::MotorInfo(-90, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
    motor::MotorInfo(0, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange)
});

platform::Platform<binds::vex5::motor::V5MotorController> plat(config);


constexpr binds::arduino::port_t mpuInterruptPort = 2;
binds::mpu6050::GyroscopeDMP<0> mpu;
core::IncrementTimer<int64_t> timer([]() -> core::Result<int64_t> {
    return static_cast<int64_t>(millis());
});

VisionerUnitVex5::Visioner visioner;

void setup() {
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    str = static_cast<char*>(malloc(500));
    
    Serial.begin(115200);
    
    visioner.setup();
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    
    
}

void go(double angle, double speed, double angularSpeed = 0) {
    auto speeds = platform::calculators::calculatePlatformSpeeds(config, angle, speed);
    if(speeds) {
        Serial.println(("Ooops, something went wrong in calculating speeds for linear movement of the platform: " + speeds.error().msg).c_str());
        return;
    }
    
    auto err = plat.setSpeeds(speeds());
    
    if(err) {
        Serial.println(("Ooops, something went wrong in applying speeds to motors for linear movement of the platform: " + err.msg).c_str());
    }
}

void move(double angle, double speed, ull_t delayMs) {
    go(angle, speed);
    delay(delayMs);
}

double speed = 1000;
ull_t sectionTime = 1000;

void loop() {      
    
    


    ++timer;
    mpu.update(nullptr);

}