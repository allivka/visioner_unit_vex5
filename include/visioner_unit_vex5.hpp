#pragma once

#include <vislib_vex5.hpp>
#include <vislib_mpu6050.hpp>

namespace VisionerUnitVex5 {

const vislib::motor::SpeedRange rpmSpeedRange(-600, 600);
const vislib::motor::SpeedRange motorUseSpeedRange(-1500, 1500);
const vislib::motor::SpeedRange motorInterfaceAngularSpeedRange(motorUseSpeedRange.mapValueToRange(-1500, rpmSpeedRange) * 2 * PI, motorUseSpeedRange.mapValueToRange(1500, rpmSpeedRange) * 2 * PI);

constexpr double wheelR = 0.1;
constexpr double motorDistance = 0.3;

constexpr vislib::binds::arduino::port_t mpuInterruptPort = 2;

constexpr double PID_K_P = 1;
constexpr double PID_K_I = 0.1;
constexpr double PID_K_D = 0.3;

class Visioner {
private:

    static vislib::platform::PlatformMotorConfig platformConfig;
    
    static vislib::binds::mpu6050::GyroscopeDMP<0> mpu;
    
    static vislib::core::IncrementTimer<int64_t> timer;
    
    static vislib::platform::GyroPlatform<vislib::binds::vex5::motor::V5MotorController, int64_t> platform;
    
    struct YawGetter : public vislib::gyro::YawGetter<vislib::core::Angle<>>{
    private:
        vislib::binds::mpu6050::GyroscopeDMP<0> *mpuPtr = &mpu;
        
    public:
        
        vislib::core::Result<vislib::core::Angle<>> getYaw() const noexcept override {
            return mpuPtr->getYaw();
        }
        
    };
    
    vislib::core::UniquePtr<vislib::gyro::YawGetter<vislib::core::Angle<>>> yawGetterPtr{nullptr};
    
public:

    void setup(vislib::core::Callable<void, const char*> printer = [](const char *msg) { delay(50); Serial.println(msg); }) {
        
        yawGetterPtr.reset(new YawGetter());

        vislib::core::TimeGetter<int64_t> timeGetter = []() -> vislib::core::Result<int64_t> { return timer.getTime(); };

        platform = vislib::platform::GyroPlatform<vislib::binds::vex5::motor::V5MotorController, int64_t>(
            vislib::platform::calculators::GyroPidCalculator<int64_t>(
                vislib::PIDRegulator<double, int64_t>(
                    PID_K_P,
                    PID_K_I,
                    PID_K_D
                ),
                platformConfig
            ),
            yawGetterPtr,
            timeGetter,
            platformConfig
        );
        
        auto e = timer.start();
        
        if (e) while (true) printer((String("Failed to initialize timer: ") + String(e.error().msg.c_str())).c_str());

        printer("Initialized timer\n\nInitializing I2C protocol");

        Wire.begin();
        printer("Initialized I2C protocol\n\nInitializing MPU6050");

        mpu.initialize();

        printer("Initialized MPU6050\n\nTesting MPU6050 connection");

        if(!mpu.testConnection()) while(true) printer("MPU6050 connection failed");

        printer("Initialized MPU6050\n\nInitializing MPU6050 DMP");

        auto er = mpu.initDMP(mpuInterruptPort);
        if (er) while (true) printer(er.msg.c_str());

        printer("Initialized MPU6050 DMP\n\n");
        
        printer("Initializing platform controller shield");
        Vex5.begin();    
        
        printer("Initialized platform controller shield\n\n");
        
        printer("Initializing platform controller");
        er = platform.init(vislib::core::Array<VEX5_PORT_t>({(VEX5_PORT_t)1, (VEX5_PORT_t)2, (VEX5_PORT_t)3, (VEX5_PORT_t)4}));
        
        printer("Initialized platform controller");
        
        printer("\nDone initialization");
    }

};

}
