#include "../include/visioner_unit_vex5.hpp"

namespace VisionerUnitVex5 {

vislib::platform::PlatformMotorConfig Visioner::platformConfig(vislib::core::Array<vislib::motor::MotorInfo>({
        vislib::motor::MotorInfo(90, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
        vislib::motor::MotorInfo(180, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
        vislib::motor::MotorInfo(-90, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange),
        vislib::motor::MotorInfo(0, motorDistance, wheelR, motorUseSpeedRange, motorInterfaceAngularSpeedRange)
}));

vislib::core::IncrementTimer<int64_t> Visioner::timer([]() -> vislib::core::Result<int64_t> {
        return static_cast<int64_t>(millis());
});

vislib::binds::mpu6050::GyroscopeDMP<0> Visioner::mpu;

vislib::core::UniquePtr<vislib::platform::GyroPlatform<vislib::binds::vex5::motor::V5MotorController, int64_t>> Visioner::platform{nullptr};

}