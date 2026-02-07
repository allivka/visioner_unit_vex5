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

}