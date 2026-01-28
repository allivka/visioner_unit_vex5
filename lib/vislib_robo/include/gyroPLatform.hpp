#pragma once
#include "platform.hpp"

namespace vislib::platform {
    
template <typename Controller_t, typename Time_t> class GyroPlatform : public Platform<Controller_t> {
protected:
    calculators::GyroPidCalculator<Time_t> calculator{};
    core::UniquePtr<gyro::YawGetter<double>> yawGetter{};
    core::TimeGetter<Time_t> timeGetter{};
    double headAngle{};
    
    
public:

    
    GyroPlatform(
        const calculators::GyroPidCalculator<Time_t>& calculator,
        core::UniquePtr<gyro::YawGetter<double>>&& yawGetter,
        core::TimeGetter<Time_t>&& timeGetter,
        const PlatformMotorConfig& configuration,
        size_t parallelismPrecision = 0) noexcept
        : calculator(calculator), yawGetter(core::move(yawGetter)), timeGetter(core::move(timeGetter)), Platform<Controller_t>(configuration, parallelismPrecision) {
            
    }

    core::Error go(const double speed, const double angularSpeed = 0, const double speedK = 1, bool isAbsoluteHead = false) noexcept {

        auto time = timeGetter();
        
        core::Result<double> angle = yawGetter->getYaw();
        if(angle.isError()) return angle.error();
        
        core::Result<PlatformMotorSpeeds> speeds = calculator.calculateSpeeds(
            time,
            angle,
            headAngle,
            speed,
            speedK,
            angularSpeed
        );
        if (speeds.isError()) return speeds.error();
        
        core::Error err = this->setSpeeds(speeds());
        
        if(err.isError()) return err;
        
        return {};
    }
    
};

} //vislib::platform
