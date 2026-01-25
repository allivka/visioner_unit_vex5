#pragma once
#include "platform.hpp"

namespace vislib::platform {
    
template <typename Controller_t, typename Time_t> class GyroPlatform : public Platform<Controller_t> {
protected:
    calculators::GyroPidCalculator<Time_t> calculator{};
    
public:

    double relativeHead{};
    
    
    GyroPlatform(const calculators::GyroPidCalculator<Time_t>& calculator, PlatformMotorConfig configuration, size_t parallelismPrecision = 0) noexcept
    : calculator(calculator), Platform<Controller_t>(configuration, parallelismPrecision) {}
    
};

} //vislib::platform
