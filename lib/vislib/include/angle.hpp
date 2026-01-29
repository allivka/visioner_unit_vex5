#pragma once

#include "math.hpp"

namespace vislib::core {

template <typename T = double, typename O = double> class Angle {
protected:
    T value{};
    
public:
    
    enum class CastType {
        degrees,
        radians
    };
    
    CastType castType = CastType::degrees;
    
    static inline T fixDegrees(T angle) {
        angle = fmod(angle, 360);
        if(angle < 0)  angle += 360;
        
        return angle;
    }
    
    static Angle fromDegrees(const T& value) {
        return Angle(fixDegrees(value));
    }
    
    static Angle fromRadians(const T& value) {
        return Angle(fixDegrees(rad2Deg(value)));
    }
    
    static Angle fromDeg(const T& value) {
        return Angle(fixDegrees(value));
    }
    
    static Angle fromRad(const T& value) {
        return Angle(fixDegrees(rad2Deg(value)));
    }
    
    Angle(const T& value) : value(fixDegrees(value)) {}
    
    Angle() = default;
    Angle(const Angle&) = default;
    Angle(Angle&&) = default;
    Angle& operator=(const Angle&) = default;
    Angle& operator=(Angle&&) = default;
    ~Angle() = default;
    
    inline void setDegrees(const T& value) {
        this->value = fixDegrees(value);
    }
    
    inline void setRadians(const T& value) {
        this->value = fixDegrees(rad2Deg(value));
    }
    
    inline T degrees() {
        return value;
    }
    
    inline T radians() {
        return deg2Rad(value);
    }
    
    inline T deg() {
        return value;
    }
    
    inline T rad() {
        return deg2Rad(value);
    }
    
    inline operator T() {
        switch (castType) {
            
            case CastType::radians:
                return deg2Rad(value);
            
            case CastType::degrees:
            default:
                return value;
            
        }
    }
    
    inline Angle& operator++() {
        value = fixDegrees(value + 1);
        return *this;
    }
    
    inline Angle& operator++(int) {
        value = fixDegrees(value + 1);
        return *this;
    }
    
    inline Angle& operator--() {
        value = fixDegrees(value - 1);
        return *this;
    }
    
    inline Angle& operator--(int) {
        value = fixDegrees(value - 1);
        return *this;
    }
    
    inline Angle& operator+=(const O& v) {
        value = fixDegrees(value + v);
        return *this;
    }
    
    inline Angle& operator-=(const O& v) {
        value = fixDegrees(value - v);
        return *this;
    }
    
    inline Angle& operator*=(const O& v) {
        value = fixDegrees(value * v);
        return *this;
    }
    
    inline Angle& operator/=(const O& v) {
        value = fixDegrees(value / v);
        return *this;
    }
    
    inline Angle& operator%=(const O& v) {
        value = fixDegrees(fmod(static_cast<double>(value), static_cast<double>(v)));
        return *this;
    }
    
    inline Angle operator+(const O& v) {
        return Angle<T, O>(fixDegrees(value + v));
    }
    
    inline Angle operator-(const O& v) {
        return Angle<T, O>(fixDegrees(value - v));
    }
    
    inline Angle operator*(const O& v) {
        return Angle<T, O>(fixDegrees(value * v));
    }
    
    inline Angle operator/(const O& v) {
        return Angle<T, O>(fixDegrees(value / v));
    }
    
    inline Angle operator%(const O& v) {
        return Angle<T, O>(fixDegrees(fmod(static_cast<double>(value), static_cast<double>(v))));
    }
    
    
    
    inline Angle& operator+=(const Angle& v) {
        value = fixDegrees(value + v.value);
        return *this;
    }
    
    inline Angle& operator-=(const Angle& v) {
        value = fixDegrees(value - v.value);
        return *this;
    }
    
    inline Angle& operator*=(const Angle& v) {
        value = fixDegrees(value * v.value);
        return *this;
    }
    
    inline Angle& operator/=(const Angle& v) {
        value = fixDegrees(value / v.value);
        return *this;
    }
    
    inline Angle& operator%=(const Angle& v) {
        value = fixDegrees(fmod(static_cast<double>(value), static_cast<double>(v.value)));
        return *this;
    }
    
    inline Angle operator+(const Angle& v) {
        return Angle<T, O>(fixDegrees(value + v.value));
    }
    
    inline Angle operator-(const Angle& v) {
        return Angle<T, O>(fixDegrees(value - v.value));
    }
    
    inline Angle operator*(const Angle& v) {
        return Angle<T, O>(fixDegrees(value * v.value));
    }
    
    inline Angle operator/(const Angle& v) {
        return Angle<T, O>(fixDegrees(value / v.value));
    }
    
    inline Angle operator%(const Angle& v) {
        return Angle<T, O>(fixDegrees(fmod(static_cast<double>(value), static_cast<double>(v.value))));
    }
    
    inline friend Angle  operator+(const O& v, const Angle<T, O>& a) {
        return Angle(fixDegrees(a.value + v));
    }
    
    inline friend Angle operator*(const O& v, const Angle<T, O>& a) {
        return Angle(fixDegrees(a.value * v));
    }
    
};


} //namespace vislib::core
