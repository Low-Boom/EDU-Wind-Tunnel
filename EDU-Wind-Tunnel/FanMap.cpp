/*
 * FanMap.cpp
 *
 * Measured fan characteristic; see FanMap.h.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "FanMap.h"

bool FanMap::add(int pwm, float rpm, float airspeed) {
    if (_count >= FAN_MAP_MAX_POINTS) return false;
    _points[_count++] = {pwm, rpm, airspeed};
    return true;
}

float FanMap::maxAirspeed() const {
    float top = 0.0;
    for (int i = 0; i < _count; i++) {
        if (_points[i].airspeed > top) top = _points[i].airspeed;
    }
    return top;
}

bool FanMap::usable(float minTopAirspeed) const {
    return _count >= 2 && maxAirspeed() >= minTopAirspeed;
}

// The measured airspeed can dip from one point to the next (noise, or a
// fan that is already at full speed). The search uses the running maximum,
// so the curve it interpolates on never falls, and it uses only segments
// that rise.
int FanMap::pwmForAirspeed(float airspeed) const {
    if (_count < 2 || airspeed > maxAirspeed()) return -1;
    float lowV = _points[0].airspeed;       // running maximum
    int lowPwm = _points[0].pwm;
    for (int i = 1; i < _count; i++) {
        float highV = _points[i].airspeed;
        if (highV <= lowV) continue;            // flat or falling: skip this point
        int highPwm = _points[i].pwm;
        if (airspeed <= highV) {
            // In this segment; below the first point this extrapolates along it
            float pwm = lowPwm + (airspeed - lowV) * (highPwm - lowPwm) / (highV - lowV);
            int rounded = (int)round(pwm);
            return (rounded < 1) ? 1 : rounded;
        }
        lowV = highV;
        lowPwm = highPwm;
    }
    return -1;   // no rising segment
}

float FanMap::gainAt(int i) const {
    if (i <= 0 || i >= _count) return 0.0;
    int dPwm = _points[i].pwm - _points[i - 1].pwm;
    if (dPwm == 0) return 0.0;
    return (_points[i].airspeed - _points[i - 1].airspeed) / dPwm;
}
