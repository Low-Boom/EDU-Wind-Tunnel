/*
 * FanMap.h
 *
 * The measured fan characteristic: airspeed (and fan RPM, if the tach wire
 * is connected) at a series of fixed PWM values. The 'fanmap' command
 * measures it; the auto-tunes use it to find the PWM for their target
 * instead of a fixed estimate.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef FAN_MAP_H
#define FAN_MAP_H

#include <Arduino.h>

const int FAN_MAP_MAX_POINTS = 12;

struct FanMapPoint {
    int pwm;
    float rpm;        // 0 without a tach signal
    float airspeed;   // m/s, mean of the measurement window
};

class FanMap {
public:
    void clear() { _count = 0; }

    /** Add the next point; PWM must rise from point to point. False if full. */
    bool add(int pwm, float rpm, float airspeed);

    int size() const { return _count; }
    const FanMapPoint &point(int i) const { return _points[i]; }

    /** Highest airspeed of the map (0 if empty). */
    float maxAirspeed() const;

    /**
     * True if the map can guide a tune: at least two points, and the
     * airspeed rises to at least minTopAirspeed.
     */
    bool usable(float minTopAirspeed) const;

    /**
     * PWM that gives this airspeed, by linear interpolation between the
     * points (below the first point: along the first rising segment, at
     * least 1). -1 if the airspeed is above the map.
     */
    int pwmForAirspeed(float airspeed) const;

    /**
     * Change of airspeed per PWM step from the previous point to point i
     * (the fan's gain there, m/s per PWM); 0 for the first point.
     */
    float gainAt(int i) const;

private:
    FanMapPoint _points[FAN_MAP_MAX_POINTS];
    int _count = 0;
};

#endif // FAN_MAP_H
