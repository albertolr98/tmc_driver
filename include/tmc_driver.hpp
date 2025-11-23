#pragma once

class TMCDriver {
public:
    virtual bool init() = 0;
    virtual bool setSpeed(float rad_per_sec) = 0;
    virtual float readPosition() = 0;
    virtual float readSpeed() = 0;
    virtual bool checkComms(const char* label) = 0;
    virtual void shutdown() = 0;
    virtual ~TMCDriver() = default;
};
