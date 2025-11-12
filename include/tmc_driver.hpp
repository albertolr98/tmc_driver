#pragma once

class TMCDriver {
public:
    virtual bool init() = 0;
    virtual bool setSpeed(int motor_id, float rpm) = 0;
    virtual float readPosition(int motor_id) = 0;
    virtual bool checkComms(const char* label) = 0;
    virtual ~TMCDriver() = default;
};
