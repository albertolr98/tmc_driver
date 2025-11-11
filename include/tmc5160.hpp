#pragma once
#include "tmc_driver.hpp"
#include <cstdint>
#include <cstddef>

class TMC5160 : public TMCDriver {
public:
    bool init() override;
    bool setSpeed(int motor_id, float rpm) override;
    float readPosition(int motor_id) override;

private:
    int spi_fd_ = -1;
    uint8_t icID_ = 0;

    // SPI control interno
    bool initSPI(const char* device = "/dev/spidev0.0", uint32_t speed = 1000000);
    void closeSPI();

    // Callbacks requeridos por la TMC-API
    static uint8_t getBusType(uint16_t icID);
    static void readWriteSPI(uint16_t icID, uint8_t *data, size_t length);
};
