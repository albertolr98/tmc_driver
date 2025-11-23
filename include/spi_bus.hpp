#pragma once
#include <cstdint>
#include <mutex>
#include <gpiod.h>
#include <unordered_map>

class SPIBus {
public:
    static bool init(const char* device = "/dev/spidev0.0", uint32_t speed = 1000000);
    static void enableLogging(bool enable);
    static void close();

    static bool transfer(uint8_t* data, size_t len, unsigned int cs_pin);

private:
    static int fd_;
    static std::mutex mtx_;
    static bool logEnabled_;
    
    static gpiod_chip* gpio_chip_;
    static std::unordered_map<unsigned int, gpiod_line*> cs_lines_;
};