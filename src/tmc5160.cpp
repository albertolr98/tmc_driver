#include "tmc5160.hpp"
#include "spi_bus.hpp"

extern "C" {
    #include "tmc/helpers/Constants.h"
    #include "tmc/ic/TMC5160/TMC5160.h"
    #include "tmc/ic/TMC5160/TMC5160_HW_Abstraction.h"
}

#include <atomic>
#include <cstdio>
#include <gpiod.h>
#include <iostream>
#include <mutex>
#include <unordered_map>

namespace {

std::atomic<uint16_t> nextIcID{0};

class CSPinRegistry {
public:
    void registerPin(uint16_t icID, unsigned int pin)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        csPins_[icID] = pin;
    }

    bool pinFor(uint16_t icID, unsigned int& pin)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = csPins_.find(icID);
        if (it == csPins_.end())
            return false;
        pin = it->second;
        return true;
    }

private:
    std::mutex mtx_;
    std::unordered_map<uint16_t, unsigned int> csPins_;
};

CSPinRegistry& csRegistry()
{
    static CSPinRegistry instance;
    return instance;
}
} // namespace

// Pin activo en bajo: EN=0 → habilitado
bool TMC5160::enableDriver(bool state)
{
    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) return false;

    gpiod_line* line = gpiod_chip_get_line(chip, en_pin_);
    if (!line) { gpiod_chip_close(chip); return false; }

    if (gpiod_line_request_output(line, "tmc_enable", 1) < 0) {
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return false;
    }

    // state=true → habilitar (nivel bajo)
    gpiod_line_set_value(line, state ? 0 : 1);

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return true;
}
TMC5160::TMC5160(unsigned int cs_gpio, unsigned int en_gpio)
    : icID_(nextIcID++), cs_pin_(cs_gpio), en_pin_(en_gpio) {}

bool TMC5160::init()
{
    if (!enableDriver(true)) {
        std::cerr << "No se pudo habilitar el pin EN\n";
        return false;
    }
    csRegistry().registerPin(icID_, cs_pin_);
    std::cout << "TMC5160 iniciado (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
    return true;
}

bool TMC5160::setSpeed(int motor_id, float rpm)
{
    (void)motor_id;
    int32_t vmax = static_cast<int32_t>(rpm * 100);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, vmax);
    return true;
}

float TMC5160::readPosition(int motor_id)
{
    (void)motor_id;
    int32_t pos = 0;
    pos = tmc5160_readRegister(icID_, TMC5160_XACTUAL);
    return static_cast<float>(pos);
}

bool TMC5160::checkComms(const char* label)
{
    const char* tag = label ? label : "TMC5160";
    const uint32_t gconf = tmc5160_readRegister(icID_, TMC5160_GCONF);
    const uint32_t inpOut = tmc5160_readRegister(icID_, TMC5160_INP_OUT);
    const uint32_t version = (inpOut & TMC5160_VERSION_MASK) >> TMC5160_VERSION_SHIFT;

    const uint32_t originalVmax = tmc5160_readRegister(icID_, TMC5160_VMAX);
    const uint32_t testVmax = 0x000A000; // pequeño valor de prueba dentro del rango
    tmc5160_writeRegister(icID_, TMC5160_VMAX, testVmax);
    const uint32_t echoed = tmc5160_readRegister(icID_, TMC5160_VMAX);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, originalVmax);

    const bool versionOk = (version == 0x30);
    const bool vmaxEchoOk = (echoed == testVmax);

    std::printf("[%s] GCONF=0x%08X | INP_OUT=0x%08X (version 0x%02X) | VMAX echo %s\n",
                tag, gconf, inpOut, version, vmaxEchoOk ? "OK" : "FAIL");
    if (!versionOk) {
        std::printf("  -> Firma esperada 0x30, recibido 0x%02X\n", version);
    }

    return versionOk && vmaxEchoOk;
}


// Librerías para TMC-API
extern "C" void tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length)
{
    unsigned int csPin = 0;
    if (!csRegistry().pinFor(icID, csPin)) {
        std::cerr << "CS no registrado para icID=" << icID << "\n";
        return;
    }
    SPIBus::transfer(data, length, csPin);
}

extern "C" TMC5160BusType tmc5160_getBusType(uint16_t)
{
    return TMC5160BusType::IC_BUS_SPI;
}

// Dummy UART, no implementado
extern "C" bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength) {return false;}
extern "C" uint8_t tmc5160_getNodeAddress(uint16_t icID) { return 0; }
