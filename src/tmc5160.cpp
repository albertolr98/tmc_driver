#include "tmc5160.hpp"
#include "spi_bus.hpp"
#include "tmc_spi_registry.hpp"

extern "C" {
    #include "tmc/helpers/Constants.h"
    #include "tmc/ic/TMC5160/TMC5160.h"
    #include "tmc/ic/TMC5160/TMC5160_HW_Abstraction.h"
}

#include <cmath>
#include <cstdio>
#include <gpiod.h>
#include <iostream>

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
    : icID_(tmc::allocateIcID()), cs_pin_(cs_gpio), en_pin_(en_gpio) {}

bool TMC5160::init()
{
    if (!enableDriver(true)) {
        std::cerr << "No se pudo habilitar el pin EN\n";
        return false;
    }
    tmc::registerCSPin(icID_, cs_pin_);
    // Inicializar la caché de la TMC-API y aplicar la configuración por defecto
    // (reset, limpieza de flags y parámetros de rampa/motor) igual que la
    // inicialización previa que compartiste.
    tmc5160_initCache();

    // Reset del driver y limpieza de flags de estado
    tmc5160_writeRegister(icID_, TMC5160_GCONF, 0x00000000); // Reset a valores por defecto
    tmc5160_writeRegister(icID_, TMC5160_GSTAT, 0x00000007); // Limpiar flags de estado

    // Reconfigurar registros principales para un motor NEMA17 de 2.5A RMS
    tmc5160_writeRegister(icID_, TMC5160_GCONF, 0x00000004);           // StealthChop + interpolation
    tmc5160_writeRegister(icID_, TMC5160_CHOPCONF, 0x000100C3);        // SpreadCycle base (16 µsteps) for suavidad
    tmc5160_writeRegister(icID_, TMC5160_IHOLD_IRUN, 0x00061908);      // IHOLD=8 (~0.87A), IRUN=25 (~2.5A), IHOLDDELAY=6
    tmc5160_writeRegister(icID_, TMC5160_TPOWERDOWN, 0x0000000A);      // 10 * 2^18 clock cycles
    tmc5160_writeRegister(icID_, TMC5160_TPWMTHRS, 0x000001F4);        // TPWM_THRS=500
    tmc5160_writeRegister(icID_, TMC5160_A1, 500); // A1=500
    tmc5160_writeRegister(icID_, TMC5160_V1, 5000); // V1=5000
    tmc5160_writeRegister(icID_, TMC5160_AMAX, 500); // AMAX=500
    tmc5160_writeRegister(icID_, TMC5160_DMAX, 700); // DMAX=700
    tmc5160_writeRegister(icID_, TMC5160_D1, 1400); // D1=1400
    tmc5160_writeRegister(icID_, TMC5160_VSTOP, 10); // VSTOP=10
    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS); // Velocity mode
    tmc5160_writeRegister(icID_, TMC5160_VMAX, 0); // VMAX=0 (stop initially)

    std::cout << "TMC5160 iniciado (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
    return true;
}

bool TMC5160::setSpeed(int motor_id, float rpm)
{
    (void)motor_id;
    // Convert RPM to VMAX (microsteps per second) using motor and driver settings.
    // Init config sets CHOPCONF with MRES=0 (native 256 microsteps), and motor is 1.8° => 200 steps/rev.
    const unsigned steps_per_rev = 200; // 1.8° per step
    const unsigned microsteps = 256;    // MRES=0 => native 256 µsteps/step in CHOPCONF used in init()
    constexpr double tmc5160_clock_hz = 12'000'000.0; // Datasheet: internal clock fCLK = 12 MHz
    constexpr double velocity_time_unit = static_cast<double>(1u << 24) / tmc5160_clock_hz;
    // VMAX units are µsteps/t where t = 2^24 / fCLK. Multiply desired µsteps/s by t to match register units.

    const double microsteps_per_rev = static_cast<double>(steps_per_rev) * static_cast<double>(microsteps);
    const double microsteps_per_second = (static_cast<double>(rpm) * microsteps_per_rev) / 60.0;
    double vmax_d = microsteps_per_second * velocity_time_unit;
    // clamp to allowed range
    if (vmax_d > static_cast<double>(TMC5160_MAX_VELOCITY)) vmax_d = static_cast<double>(TMC5160_MAX_VELOCITY);
    if (vmax_d < -static_cast<double>(TMC5160_MAX_VELOCITY)) vmax_d = -static_cast<double>(TMC5160_MAX_VELOCITY);

    int32_t vmax = static_cast<int32_t>(std::lround(vmax_d));
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

    const bool versionOk = (version == 0x21 || version == 0x30);
    const bool vmaxEchoOk = (echoed == testVmax);

    std::printf("[%s] GCONF=0x%08X | INP_OUT=0x%08X (version 0x%02X) | VMAX echo %s\n",
                tag, gconf, inpOut, version, vmaxEchoOk ? "OK" : "FAIL");
    if (!versionOk) {
        std::printf("  -> Firma esperada 0x21/0x30, recibido 0x%02X\n", version);
    }

    return versionOk && vmaxEchoOk;
}


// Librerías para TMC-API
extern "C" void tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length)
{
    unsigned int csPin = 0;
    if (!tmc::csPinFor(icID, csPin)) {
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
