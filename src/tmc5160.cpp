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
#include <chrono>
#include <algorithm>
#include <gpiod.h>
#include <iostream>
#include <thread>

namespace
{
constexpr double kStepsPerRev = 200.0;            // 1.8° per step
constexpr double kMicrostepsPerStep = 256.0;      // CHOPCONF MRES=0
constexpr double kMicrostepsPerRev = kStepsPerRev * kMicrostepsPerStep;
constexpr double kTau = 6.28318530717958647692;   // 2 * pi
constexpr double kMicrostepToRad = kTau / kMicrostepsPerRev;
constexpr double kTmc5160ClockHz = 12'000'000.0;  // Datasheet fCLK
constexpr double kVelocityTimeUnit =
  static_cast<double>(1u << 24) / kTmc5160ClockHz;  // µsteps/s -> register factor
// Tiempo fijo de rampa: todas las ruedas llegan al objetivo en el mismo tiempo (más suave)
constexpr double kRampTimeSeconds = 3.0;  // sube para suavizar aceleración
// Escala de aceleración en unidades de registro (mismo factor que VMAX^2).
constexpr double kAccelerationTimeUnit = kVelocityTimeUnit * kVelocityTimeUnit;
constexpr double kMinAccelerationReg = 1.0;
constexpr double kMaxAccelerationReg = static_cast<double>(TMC5160_MAX_ACCELERATION);

double toMicrostepsPerSecond(double rad_per_sec)
{
    return std::fabs(rad_per_sec) / kMicrostepToRad;
}

double vactualToMicrostepsPerSecond(uint32_t raw_vactual)
{
    const int32_t vel = (raw_vactual & 0x800000) ? (raw_vactual | 0xFF000000) : (raw_vactual & 0x00FFFFFF);
    return static_cast<double>(vel) / kVelocityTimeUnit;
}

uint32_t accelToRegister(double microsteps_per_second2)
{
    const double reg_value = microsteps_per_second2 * kAccelerationTimeUnit;
    const double clamped = std::clamp(reg_value, kMinAccelerationReg, kMaxAccelerationReg);
    return static_cast<uint32_t>(std::lround(clamped));
}
}  // namespace

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
    tmc5160_writeRegister(icID_, TMC5160_IHOLD_IRUN, 0x00061004);      // IHOLD=4 (~0.44A), IRUN=16 (~1.6A), IHOLDDELAY=6
    tmc5160_writeRegister(icID_, TMC5160_TPOWERDOWN, 0x0000000A);      // 10 * 2^18 clock cycles
    tmc5160_writeRegister(icID_, TMC5160_TPWMTHRS, 0x000001F4);        // TPWM_THRS=500
    tmc5160_writeRegister(icID_, TMC5160_VSTART, 0); // VSTART=0 para arrancar siempre desde reposo
    tmc5160_writeRegister(icID_, TMC5160_A1, 500); // A1=500
    tmc5160_writeRegister(icID_, TMC5160_V1, 5000); // V1=5000
    tmc5160_writeRegister(icID_, TMC5160_AMAX, 500); // AMAX=500
    tmc5160_writeRegister(icID_, TMC5160_DMAX, 700); // DMAX=700
    tmc5160_writeRegister(icID_, TMC5160_D1, 1400); // D1=1400
    tmc5160_writeRegister(icID_, TMC5160_VSTOP, 10); // VSTOP=10
    tmc5160_writeRegister(icID_, TMC5160_XACTUAL, 0); // XACTUAL=0

    // Detén cualquier movimiento previo y deja VMAX a 0 antes de aceptar comandos.
    tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, TMC5160_MODE_HOLD);

    std::cout << "TMC5160 iniciado (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
    return true;
}

bool TMC5160::setSpeed(float rad_per_sec)
{

    // 1. Determinamos la dirección y la magnitud
    // Nota: Mantenemos siempre el modo velocidad (VELPOS o VELNEG).
    // NUNCA usamos HOLD (3) para parar, porque HOLD mantiene la velocidad actual.
    bool negative_dir = rad_per_sec < 0.0f;

    const uint32_t raw_vactual = tmc5160_readRegister(icID_, TMC5160_VACTUAL);
    const double current_microsteps_per_second_signed = vactualToMicrostepsPerSecond(raw_vactual);
    const double current_microsteps_per_second_abs = std::fabs(current_microsteps_per_second_signed);
    const bool currentlyNegative = current_microsteps_per_second_signed < 0.0;

    // 2. Caso de parada (velocidad cercana a 0)
    if (std::fabs(rad_per_sec) < 1e-3f) {
        // Si ya estamos parados, no hacemos nada
        if (current_microsteps_per_second_abs < 1e-6) return true;

        // Rampa de frenado calculada para llegar a 0 en kRampTimeSeconds
        const uint32_t decel_reg = accelToRegister(current_microsteps_per_second_abs / kRampTimeSeconds);

        tmc5160_writeRegister(icID_, TMC5160_AMAX, decel_reg);
        tmc5160_writeRegister(icID_, TMC5160_DMAX, decel_reg);

        tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, currentlyNegative ? TMC5160_MODE_VELNEG : TMC5160_MODE_VELPOS);
        tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
        
        return true;
    }

    // 2b. Si la orden invierte el sentido y aún vamos rápido, primero frena suavemente a 0.
    if ((currentlyNegative != negative_dir) && (current_microsteps_per_second_abs > 1e-3)) {
        const uint32_t decel_reg = accelToRegister(current_microsteps_per_second_abs / kRampTimeSeconds);
        tmc5160_writeRegister(icID_, TMC5160_AMAX, decel_reg);
        tmc5160_writeRegister(icID_, TMC5160_DMAX, decel_reg);
        tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, currentlyNegative ? TMC5160_MODE_VELNEG : TMC5160_MODE_VELPOS);
        tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
        return true;  // vuelve a llamar a setSpeed() en el siguiente ciclo con la nueva dirección
    }

    // 3. Caso de movimiento normal
    const double microsteps_per_second = toMicrostepsPerSecond(rad_per_sec);
    const uint32_t accel_reg = accelToRegister(microsteps_per_second / kRampTimeSeconds);

    // Misma aceleración/dec para las tres ruedas → misma rampa temporal
    tmc5160_writeRegister(icID_, TMC5160_AMAX, accel_reg);
    tmc5160_writeRegister(icID_, TMC5160_DMAX, accel_reg);
    // Segmento suave inicial: V1 a un tercio del objetivo y A1/D1 a ~1/3 de AMAX
    const uint32_t a1_reg = std::max<uint32_t>(1u, accel_reg / 3u);
    const uint32_t d1_reg = std::max<uint32_t>(1u, accel_reg / 3u);

    double vmax_d = microsteps_per_second * kVelocityTimeUnit;
    
    if (vmax_d > static_cast<double>(TMC5160_MAX_VELOCITY)) 
        vmax_d = static_cast<double>(TMC5160_MAX_VELOCITY);

    int32_t vmax = static_cast<int32_t>(std::lround(vmax_d));
    const uint32_t v1_reg = static_cast<uint32_t>(std::max(1.0, vmax_d / 3.0));

    // CORRECCIÓN: incluso si vmax es 0 por redondeo, forzamos VELPOS/VELNEG (no HOLD).
    uint8_t rampMode = negative_dir ? TMC5160_MODE_VELNEG : TMC5160_MODE_VELPOS;

    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, rampMode);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, vmax);
    tmc5160_writeRegister(icID_, TMC5160_V1, v1_reg);
    tmc5160_writeRegister(icID_, TMC5160_A1, a1_reg);
    tmc5160_writeRegister(icID_, TMC5160_D1, d1_reg);
    return true;
}

float TMC5160::readPosition()
{
    const int32_t pos = tmc5160_readRegister(icID_, TMC5160_XACTUAL);
    const double radians = static_cast<double>(pos) * kMicrostepToRad;
    return static_cast<float>(radians);
}

float TMC5160::readSpeed()
{
    // VACTUAL is a 24-bit signed value in the lower bits of a 32-bit register
    const uint32_t raw = tmc5160_readRegister(icID_, TMC5160_VACTUAL);
    // Sign-extend from 24-bit to 32-bit
    const int32_t vel = (raw & 0x800000) ? (raw | 0xFF000000) : (raw & 0x00FFFFFF);
    const double microsteps_per_second = static_cast<double>(vel) / kVelocityTimeUnit;
    const double rad_per_second = microsteps_per_second * kMicrostepToRad;
    return static_cast<float>(rad_per_second);
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

void TMC5160::shutdown()
{
    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, TMC5160_MODE_HOLD);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
    enableDriver(false);
    std::cout << "TMC5160 detenido (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
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
