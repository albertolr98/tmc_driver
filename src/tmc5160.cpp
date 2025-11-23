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
// ==========================================
// 1. Constantes Físicas del Motor
// ==========================================
constexpr double kMotorStepsPerRev = 200.0;         // Motor estándar de 1.8 grados/paso
constexpr double kMicrostepsPerStep = 256.0;        // Interpolación interna (MRES=0)
constexpr double kTotalMicrostepsPerRev = kMotorStepsPerRev * kMicrostepsPerStep;

constexpr double kRadiansPerRev = 6.28318530717958647692; // 2 * PI
constexpr double kRadiansPerMicrostep = kRadiansPerRev / kTotalMicrostepsPerRev;

constexpr double kDriverClockFreqHz = 12'000'000.0; // Frecuencia de reloj del chip

// ==========================================
// 2. Configuración de Sincronización
// ==========================================
// Tiempo fijo que tardarán TODAS las ruedas en alcanzar su velocidad objetivo.
// Al forzar un tiempo constante, garantizamos que los motores terminen
// de acelerar al mismo tiempo, manteniendo la trayectoria recta.
constexpr double kRampTimeSeconds = 0.5; 

// ==========================================
// 3. Factores de Conversión (Datasheet TMC5160)
// ==========================================

// Factor para convertir Velocidad (Hz) -> Registro VMAX
// v_reg = v_hz * (2^24 / fCLK)
constexpr double kVelocityToRegisterScale = static_cast<double>(1u << 24) / kDriverClockFreqHz;

// Factor para convertir Aceleración (Hz/s) -> Registro AMAX
// a_reg = a_hz * (2^41 / fCLK^2)
// Se usa 1ULL<<41 para asegurar precisión en 64 bits.
constexpr double kAccelerationToRegisterScale = static_cast<double>(1ULL << 41) / (kDriverClockFreqHz * kDriverClockFreqHz);

// ==========================================
// 4. Funciones Auxiliares de Conversión
// ==========================================

// Convierte de Radianes/segundo a Micropasos/segundo (Valor absoluto)
double RadsPerSec_To_MicrostepsPerSec(double rad_per_sec)
{
    return std::fabs(rad_per_sec) / kRadiansPerMicrostep;
}

// Convierte el valor crudo del registro VACTUAL a Micropasos/segundo (con signo)
double Vactual_To_MicrostepsPerSec(uint32_t raw_vactual)
{
    // VACTUAL es un valor de 24 bits en complemento a dos.
    // Verificamos el bit 23 para extender el signo a 32 bits si es negativo.
    const int32_t vel = (raw_vactual & 0x800000) ? (raw_vactual | 0xFF000000) : (raw_vactual & 0x00FFFFFF);
    return static_cast<double>(vel) / kVelocityToRegisterScale; 
}

// Convierte Micropasos/segundo^2 a valor de registro de Aceleración
uint32_t MicrostepsPerSecSq_To_AccelReg(double microsteps_per_second2)
{
    const double reg_value = microsteps_per_second2 * kAccelerationToRegisterScale;
    // Limitamos el valor para evitar desbordamientos del registro (aprox 65535)
    // y aseguramos que sea al menos 1 para evitar divisiones por cero internas.
    const double clamped = std::clamp(reg_value, 1.0, 65535.0);
    return static_cast<uint32_t>(std::lround(clamped));
}

}  // namespace anónimo

// ==========================================
// Implementación de la Clase TMC5160
// ==========================================

TMC5160::TMC5160(unsigned int cs_gpio, unsigned int en_gpio)
    : icID_(tmc::allocateIcID()), cs_pin_(cs_gpio), en_pin_(en_gpio) {}

// Gestiona el pin físico ENABLE a través de libgpiod
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

    // La lógica es inversa: Low (0) = Habilitado, High (1) = Deshabilitado
    gpiod_line_set_value(line, state ? 0 : 1);

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return true;
}

bool TMC5160::init()
{
    if (!enableDriver(true)) {
        std::cerr << "No se pudo habilitar el pin EN\n";
        return false;
    }
    tmc::registerCSPin(icID_, cs_pin_);
    
    // Inicializa la estructura de caché de la librería TMC-API
    tmc5160_initCache();

    // Reset de registros globales y limpieza de errores (GSTAT)
    tmc5160_writeRegister(icID_, TMC5160_GCONF, 0x00000000); 
    tmc5160_writeRegister(icID_, TMC5160_GSTAT, 0x00000007); 

    // Configuración de Chopper (StealthChop + SpreadCycle) y Corrientes
    tmc5160_writeRegister(icID_, TMC5160_GCONF, 0x00000004);           
    tmc5160_writeRegister(icID_, TMC5160_CHOPCONF, 0x000100C3);        
    tmc5160_writeRegister(icID_, TMC5160_IHOLD_IRUN, 0x00061004);      
    tmc5160_writeRegister(icID_, TMC5160_TPOWERDOWN, 0x0000000A);      
    tmc5160_writeRegister(icID_, TMC5160_TPWMTHRS, 0x000001F4);        
    
    // Valores iniciales seguros para los registros de rampa.
    // Estos se sobrescribirán dinámicamente en setSpeed.
    tmc5160_writeRegister(icID_, TMC5160_VSTART, 0); 
    tmc5160_writeRegister(icID_, TMC5160_A1, 0); 
    tmc5160_writeRegister(icID_, TMC5160_V1, 0); 
    tmc5160_writeRegister(icID_, TMC5160_VSTOP, 10); // Umbral de parada segura
    tmc5160_writeRegister(icID_, TMC5160_XACTUAL, 0); 

    // Asegurar que el motor arranca detenido
    tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, TMC5160_MODE_HOLD);

    std::cout << "TMC5160 iniciado (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
    return true;
}

// Lógica principal de movimiento sincronizado por tiempo
bool TMC5160::setSpeed(float rad_per_sec)
{
    // 1. Filtrado de ruido en zona muerta
    if (std::fabs(rad_per_sec) < 1e-3f) {
        rad_per_sec = 0.0f;
    }

    bool negative_dir = rad_per_sec < 0.0f;

    // 2. Lectura de estado actual
    // Necesitamos saber a qué velocidad vamos realmente para calcular cuánto falta.
    uint32_t raw_vactual = tmc5160_readRegister(icID_, TMC5160_VACTUAL);
    double current_steps_s = Vactual_To_MicrostepsPerSec(raw_vactual);
    
    // 3. Cálculo de objetivos
    double target_steps_s = RadsPerSec_To_MicrostepsPerSec(std::fabs(rad_per_sec));
    
    // Asignamos signo al target para poder restar correctamente
    if (negative_dir) target_steps_s = -target_steps_s; 

    // 4. Cálculo de aceleración dinámica
    // Fórmula física: a = (v_final - v_inicial) / tiempo
    double delta_v = std::fabs(target_steps_s - current_steps_s);

    // Si la diferencia es muy pequeña, usamos una aceleración mínima por defecto.
    // De lo contrario, dividimos por la constante de tiempo (1.0s).
    double accel_steps_s2 = (delta_v < 1.0) ? 1000.0 : (delta_v / kRampTimeSeconds);

    // 5. Configuración de registros de rampa
    uint32_t amax_reg = MicrostepsPerSecSq_To_AccelReg(accel_steps_s2);
    
    // Configuramos una rampa LINEAL (Trapezoidal).
    // Al poner V1 = 0 y A1 = AMAX, deshabilitamos el perfil de 6 puntos (curva S).
    // Esto es necesario para que la relación Tiempo = Vel / Acel se cumpla estrictamente.
    tmc5160_writeRegister(icID_, TMC5160_A1, amax_reg);
    tmc5160_writeRegister(icID_, TMC5160_V1, 0);
    tmc5160_writeRegister(icID_, TMC5160_AMAX, amax_reg);
    tmc5160_writeRegister(icID_, TMC5160_DMAX, amax_reg);
    tmc5160_writeRegister(icID_, TMC5160_D1, amax_reg);
    
    // 6. Ejecución del movimiento
    double abs_target = std::fabs(target_steps_s);

    if (abs_target < 1e-3) {
        // Caso de PARADA:
        // Ponemos VMAX a 0, pero mantenemos el modo de velocidad (VELPOS/VELNEG)
        // para que el driver use la rampa de deceleración configurada (DMAX).
        tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
        
        // Mantenemos el signo de la velocidad actual para frenar en la dirección correcta.
        bool currently_neg = (current_steps_s < 0);
        tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, currently_neg ? TMC5160_MODE_VELNEG : TMC5160_MODE_VELPOS);

    } else {
        // Caso de MARCHA:
        int32_t vmax_reg = static_cast<int32_t>(std::lround(abs_target * kVelocityToRegisterScale));
        
        // Clamp de seguridad contra valores fuera de rango del datasheet
        if (vmax_reg > (int32_t)TMC5160_MAX_VELOCITY) vmax_reg = TMC5160_MAX_VELOCITY;

        // Establecemos el modo según la dirección deseada
        tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, negative_dir ? TMC5160_MODE_VELNEG : TMC5160_MODE_VELPOS);
        tmc5160_writeRegister(icID_, TMC5160_VMAX, vmax_reg);
    }

    return true;
}

float TMC5160::readPosition()
{
    const int32_t pos = tmc5160_readRegister(icID_, TMC5160_XACTUAL);
    return static_cast<float>(static_cast<double>(pos) * kRadiansPerMicrostep);
}

float TMC5160::readSpeed()
{
    const uint32_t raw = tmc5160_readRegister(icID_, TMC5160_VACTUAL);
    double microsteps_sec = Vactual_To_MicrostepsPerSec(raw);
    return static_cast<float>(microsteps_sec * kRadiansPerMicrostep);
}

bool TMC5160::checkComms(const char* label)
{
    const char* tag = label ? label : "TMC5160";
    const uint32_t gconf = tmc5160_readRegister(icID_, TMC5160_GCONF);
    const uint32_t inpOut = tmc5160_readRegister(icID_, TMC5160_INP_OUT);
    const uint32_t version = (inpOut & TMC5160_VERSION_MASK) >> TMC5160_VERSION_SHIFT;

    // Test de lectura/escritura en registro VMAX
    const uint32_t originalVmax = tmc5160_readRegister(icID_, TMC5160_VMAX);
    const uint32_t testVmax = 0x000A000; 
    tmc5160_writeRegister(icID_, TMC5160_VMAX, testVmax);
    const uint32_t echoed = tmc5160_readRegister(icID_, TMC5160_VMAX);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, originalVmax);

    const bool versionOk = (version == 0x21 || version == 0x30);
    const bool vmaxEchoOk = (echoed == testVmax);

    std::printf("[%s] GCONF=0x%08X | INP_OUT=0x%08X (version 0x%02X) | VMAX echo %s\n",
                tag, gconf, inpOut, version, vmaxEchoOk ? "OK" : "FAIL");
    
    return versionOk && vmaxEchoOk;
}

void TMC5160::shutdown()
{
    // Detiene el motor inmediatamente y deshabilita la etapa de potencia
    tmc5160_writeRegister(icID_, TMC5160_RAMPMODE, TMC5160_MODE_HOLD);
    tmc5160_writeRegister(icID_, TMC5160_VMAX, 0);
    enableDriver(false);
    std::cout << "TMC5160 detenido (CS=" << cs_pin_ << ", EN=" << en_pin_ << ")\n";
}

// ==========================================
// Puentes (Trampolines) para la TMC-API en C
// ==========================================
extern "C" void tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length)
{
    unsigned int csPin = 0;
    if (tmc::csPinFor(icID, csPin)) {
        SPIBus::transfer(data, length, csPin);
    }
}

extern "C" TMC5160BusType tmc5160_getBusType(uint16_t)
{
    return TMC5160BusType::IC_BUS_SPI;
}

// Funciones UART no utilizadas en modo SPI
extern "C" bool tmc5160_readWriteUART(uint16_t, uint8_t*, size_t, size_t) { return false; }
extern "C" uint8_t tmc5160_getNodeAddress(uint16_t) { return 0; }