#include "tmc5160.hpp"
#include "spi_bus.hpp"

extern "C" {
    #include "tmc/ic/TMC5160/TMC5160.h"
}

#include <array>
#include <cstdio>
#include <utility>

static bool runCommsProbe(TMC5160& driver, const char* label)
{
    const uint16_t id = driver.icID();
    const uint32_t gconf = tmc5160_readRegister(id, TMC5160_GCONF);
    const uint32_t inpOut = tmc5160_readRegister(id, TMC5160_INP_OUT);
    const uint32_t version = (inpOut & TMC5160_VERSION_MASK) >> TMC5160_VERSION_SHIFT;

    const uint32_t originalVmax = tmc5160_readRegister(id, TMC5160_VMAX);
    const uint32_t testVmax = 0x000A000; // pequeño valor de prueba dentro del rango
    tmc5160_writeRegister(id, TMC5160_VMAX, testVmax);
    const uint32_t echoed = tmc5160_readRegister(id, TMC5160_VMAX);
    tmc5160_writeRegister(id, TMC5160_VMAX, originalVmax);

    const bool versionOk = (version == 0x21);
    const bool vmaxEchoOk = (echoed == testVmax);

    std::printf("[%s] GCONF=0x%08X | INP_OUT=0x%08X (version 0x%02X) | VMAX echo %s\n",
                label, gconf, inpOut, version, vmaxEchoOk ? "OK" : "FAIL");
    if (!versionOk) {
        std::printf("  -> Firma esperada 0x21, recibido 0x%02X\n", version);
    }

    return versionOk && vmaxEchoOk;
}

int main()
{
    if (!SPIBus::init("/dev/spidev0.0", 1000000)) {
        std::perror("SPIBus::init");
        return 1;
    }

    // Crear instancias de los 4 drivers con sus pines reales
    TMC5160 drv1(5, 26);   // CS=GPIO5,  EN=GPIO26
    TMC5160 drv2(22, 19);  // CS=GPIO22, EN=GPIO19
    TMC5160 drv3(27, 24);  // CS=GPIO27, EN=GPIO24
    TMC5160 drv4(17, 23);  // CS=GPIO17, EN=GPIO23

    std::array<std::pair<const char*, TMC5160*>, 4> drivers = {{
        {"DRV1", &drv1},
        {"DRV2", &drv2},
        {"DRV3", &drv3},
        {"DRV4", &drv4},
    }};

    for (auto& [label, drv] : drivers) {
        if (!drv->init()) {
            std::printf("[%s] Error al inicializar\n", label);
            continue;
        }
        const bool ok = runCommsProbe(*drv, label);
        std::printf("[%s] Resultado comunicación: %s\n", label, ok ? "OK" : "FALLO");
    }

    SPIBus::close();
    return 0;
}
