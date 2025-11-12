#include "tmc_driver.hpp"
#include "tmc5160.hpp"
#include "spi_bus.hpp"

#include <array>
#include <cstdio>
#include <utility>



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

    std::array<std::pair<const char*, TMCDriver*>, 4> drivers = {{
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
        const bool ok = drv->checkComms(label);
        std::printf("[%s] Resultado comunicación: %s\n", label, ok ? "OK" : "FALLO");
    }

    SPIBus::close();
    return 0;
}
