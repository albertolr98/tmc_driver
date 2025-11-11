#include "tmc5160.hpp"
extern "C" {
    #include "tmc/helpers/Constants.h"
    #include "tmc/ic/TMC5160/TMC5160.h"
    #include "tmc/ic/TMC5160/TMC5160_HW_Abstraction.h"
}
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>

static int g_spi_fd = -1;

static bool spi_open(const char* device, uint32_t speed)
{
    g_spi_fd = open(device, O_RDWR);
    if (g_spi_fd < 0) {
        perror("open SPI");
        return false;
    }

    uint8_t mode = SPI_MODE_3;
    uint8_t bits = 8;

    if (ioctl(g_spi_fd, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(g_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(g_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return false;

    return true;
}

static void spi_close()
{
    if (g_spi_fd >= 0) {
        close(g_spi_fd);
        g_spi_fd = -1;
    }
}

// --- Callbacks C requeridos por TMC-API ---
extern "C" TMC5160BusType tmc5160_getBusType(uint16_t icID)
{
    (void)icID;
    return IC_BUS_SPI;
}

extern "C" void tmc5160_readWriteSPI(uint16_t icID, uint8_t *data, size_t length)
{
    (void)icID;
    if (g_spi_fd < 0 || data == nullptr || length == 0) return;

    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(data);
    tr.rx_buf = reinterpret_cast<unsigned long>(data);
    tr.len = length;

    ioctl(g_spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

extern "C" bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    (void)icID;
    (void)data;
    (void)writeLength;
    (void)readLength;
    return false; // UART no soportado en esta implementación
}

extern "C" uint8_t tmc5160_getNodeAddress(uint16_t icID)
{
    (void)icID;
    return 0;
}

// --- Implementación C++ ---
bool TMC5160::init()
{
    if (!spi_open("/dev/spidev0.0", 1000000)) {
        std::cerr << "Error abriendo SPI\n";
        return false;
    }

    icID_ = 0;
    // Inicialización mínima (en la API, muchos chips no requieren init explícito)
    std::cout << "TMC5160 inicializado correctamente\n";
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
