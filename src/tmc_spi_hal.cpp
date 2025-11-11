#include "tmc_spi_hal.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

static int spi_fd = -1;

bool spi_init(const char* device, unsigned int speed) {
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) return false;
    uint8_t mode = SPI_MODE_3;
    uint8_t bits = 8;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return true;
}

void spi_close() { if (spi_fd >= 0) close(spi_fd); }

