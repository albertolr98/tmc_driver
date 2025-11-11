#include "spi_bus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <iomanip>

int SPIBus::fd_ = -1;
std::mutex SPIBus::mtx_;
bool SPIBus::logEnabled_ = false;

bool SPIBus::init(const char* device, uint32_t speed)
{
    fd_ = open(device, O_RDWR);
    if (fd_ < 0) {
        perror("open spi");
        return false;
    }

    uint8_t mode = SPI_MODE_3;
    uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return false;

    return true;
}

void SPIBus::enableLogging(bool enable)
{
    logEnabled_ = enable;
}

void SPIBus::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SPIBus::transfer(uint8_t* data, size_t len, unsigned int cs_pin)
{
    if (fd_ < 0 || !data || len == 0) return false;

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) return false;
    gpiod_line* cs = gpiod_chip_get_line(chip, cs_pin);
    if (!cs) { gpiod_chip_close(chip); return false; }
    if (gpiod_line_request_output(cs, "spi_cs", 1) < 0) {
        gpiod_line_release(cs);
        gpiod_chip_close(chip);
        return false;
    }

    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(data);
    tr.rx_buf = reinterpret_cast<unsigned long>(data);
    tr.len = len;

    std::lock_guard<std::mutex> lock(mtx_);
    if (logEnabled_) {
        std::cerr << "[SPI] CS=" << cs_pin << " tx:";
        for (size_t i = 0; i < len; ++i) {
            std::cerr << ' ' << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]);
        }
        std::cerr << std::dec << std::endl;
    }
    gpiod_line_set_value(cs, 0);
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    gpiod_line_set_value(cs, 1);
    if (logEnabled_) {
        std::cerr << "[SPI] CS=" << cs_pin << " rx:";
        for (size_t i = 0; i < len; ++i) {
            std::cerr << ' ' << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]);
        }
        std::cerr << std::dec << " (ret=" << ret << ')' << std::endl;
    }

    gpiod_line_release(cs);
    gpiod_chip_close(chip);
    return true;
}
