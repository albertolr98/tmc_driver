#pragma once
#include <cstdint>   
#include <cstddef>

bool spi_init(const char* device = "/dev/spidev0.0", unsigned int speed = 1000000);
void spi_close();
void tmc5160_readWriteSPI(uint8_t icID, uint8_t *data, size_t length);
uint8_t tmc5160_getBusType(uint8_t icID);

