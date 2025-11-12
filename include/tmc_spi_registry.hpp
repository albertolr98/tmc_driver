#pragma once

#include <cstdint>

namespace tmc {

uint16_t allocateIcID();
void registerCSPin(uint16_t icID, unsigned int csPin);
bool csPinFor(uint16_t icID, unsigned int& pin);

} // namespace tmc
