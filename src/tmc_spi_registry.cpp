#include "tmc_spi_registry.hpp"

#include <atomic>
#include <mutex>
#include <unordered_map>

namespace {

std::atomic<uint16_t> nextIcID{0};

class CSPinRegistry {
public:
    void registerPin(uint16_t icID, unsigned int pin)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        csPins_[icID] = pin;
    }

    bool pinFor(uint16_t icID, unsigned int& pin)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = csPins_.find(icID);
        if (it == csPins_.end())
            return false;
        pin = it->second;
        return true;
    }

private:
    std::mutex mtx_;
    std::unordered_map<uint16_t, unsigned int> csPins_;
};

CSPinRegistry& registry()
{
    static CSPinRegistry instance;
    return instance;
}

} // namespace

namespace tmc {

uint16_t allocateIcID()
{
    return nextIcID++;
}

void registerCSPin(uint16_t icID, unsigned int csPin)
{
    registry().registerPin(icID, csPin);
}

bool csPinFor(uint16_t icID, unsigned int& pin)
{
    return registry().pinFor(icID, pin);
}

} // namespace tmc
