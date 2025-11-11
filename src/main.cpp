#include <cstdio>       
#include "tmc5160.hpp"

int main() {
    TMC5160 driver;
    if (!driver.init()) return 1;

    driver.setSpeed(0, 120.0);
    float pos = driver.readPosition(0);

    std::printf("Posición: %.2f\n", pos);
    return 0;
}
