#ifndef SEN_MS56_HPP
#define SEN_MS56_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "util.hpp"

class MS56 {
public:
    MS56(struct device const *i2c, uint8_t addr);
    ~MS56();

    int8_t setup();
    int8_t getMeasurement(uint8_t *data, uint8_t *cnt);
private:
    uint16_t promcvt(uint8_t *data);
    uint32_t adccvt(uint8_t *data);
    struct device const *_i2c;
    uint8_t _addr;

    uint8_t _cmd = 0x48;
    // uint8_t _d1[3] = {0};
    // uint8_t _d2[3] = {0};
    uint16_t _const[6] = {0};
    int64_t _temp;
    int64_t _pres;
    int64_t _dT;
};

#endif