#include "sen_adxl.hpp"

ADXL::ADXL(struct device const *i2c, uint8_t addr)
{
    _i2c = i2c;
    _addr = addr;
}

ADXL::~ADXL()
{
}

int8_t ADXL::setup()
{
    if (_i2c == NULL)
    {
        return -1;
    }

    int8_t ret = 0;

    uint8_t txbuf = 0;
    uint8_t devid[3] = {0};
    ret += i2c_write_read(_i2c, _addr, &txbuf, 1, &devid, 1);

    uint8_t txbuf1[] = {0x38, 0b10011111};
    ret += i2c_write(_i2c, txbuf1, 2, _addr);

    uint8_t txbuf2[] = {0x2D, 0b00001000};
    ret += i2c_write(_i2c, txbuf2, 2, _addr);

    // printk("ADXL %02X %02X %02X\n", devid[0], devid[1], devid[2]);
    return ret;
}

int8_t ADXL::getMeasurement(uint8_t *data, uint8_t *cnt)
{
    if (data == NULL or _i2c == NULL or cnt == 0)
    {
        return -1;
    }

    uint8_t txdata = 0x39;
    uint8_t bufcnt = 0;
    int8_t ret = 0;

    ret += i2c_write_read(_i2c, _addr, &txdata, 1, &bufcnt, 1);

    bufcnt = bufcnt > *cnt ? *cnt : bufcnt;

    *cnt = 0;

    for (; bufcnt > 0; bufcnt--)
    {
        txdata = 0x32;
        ret += i2c_write_read(_i2c, _addr, &txdata, 1, data, 6);
        (*cnt)++;
    }

    return ret;
}