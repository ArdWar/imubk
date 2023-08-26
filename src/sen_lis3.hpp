#ifndef SEN_LIS3_HPP
#define SEN_LIS3_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "util.hpp"

class LIS3
{
public:
	LIS3(struct device const *i2c, uint8_t addr);
	~LIS3();

	int8_t setup();
	int8_t getMeasurement(uint8_t *data, uint8_t *cnt);

private:
	struct device const *_i2c;
	uint8_t _addr;
};

#endif