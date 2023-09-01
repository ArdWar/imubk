#ifndef SEN_LSM6_HPP
#define SEN_LSM6_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "util.hpp"

class LSM6
{
public:
	LSM6(struct device const *i2c, uint8_t addr);
	~LSM6();

	int8_t setup();
	int8_t getMeasurement(uint8_t *data, uint8_t *cnt);

private:
	struct device const *_i2c;
	const uint8_t _addr;
};

#endif