#ifndef SEN_ADXL_HPP
#define SEN_ADXL_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "util.hpp"

class ADXL
{
public:
	ADXL(struct device const *i2c, uint8_t addr);
	~ADXL();

	int8_t setup();
	int8_t getMeasurement(uint8_t *data, uint8_t *cnt);

private:
	struct device const *_i2c;
	uint8_t _addr;
};

#endif