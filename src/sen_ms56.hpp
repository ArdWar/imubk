#ifndef SEN_MS56_HPP
#define SEN_MS56_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "util.hpp"

#define MS56_CONV_PRES	0x48
#define MS56_CONV_TEMP	0x58

class MS56
{
public:
	MS56(struct device const *i2c, uint8_t addr);
	~MS56();

	int8_t setup();
	int8_t getMeasurement(uint8_t *data, uint8_t *cnt);

private:
	const uint8_t _addr;
	struct device const *_i2c;
	uint16_t promcvt(uint8_t *data);
	uint32_t adccvt(uint8_t *data);

	uint8_t _cmd = 0x48;
	uint8_t _n = 0;
	uint16_t _const[7] = {0};
	int64_t _temp;
	int64_t _pres;
	int64_t _dT;
};

#endif