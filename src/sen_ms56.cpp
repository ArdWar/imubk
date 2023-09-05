#include "sen_ms56.hpp"

MS56::MS56(struct device const *i2c, uint8_t addr) : _addr(addr), _i2c(i2c)
{
}

MS56::~MS56()
{
}

int8_t MS56::setup()
{
	uint8_t reset = 0b00011110;
	i2c_write(_i2c, &reset, 1, _addr);
	k_msleep(10);
	uint8_t prom;
	uint8_t buf[2];

	for (int p = 0; p < 7; p++)
	{
		prom = 0xA0 + p * 2;
		i2c_write_read(_i2c, _addr, &prom, 1, buf, 2);
		_const[p] = promcvt(buf);
	}

	return 0;
}

int8_t MS56::getMeasurement(uint8_t *data, uint8_t *cnt)
{
	uint8_t read = 0;
	uint8_t buf[3];

	// Theoretically it should work with 10ms round robin rate, but it doesnt...
	if (_n == 0)
	{
		i2c_write_read(_i2c, _addr, &read, 1, buf, 3);
		_dT = adccvt(buf) - ((uint32_t)_const[5] << 8);
		_temp = ((_dT * _const[6]) >> 23) + 2000;

		_cmd = MS56_CONV_PRES;
		i2c_write(_i2c, &_cmd, 1, _addr);
	}
	else if (_n == 2)
	{
		i2c_write_read(_i2c, _addr, &read, 1, buf, 3);
		int64_t ofs = ((uint32_t)_const[2] << 16) + ((_const[4] * _dT) >> 7);
		int64_t sens = ((uint32_t)_const[1] << 15) + ((_const[3] * _dT) >> 8);
		_pres = (((adccvt(buf) * sens) >> 21) - ofs) >> 15;

		_cmd = MS56_CONV_TEMP;
		i2c_write(_i2c, &_cmd, 1, _addr);
	}

	data[0] = (_pres >> 1) & 0xFF;
	data[1] = (_pres >> 9) & 0xFF;
	data[2] = _temp & 0xFF;
	data[3] = (_temp >> 8) & 0xFF;
	_n = (_n + 1) % 4;
	return 0;
}

uint16_t MS56::promcvt(uint8_t *data)
{
	return (uint16_t)data[0] << 8 | data[1];
}

uint32_t MS56::adccvt(uint8_t *data)
{
	return (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2];
}