#include "sen_ms56.hpp"

MS56::MS56(struct device const *i2c, uint8_t addr)
{
	_i2c = i2c;
	_addr = addr;
}

MS56::~MS56()
{
}

int8_t MS56::setup()
{
	uint8_t reset = 0b00011110;
	i2c_write(_i2c, &reset, 1, _addr);
	k_msleep(4);
	uint8_t prom = 0xA0;
	uint8_t buf[16];
	i2c_write_read(_i2c, _addr, &prom, 1, buf, 16);

	for (int8_t i = 1; i < 7; i++)
	{
		_const[i] = promcvt(&buf[(i - 1) * 2]);
	}

	return 0;
}

int8_t MS56::getMeasurement(uint8_t *data, uint8_t *cnt)
{
	uint8_t read = 0;
	uint8_t buf[3];

	i2c_write_read(_i2c, _addr, &read, 1, buf, 3);

	if (_cmd == 0x48)
	{
		_dT = adccvt(buf) - ((uint32_t)_const[5] << 8);
		_temp = 2000 + (_dT * _const[6]) >> 23;
		_cmd = 0x58;
	}
	else
	{
		int64_t ofs = (uint32_t)_const[2] << 16 + (_const[4] * _dT) >> 7;
		int64_t sens = (uint32_t)_const[1] << 15 + (_const[3] * _dT) >> 8;
		_pres = ((adccvt(buf) * sens) >> 21 - ofs) >> 15;
		_cmd = 0x48;
	}

	data[0] = _pres;
	data[1] = _temp;
	i2c_write(_i2c, &_cmd, 1, _addr);

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