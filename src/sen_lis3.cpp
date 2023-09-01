#include "sen_lis3.hpp"

LIS3::LIS3(struct device const *i2c, uint8_t addr) : _i2c(i2c), _addr(addr)
{
}

LIS3::~LIS3()
{
}

int8_t LIS3::setup()
{
	uint8_t devid[2] = {0x0F}; // WHO
	i2c_write_read(_i2c, _addr, &devid[0], 1, &devid[0], 1);

	uint8_t txbuf[] = {0x20, 0b01111110, 0b00000000, 0b00000000, 0b00001100, 0b01000000}; // TNE 155hz 4 gauss
	devid[1] = i2c_write(_i2c, txbuf, 6, _addr);										  // REBOOT

	return 0;
}

int8_t LIS3::getMeasurement(uint8_t *data, uint8_t *cnt)
{
	uint8_t txbuf = 0x27;
	uint8_t rxbuf;
	i2c_write_read(_i2c, _addr, &txbuf, 1, &rxbuf, 1);
	if (rxbuf & 0b00001000)
	{
		txbuf = 0x28;
		i2c_write_read(_i2c, _addr, &txbuf, 1, data, 6);
		*cnt = 1;
	}
	return 0;
}