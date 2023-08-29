#include "sen_lsm6.hpp"

LSM6::LSM6(struct device const *i2c, uint8_t addr)
{
	_i2c = i2c;
	_addr = addr;
}

LSM6::~LSM6()
{
}

int8_t LSM6::setup()
{
	uint8_t devid[5] = {0x0F}; // WHO
	i2c_write_read(_i2c, _addr, &devid[0], 1, &devid[0], 1);

	uint8_t txbuf[] = {0x12, 0b10000000};
	devid[1] = i2c_write(_i2c, txbuf, 2, _addr); // REBOOT
	k_msleep(40);

	uint8_t txbuf2[] = {0x12, 0b00010100}; // LOOP ADDR
	devid[2] = i2c_write(_i2c, txbuf2, 2, _addr);

	uint8_t txbuf3[] = {0x10, 0b01000100, 0b01001101}; // ODR
	devid[3] = i2c_write(_i2c, txbuf3, 3, _addr);

	uint8_t txbuf4[] = {0x09, 0b01000100, 0b00000110}; // FIFO
	devid[4] = i2c_write(_i2c, txbuf4, 3, _addr);
	// printk("LSM6 %02X %02X %02X %02X %02X\n", devid[0], devid[1], devid[2], devid[3], devid[4]);

	return 0;
}

int8_t LSM6::getMeasurement(uint8_t *data, uint8_t *cnt)
{
	*cnt = 0;
	uint8_t txbuf = 0x22;
	i2c_write_read(_i2c, _addr, &txbuf, 1, data, 12);
	(*cnt)++;
	
	return 0;
}