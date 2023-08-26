#include "sen_mti3.hpp"

MTI3::MTI3(struct device const *spi, struct spi_config cfg)
{
	_spi = spi;
	_cfg = cfg;

	_tx_bufs[0] = {.buf = _spi_txbuf};
	_spi_txset = {.buffers = _tx_bufs, .count = 1};

	_rx_bufs[0] = {.buf = _spi_rxbuf};
	_spi_rxset = {.buffers = _rx_bufs, .count = 1};
}

MTI3::~MTI3()
{
}

int8_t MTI3::getStatus(uint16_t *notifCount, uint16_t *measCount)
{
	uint8_t opcode[] = {0x04, 0xFF, 0xFF, 0xFF};
	_tx_bufs[0].len = 4;
	_tx_bufs[0].buf = opcode;
	_rx_bufs[0].len = 8;
	_rx_bufs[0].buf = _spi_rxbuf;
	int8_t ret = spi_transceive(_spi, &_cfg, &_spi_txset, &_spi_rxset);
	*notifCount = (uint16_t)_spi_rxbuf[4] | ((uint16_t)_spi_rxbuf[5] << 8);
	*measCount = (uint16_t)_spi_rxbuf[6] | ((uint16_t)_spi_rxbuf[7] << 8);

	return ret;
}

int8_t MTI3::getNotification(uint8_t *data, uint16_t notifCount)
{
	uint8_t opcode[] = {0x05, 0xFF, 0xFF, 0xFF};
	_tx_bufs[0].len = 4;
	_tx_bufs[0].buf = opcode;
	_rx_bufs[0].len = 4 + notifCount;
	_rx_bufs[0].buf = data;
	return spi_transceive(_spi, &_cfg, &_spi_txset, &_spi_rxset);
}

int8_t MTI3::getMeasurement(uint8_t *data, uint16_t measCount)
{
	uint8_t opcode[] = {0x06, 0xFF, 0xFF, 0xFF};
	_tx_bufs[0].len = 4;
	_tx_bufs[0].buf = opcode;
	_rx_bufs[0].len = 4 + measCount;
	_rx_bufs[0].buf = data;
	return spi_transceive(_spi, &_cfg, &_spi_txset, &_spi_rxset);
}

int8_t MTI3::getConfig(uint8_t *data, uint8_t length)
{
	uint8_t opcode[] = {0x03, 0xFF, 0xFF, 0xFF};
	memcpy(_spi_txbuf, opcode, 4);
	memcpy(&_spi_txbuf[4], data, length);
	_spi_txbuf[4 + length] = calcChecksum(data, length);
	_tx_bufs[0].len = 4 + length + 1;
	_tx_bufs[0].buf = _spi_txbuf;
	return spi_transceive(_spi, &_cfg, &_spi_txset, &_spi_rxset);
}

int8_t MTI3::setConfig(uint8_t *data, uint8_t length)
{
	uint8_t opcode[] = {0x03, 0xFF, 0xFF, 0xFF};
	memcpy(_spi_txbuf, opcode, 4);
	memcpy(&_spi_txbuf[4], data, length);
	_spi_txbuf[4 + length] = calcChecksum(data, length);
	_tx_bufs[0].len = 4 + length + 1;
	_tx_bufs[0].buf = _spi_txbuf;
	return spi_transceive(_spi, &_cfg, &_spi_txset, &_spi_rxset);
}

uint8_t MTI3::calcChecksum(uint8_t *dat, uint8_t len)
{
	uint8_t cks = 1;
	while (len-- > 0)
	{
		cks -= dat[len];
	}
	return cks;
}

int8_t MTI3::swapEndian(uint16_t *data, uint8_t len)
{
	// uint16_t cpy[len];                                                              //Create a copy of the data
	// memcpy(cpy, data, len);
	for (int i = 0; i < len / 2; i++)
	{
		// printk("%02X", data[i]);

		data[i] = data[i] >> 8 | data[i] << 8; // Within each 4-byte (32-bit) float, reverse the order of the individual bytes
	}
	return 0;
}
