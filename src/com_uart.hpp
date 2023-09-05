#ifndef DATCOM_HPP
#define DATCOM_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include "util.hpp"

#define COM_RINGBUFFER_LEN 8

typedef struct datcom_payload
{
	uint8_t len;
	uint8_t *pld;
} datcom_payload;

class DATCOM
{
public:
	DATCOM(struct device const *uart);
	~DATCOM();

	int8_t setup();

	int8_t putPayload(datcom_payload *pld);
	int8_t txFinish();
	int8_t txAbort();
private:
	int8_t send();

	struct device const *_uart;
	datcom_payload *_pld[COM_RINGBUFFER_LEN];
	uint8_t _uartTxBusy = 0;
	uint8_t _pldHead = 0;
	uint8_t _pldTail = 0;
};

#endif