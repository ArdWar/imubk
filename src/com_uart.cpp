#include "com_uart.hpp"

DATCOM::DATCOM(struct device const *uart)
{
	_uart = uart;
}

DATCOM::~DATCOM()
{
}

int8_t DATCOM::setup()
{
	return 0;
}

int8_t DATCOM::putPayload(datcom_payload *data)
{
	if ((_pldHead + 1) % COM_RINGBUFFER_LEN == _pldTail)
	{
		return -1;
	}

	_pld[_pldHead] = data;
	_pldHead = (_pldHead + 1) % COM_RINGBUFFER_LEN;

	if (!_uartTxBusy)
	{
		this->send();
	}
	return 0;
}

int8_t DATCOM::send()
{
	if (_pldTail == _pldHead)
	{
		return -1;
	}
	_uartTxBusy = 1;
	return uart_tx(_uart, _pld[_pldTail]->pld, _pld[_pldTail]->len, SYS_FOREVER_US);
}

int8_t DATCOM::txFinish()
{
	delete _pld[_pldTail]->pld;
	delete _pld[_pldTail];

	_pldTail = (_pldTail + 1) % COM_RINGBUFFER_LEN;

	_uartTxBusy = 0;
	if (_pldTail != _pldHead)
	{
		this->send();
	}
	return 0;
}