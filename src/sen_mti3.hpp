#ifndef SEN_MTI3_HPP
#define SEN_MTI3_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include "util.hpp"

#define XSENS_CONTROL_PIPE 0x03 // Use this opcode for sending (configuration) commands to the MTi
#define XSENS_STATUS_PIPE 0x04  // Use this opcode for reading the status of the notification/measurement pipes
#define XSENS_NOTIF_PIPE 0x05   // Use this opcode for reading a notification message
#define XSENS_MEAS_PIPE 0x06    // Use this opcode for reading measurement data (MTData2 message)

#define XSENS_REQDID 0x00
#define XSENS_DEVICEID 0x01
#define XSENS_REQCONFIGURATION 0x0C
#define XSENS_CONFIGURATION 0x0D
#define XSENS_GOTOMEASUREMENT 0x10
#define XSENS_GOTOMEASUREMENTACK 0x11
#define XSENS_REQFWREV 0x12
#define XSENS_FIRMWAREREV 0x13
#define XSENS_REQBAUDRATE 0x18
#define XSENS_SETBAUDRATE 0x18
#define XSENS_REQPRODUCTCODE 0x1C
#define XSENS_PRODUCTCODE 0x1D
#define XSENS_REQHARDWAREVERSION 0x1E
#define XSENS_HARDWAREVERSION 0x1F
#define XSENS_GOTOCONFIG 0x30
#define XSENS_GOTOCONFIGACK 0x31
#define XSENS_MTDATA2 0x36
#define XSENS_WAKEUP 0x3E
#define XSENS_RESET 0x40
#define XSENS_ERROR 0x42
#define XSENS_WARNING 0x43
#define XSENS_REQFILTERPROFILE 0x64
#define XSENS_SETFILTERPROFILE 0x64
#define XSENS_REQFILTERPROFILEACK 0x65
#define XSENS_REQOUTPUTCONFIGURATION 0xC0
#define XSENS_SETOUTPUTCONFIGURATION 0xC0
#define XSENS_OUTPUTCONFIGURATION 0xC1

#define XDI_TEMPERATURE 0x0810
#define XDI_PACKETCOUNTER 0x1020
#define XDI_SAMPLETIMEFINE 0x1060
#define XDI_EULERANGLES 0x2030
#define XDI_ACCELERATION 0x4020
#define XDI_DELTAV 0x4010
#define XDI_RATEOFTURN 0x8020
#define XDI_DELTAQ 0x8030
#define XDI_MAGNETICFIELD 0xC020
#define XDI_STATUSWORD 0xE020

#define XDI_ENU 0x00
#define XDI_NED 0x04
#define XDI_NWU 0x08

#define XDI_F32 0x00
#define XDI_I32 0x01
#define XDI_I48 0x02
#define XDI_F64 0x03

class MTI3
{
public:
	MTI3(struct device const *spi, struct spi_config cfg);
	~MTI3();

	int8_t getStatus(uint16_t *notifCount, uint16_t *measCount);
	int8_t getNotification(uint8_t *data, uint16_t notifCount);
	int8_t getMeasurement(uint8_t *data, uint16_t measCount);
	int8_t getConfig(uint8_t *data, uint8_t length);
	int8_t setConfig(uint8_t *data, uint8_t length);
	int8_t swapEndian(uint16_t *data, uint8_t len);

private:
	uint8_t calcChecksum(uint8_t *pld, uint8_t len);

	struct device const *_spi;
	struct spi_config _cfg;

	uint8_t _spi_txbuf[128];
	uint8_t _spi_rxbuf[128];

	struct spi_buf _tx_bufs[1];
	struct spi_buf _rx_bufs[1];

	struct spi_buf_set _spi_txset;
	struct spi_buf_set _spi_rxset;
};

#endif