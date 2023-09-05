#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include "util.hpp"
#include "data.hpp"
#include <zephyr/sys/crc.h>

class PROTOCOL
{
public:
	PROTOCOL(uint8_t *data, uint8_t id, uint8_t count, uint8_t pldlen);
	~PROTOCOL();

	int8_t finish();

private:
	int8_t calcCRC();

protected:
	uint8_t head(uint16_t data);
	uint8_t half(uint16_t data1, uint16_t data2);
	uint8_t tail(uint16_t data);

	uint8_t *_data;
	uint8_t _id;
	uint8_t _pldlen;
	uint8_t _count;
};

class PROTOCOL_IMU : public PROTOCOL
{
public:
	PROTOCOL_IMU(uint8_t *data, uint8_t id, uint8_t count);
	~PROTOCOL_IMU();

	int8_t putADXLData(ADXLRawData *data);
	int8_t putLSM6Data(LSM6RawData *data);
	int8_t putLIS3Data(LIS3RawData *data);
	int8_t putMTI3Data(MTI3RawData *data);
};

class PROTOCOL_GPS : public PROTOCOL
{
public:
	PROTOCOL_GPS(uint8_t *data, uint8_t id, uint8_t count);
	~PROTOCOL_GPS();

	int8_t putNMEAData(NMEARawData *data);
	int8_t putMS56Data(MS56RawData *data);
};

#endif