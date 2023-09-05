#ifndef DATA_HPP
#define DATA_HPP

#include <stdint.h>

typedef struct MTI3RawData
{
	int32_t pitch;
	int32_t roll;
	int32_t yaw;
	int32_t accX;
	int32_t accY;
	int32_t accZ;
	int32_t rotX;
	int32_t rotY;
	int32_t rotZ;
	int32_t magX;
	int32_t magY;
	int32_t magZ;
	int32_t dvX;
	int32_t dvY;
	int32_t dvZ;
	int32_t dqA;
	int32_t dqB;
	int32_t dqC;
	int32_t dqD;
	uint32_t sTime;
	uint32_t counter;
} MTI3RawData;

typedef struct ADXLRawData
{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
} ADXLRawData;

typedef struct LSM6RawData
{
	int16_t rotX;
	int16_t rotY;
	int16_t rotZ;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
} LSM6RAWData;

typedef struct LIS3RawData
{
	int16_t magX;
	int16_t magY;
	int16_t magZ;
} LIS3RAWData;

typedef struct MS56RawData
{
	uint16_t pres;
	int16_t temp1;
	int16_t temp2;
} MS56RawData;

typedef struct NMEARawData
{
	int32_t lat;
	int32_t lon;
	uint16_t alt;
	uint16_t vel;
	uint16_t hdg;
	uint16_t tim;
} NMEARawData;

#endif