#ifndef DATA_HPP
#define DATA_HPP

#include <stdint.h>

typedef struct MTI3RawData
{
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t rotX;
	int16_t rotY;
	int16_t rotZ;
	int16_t magX;
	int16_t magY;
	int16_t magZ;
	int16_t dvX;
	int16_t dvY;
	int16_t dvZ;
	int16_t dqX;
	int16_t dqY;
	int16_t dqZ;
	int16_t sTime;
	int16_t counter;
} MTI3RawData;

typedef struct ADXLRawData
{
	uint16_t accX;
	uint16_t accY;
	uint16_t accZ;
} ADXLRawData;

typedef struct LSM6RawData
{
	uint16_t accX;
	uint16_t accY;
	uint16_t accZ;
	uint16_t rotX;
	uint16_t rotY;
	uint16_t rotZ;
} LSM6RAWData;

typedef struct LIS3RawData
{
	uint16_t magX;
	uint16_t magY;
	uint16_t magZ;
} LIS3RAWData;

typedef struct MS56RawData
{
	uint16_t pres;
	uint16_t temp;
} MS56RawData;

#endif