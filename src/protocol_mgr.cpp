#include "protocol_mgr.hpp"

PROTOCOL::PROTOCOL(uint8_t *data, uint8_t id, uint8_t count, uint8_t pldlen)
{
    _data = data;
    _id = id;
    _pldlen = pldlen;
    _count = count;

    memset(_data, 0, _pldlen + 4);

    _data[0] = 0xA5;
    _data[1] = _id << 6 | (_pldlen & 0x3F);
    _data[2] = _count;
}

PROTOCOL::~PROTOCOL()
{
}

int8_t PROTOCOL::finish()
{
    return calcCRC();
}

uint8_t PROTOCOL::head(uint16_t data)
{
    return data & 0xFF;
}

uint8_t PROTOCOL::half(uint16_t data1, uint16_t data2)
{
    return (data2 & 0x0F) << 4 | (data1 >> 8 & 0x0F);
}

uint8_t PROTOCOL::tail(uint16_t data)
{
    return data >> 4 & 0xFF;
}

int8_t PROTOCOL::calcCRC()
{
    _data[3 + _pldlen] = crc8(&_data[1], _pldlen + 2, 0x07, 0x00, false);
    return 0;
}

PROTOCOL_IMU::PROTOCOL_IMU(uint8_t *data, uint8_t id, uint8_t count) : PROTOCOL(data, id, count, 48)
{
}

PROTOCOL_IMU::~PROTOCOL_IMU()
{
}

int8_t PROTOCOL_IMU::putADXLData(ADXLRawData *data)
{
    _data[3] = head(data->accX);             // XX  1234 -> 23
    _data[4] = half(data->accX, data->accY); // XY  1234 -> 40 | 1234 -> 02
    _data[5] = tail(data->accY);             // YY  1234 -> 34
    _data[6] = head(data->accZ);             // ZZ
    _data[7] = half(data->accZ, 0);          // Z0

    return 0;
}

int8_t PROTOCOL_IMU::putLSM6Data(LSM6RawData *data)
{
    _data[7] = _data[7] | half(0, data->accX >> 4);     // 0X
    _data[8] = tail(data->accX >> 4);                   // XX
    _data[9] = head(data->accY >> 4);                   // YY
    _data[10] = half(data->accY >> 4, data->accZ >> 4); // YZ
    _data[11] = tail(data->accZ >> 4);                  // ZZ

    _data[12] = head(data->rotX >> 4);                  // XX  1234 -> 23
    _data[13] = half(data->rotX >> 4, data->rotY >> 4); // XY  1234 -> 40 | 1234 -> 02
    _data[14] = tail(data->rotY >> 4);                  // YY  1234 -> 34
    _data[15] = head(data->rotZ >> 4);                  // ZZ
    _data[16] = half(data->rotZ >> 4, 0);               // Z0
    return 0;
}

int8_t PROTOCOL_IMU::putLIS3Data(LIS3RawData *data)
{
    _data[16] = _data[16] | half(0, data->magX >> 4);   // 0X
    _data[17] = tail(data->magX >> 4);                  // XX
    _data[18] = head(data->magY >> 4);                  // YY
    _data[19] = half(data->magY >> 4, data->magZ >> 4); // YZ
    _data[20] = tail(data->magZ >> 4);                  // ZZ
    return 0;
}

int8_t PROTOCOL_IMU::putMTI3Data(MTI3RawData *data)
{
    printk("A%d\n", data->accX >> 20);
    _data[21] = head(data->accX >> 7);                    // XX  1234 -> 23    MTI default is 12.20
    _data[22] = half(data->accX >> 7, data->accY >> 7);   // XY  1234 -> 40 | 1234 -> 02
    _data[23] = tail(data->accY >> 7);                    // YY  1234 -> 34
    _data[24] = head(data->accZ >> 7);                    // ZZ
    _data[25] = half(data->accZ >> 7, data->rotX >> 16);  // ZX
    _data[26] = tail(data->rotX >> 16);                   // XX
    _data[27] = head(data->rotY >> 16);                   // YY
    _data[28] = half(data->rotY >> 16, data->rotZ >> 16); // YZ
    _data[29] = tail(data->rotZ >> 16);                   // ZZ
    _data[30] = head(data->magX >> 7);                    // XX  1234 -> 23
    _data[31] = half(data->magX >> 7, data->magY >> 7);   // XY  1234 -> 40 | 1234 -> 02
    _data[32] = tail(data->magY >> 7);                    // YY  1234 -> 34
    _data[33] = head(data->magZ >> 7);                    // ZZ
    _data[34] = half(data->magZ >> 7, data->pitch >> 17); // ZX
    _data[35] = tail(data->pitch >> 17);                  // XX
    _data[36] = head(data->roll >> 17);                   // YY
    _data[37] = half(data->roll >> 17, data->yaw >> 17);  // YZ
    _data[38] = tail(data->yaw >> 17);                    // ZZ
    _data[39] = head(data->dvX >> 16);                    // XX  1234 -> 23
    _data[40] = half(data->dvX >> 16, data->dvY >> 16);   // XY  1234 -> 40 | 1234 -> 02
    _data[41] = tail(data->dvY >> 16);                    // YY  1234 -> 34
    _data[42] = head(data->dvZ >> 16);                    // ZZ
    _data[43] = half(data->dvZ >> 16, data->dqA >> 16);   // ZX
    _data[44] = tail(data->dqA >> 16);                    // XX
    _data[45] = head(data->dqB >> 16);                    // YY
    _data[46] = half(data->dqB >> 16, data->dqC >> 16);   // YZ
    _data[47] = tail(data->dqC >> 16);                    // ZZ
    _data[48] = head(data->dqD >> 16);                    // ZZ
    _data[49] = half(data->dqD >> 16, data->sTime >> 0);  // ZZ
    _data[50] = tail(data->sTime >> 0);                   // ZZ

    return 0;
}