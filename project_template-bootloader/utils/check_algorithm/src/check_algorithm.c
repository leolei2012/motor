#include "check_algorithm.h"



#if CHECKSUM_ENABLE
uint8_t checksum_uint8_t(const uint8_t *data_addr, uint8_t len, uint8_t *checksum_res)
{
    uint8_t sum = 0;
    size_t i;

    for (i = 0; i < len; i++)
    {
        sum += data_addr[i];
    }

    if (checksum_res != NULL)
    {
        *checksum_res = sum;
    }

    return sum;
}
#endif




#if CRC16_ENABLE
uint16_t check_crc16(const uint8_t *p_data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    uint8_t j;

    for (i = 0; i < len; i++)
    {
        crc ^= p_data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
#endif


#if CRC32_ENABLE

#define CRC32_POLY              (0xEDB88320U)

uint32_t check_crc32(const uint8_t *p_data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;

    while (len != 0U)
    {
        uint8_t bit;

        crc ^= *p_data;
        p_data++;
        len--;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ CRC32_POLY;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return ~crc;
}
#endif











