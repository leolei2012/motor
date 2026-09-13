#ifndef CHECK_ALGORITHM_H
#define CHECK_ALGORITHM_H

#include <stddef.h>
#include <stdint.h>


#define CHECKSUM_ENABLE    0
#define CRC16_ENABLE       1
#define CRC32_ENABLE       1



#if CHECKSUM_ENABLE
uint8_t checksum_uint8_t(const uint8_t *data_addr, uint8_t len, uint8_t *checksum_res);
#endif

#if CRC16_ENABLE
uint16_t check_crc16(const uint8_t *p_data, uint8_t len);
#endif

#if CRC32_ENABLE
uint32_t check_crc32(const uint8_t *p_data, uint32_t len);
#endif


#endif // CHECK_ALGORITHM_H