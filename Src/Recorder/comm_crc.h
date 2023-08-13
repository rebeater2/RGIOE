//
// Created by linfe on 2023/5/14.
//

#ifndef H750_BSP_COMM_CRC_H
#define H750_BSP_COMM_CRC_H
#ifdef __cplusplus
extern "C" {
#endif
#ifndef CRC32_ENABLE_TABLE
#define CRC32_ENABLE_TABLE 1
#endif
#include <stdint.h>

#define REVERT_CRC32(crc32) \
    ((((crc32) & 0xff) << 24u) | (((crc32) & 0xff00) << 8u) | (((crc32) & 0xff0000) >> 8u) | ((((crc32) & 0xff000000)>>24)))

uint32_t crc32_checksum(const uint8_t *indata, uint32_t num_bytes);
#ifdef __cplusplus
};
#endif

#endif //H750_BSP_COMM_CRC_H
