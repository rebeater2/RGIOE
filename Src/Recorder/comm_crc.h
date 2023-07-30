//
// Created by linfe on 2023/5/14.
//

#ifndef H750_BSP_COMM_CRC_H
#define H750_BSP_COMM_CRC_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
uint32_t crc32_checksum(const uint8_t *indata, uint32_t num_bytes);
#ifdef __cplusplus
};
#endif

#endif //H750_BSP_COMM_CRC_H
