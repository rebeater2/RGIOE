//
// Created by linfe on 2023/5/14.
//

#include "comm_crc.h"

#define CRC32_POLY 0x04C11DB7U
#define CRC32_START 0xFFFFFFFFU
#define HARDWARE_CRC  0
uint32_t crc32_checksum(const uint8_t *indata, uint32_t num_bytes) {
#if HARDWARE_CRC == 1
    /*在STM32中通过硬件硬件实现*/
#else
    /*软件校验*/
    uint32_t crc = CRC32_START;
    const uint8_t *p = indata;
    for (int i = 0; i < num_bytes; i++) {
        crc = crc ^ (*p++ << 24U);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & (1LU << 31U)) crc = (crc << 1U) ^ CRC32_POLY;
            else crc = (crc << 1U);
        }
    }
#endif
    return crc;
}