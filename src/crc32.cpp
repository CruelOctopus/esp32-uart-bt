#include <stdint.h>
#include "crc32.h"

void crc32_make_table(uint32_t *Crc32Lookup,uint32_t Polynomial)
{
    for (unsigned int i = 0; i <= 0xFF; i++)
    {
        uint32_t crc = i;
        for (unsigned int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (-int(crc & 1) & Polynomial);
        Crc32Lookup[i] = crc;
    }
}

uint32_t crc32(const void *data, uint32_t length, uint32_t previousCrc32=0)
{
    uint32_t crc = ~previousCrc32;
    unsigned char *current = (unsigned char *)data;
    while (length--)
        crc = (crc >> 8) ^ crc32_table[(crc & 0xFF) ^ *current++];
    return ~crc;
}