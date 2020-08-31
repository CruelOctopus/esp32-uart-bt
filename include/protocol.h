#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>

#define SYNCHRO 0xaa

typedef struct _FirstFrame{
    uint8_t Synchro[8];
    uint8_t AESIV[16];
    uint8_t RandomData[32];
    uint32_t CRC32;
} FirstFrame;
struct NextFrame{
    uint8_t Data[16];
    uint8_t RandomData[32];
};
typedef struct _protocol
{
    uint8_t Synchro[8];
    uint8_t EncData[48];
    uint8_t CRC32[4];
} protocol;

#endif