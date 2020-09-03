#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>

#define SYNCHRO 0xaa
enum ProtocolState {SetupConnection,};
static uint8_t FrameSize[1]={20};

typedef struct _FrameSetup{
    uint8_t RandomData[16];
    uint32_t CRC32;
} FrameSetup;
typedef struct _FrameFirst{
    uint8_t AESIV[16];
    uint8_t RandomData[32];
    uint32_t CRC32;
} FrameFirst;
struct NextFrame{
    uint8_t Data[16];
    uint8_t RandomData[32];
    uint8_t CRC32[4];
};
typedef struct _protocol
{
    uint8_t Synchro[8];
    uint8_t EncData[48];
    uint8_t CRC32[4];
} protocol;

#endif