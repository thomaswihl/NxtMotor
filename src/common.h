#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

/* The already shifted address, so lsb has to be 0 */
#define TW_CENTER_ADDR 0x20
#define TW_LEFT_ADDR 0x30
#define TW_RIGHT_ADDR 0x32

enum class Command : uint8_t
{
    SetPwm,
    SetSpeed,
    SetPosition,
    Stop,
    PwmOnStandby,
    GetPosition,
};

#endif // COMMON_H
