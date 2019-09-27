/******************************************************************************
        team lights
        File: defines.h
        Author: Ben Olayinka
        Date: 25 Jun 2019
        Brief:
******************************************************************************/
#ifndef TEAMLIGHTS_DEFINES_H
#define TEAMLIGHTS_DEFINES_H

//function macros
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#ifndef max
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b)  (((a) < (b)) ? (a) : (b))
#endif

#endif // TEAMLIGHTS_DEFINES_H