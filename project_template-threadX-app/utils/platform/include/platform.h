#ifndef PLATFORM_H
#define PLATFORM_H


#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

enum on_off_t
{
    OFF = 0,
    ON = 1,
};

enum
{
    NO = 0,
    YES = 1,
};



#endif // PLATFORM_H
