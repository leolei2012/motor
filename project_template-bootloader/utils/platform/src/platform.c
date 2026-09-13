#include "platform.h"
#include "stm32g4xx.h"

void delay_nop(uint16_t n)
{
    while (n > 0u)
    {
        __NOP();
        n--;
    }
}


