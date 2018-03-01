#include "dbtimer.h"
#include <string.h>
#include <Arduino.h>

#define NTIMER 8

uint32_t g_dbt_begin[NTIMER];
uint32_t g_dbt_elapsed[NTIMER];
uint32_t g_dbt_ncall[NTIMER];

uint32_t dbtimer_tic(uint8_t itimer) 
{
    itimer = (itimer < NTIMER) ? itimer : 0;
    uint32_t now = micros();
    g_dbt_begin[itimer] = now;
    g_dbt_ncall[itimer]++;
    return now;
}

uint32_t dbtimer_toc(uint8_t itimer)
{   
    itimer = (itimer < NTIMER) ? itimer : 0;
    uint32_t dt = micros() - g_dbt_begin[itimer];
    g_dbt_elapsed[itimer] += dt;
    return dt;
}

uint32_t dbtimer_elapsed(uint8_t itimer)
{
    return g_dbt_elapsed[itimer];
}

uint32_t dbtimer_calls(uint8_t itimer)
{
    return g_dbt_ncall[itimer];
}

void dbtimer_clear(uint8_t itimer) 
{   
    if (itimer < NTIMER) {
        g_dbt_begin[itimer] = 0;
        g_dbt_elapsed[itimer] = 0;
        g_dbt_ncall[itimer] = 0;    
    } else {
        memset(g_dbt_begin, 0, NTIMER*4);
        memset(g_dbt_elapsed, 0, NTIMER*4);
        memset(g_dbt_ncall, 0, NTIMER*4);
    }
}

