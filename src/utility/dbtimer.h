//
// Simple Arduino us timer for latency investigations
//

#ifndef DBTIMER_H
#define DBTIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif 

// register start time
uint32_t dbtimer_tic(uint8_t itimer);

// register end time, return tiem since tic and accumulate 
uint32_t dbtimer_toc(uint8_t itimer);

// return number of microseconds elasped
uint32_t dbtimer_elapsed(uint8_t itimer);

// return number of times tic() was called 
uint32_t dbtimer_calls(uint8_t itimer);

// clear timer 
void dbtimer_clear(uint8_t itimer);

#ifdef __cplusplus
}
#endif 

#endif 