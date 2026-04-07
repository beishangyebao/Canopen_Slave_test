
#ifndef __GENERATOR_H
#define __GENERATOR_H

#include <stdbool.h>
#include <stdint.h>

void Generator_Init(uint32_t cycle_us);
bool Generator_ConsumeTick(void);
void Generator_Run(void);
void Generator_OnG4Feedback(uint8_t mode,  int32_t act_main);

#endif
