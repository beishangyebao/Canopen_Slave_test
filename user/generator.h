
#ifndef __GENERATOR_H
#define __GENERATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ds402_state.h"

void Generator_Init(uint32_t cycle_us);
bool Generator_ConsumeTick(void);
void Generator_FillDs402Input(DS402_Input *input);
void Generator_FillDs402StatusBits(DS402_StatusBits *bits);
void Generator_OnDs402FaultReset(void);
void Generator_Run(void);
void Generator_OnG4Feedback(uint8_t mode, int16_t act_torque, int32_t act_main);

#endif
