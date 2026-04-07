#ifndef __G4_MOTOR_H
#define __G4_MOTOR_H

#include "canfestival.h"
#include "stm32f10x.h"

typedef enum {
    CMD_TYPE_DISABLE = 0,
    CMD_TYPE_ENABLE = 1,
    CMD_TYPE_QUICKSTOP = 2,
    CMD_TYPE_HALT = 3
} G4_CmdType;

typedef struct {
    uint8_t cmd_type;
    int8_t mode;
    uint8_t control_flags;
    int8_t homing_method;
    int32_t target_value;
} G4_CommandFrame;

#define G4_FLAG_TRIGGER  0x01u
#define G4_FLAG_RELATIVE 0x02u

extern volatile uint8_t g_g4_fault_status;

void Driver_Send_CommandFrame(const G4_CommandFrame *command);

#endif
