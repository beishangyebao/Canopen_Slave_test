#include "g4_motor.h"

/* G4 驱动命令帧封装模块，负责把上层命令打包成 CAN 报文。 */

extern uint8_t canSend(CAN_PORT notused, Message *message);

volatile uint8_t g_g4_fault_status;

#define ID_G4_CMD   0x010u

/* 
 * 参数说明：
 * command：已经构建好的 G4 命令帧结构体
 */
/* 把实时控制命令按 G4 私有协议格式打包后发送出去。 */
void Driver_Send_CommandFrame(const G4_CommandFrame *command)
{
    Message msg;
    uint32_t raw_target;

    /* 该 ID 固定用于“从站 -> G4 驱动”的实时控制帧。 */
    msg.cob_id = ID_G4_CMD;
    msg.rtr = 0;
    msg.len = 8;

    /* 第 0~3 字节放控制类型、模式、控制标志和回零方式。 */
    msg.data[0] = command->cmd_type;
    msg.data[1] = (uint8_t)command->mode;
    msg.data[2] = command->control_flags;
    msg.data[3] = (uint8_t)command->homing_method;
    /*
     * 第 4~7 字节放 32 位目标值，按小端字节序拆分。
     * mode=4 时该字段是转矩千分比，不在 slave 侧换算成电流。
     */
    raw_target = (uint32_t)command->target_value;
    msg.data[4] = (uint8_t)(raw_target & 0xFFu);
    msg.data[5] = (uint8_t)((raw_target >> 8) & 0xFFu);
    msg.data[6] = (uint8_t)((raw_target >> 16) & 0xFFu);
    msg.data[7] = (uint8_t)((raw_target >> 24) & 0xFFu);

    /* 最终通过底层 canSend 发到总线上。 */
    canSend(0, &msg);
}
