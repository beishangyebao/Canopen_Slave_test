#ifndef __DS402_STATE_H
#define __DS402_STATE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 函数作用：
 * 描述 CiA402 Power Drive System finite state automaton 的基础状态
 *
 * 说明：
 * - 上电后必须先进入 NOT_READY_TO_SWITCH_ON
 * - 状态机在 main.c 的 1 ms 周期中只更新一次
 */
typedef enum
{
    DS402_STATE_NOT_READY_TO_SWITCH_ON = 0,
    DS402_STATE_SWITCH_ON_DISABLED,
    DS402_STATE_READY_TO_SWITCH_ON,
    DS402_STATE_SWITCHED_ON,
    DS402_STATE_OPERATION_ENABLED,
    DS402_STATE_QUICK_STOP_ACTIVE,
    DS402_STATE_FAULT_REACTION_ACTIVE,
    DS402_STATE_FAULT
} DS402_State;

/*
 * 函数/模块作用：
 * 作为 Ds402_Update() 的周期输入，把 CANopen、G4 链路和运动模块安全条件统一交给状态机
 *
 * 字段说明：
 * controlword         : 0x6040 Controlword
 * startup_ready       : 上层软件初始化是否完成；状态机内部还会额外保证最小 Not ready 驻留时间
 * nmt_operational     : CANopen NMT 是否为 Operational
 * drive_online        : G4 反馈链路是否稳定在线
 * local_fault         : G4 故障、G4 通信超时、跟随误差等任一故障源
 * fault_reset_allowed : 允许清除本地软锁存故障；硬件故障或通信超时未消失时必须为 false
 * fault_reaction_done : 故障反应动作是否完成
 * quick_stop_done     : 急停减速是否完成
 */
typedef struct
{
    uint16_t controlword;
    bool startup_ready;
    bool nmt_operational;
    bool drive_online;
    bool local_fault;
    bool fault_reset_allowed;
    bool fault_reaction_done;
    bool quick_stop_done;
} DS402_Input;

/*
 * 函数/模块作用：
 * 承载模式相关状态位和公共状态位，用于 Ds402_BuildStatusword() 统一生成 0x6041
 *
 * 字段说明：
 * voltage_enabled : bit4，G4 在线且允许驱动电源时置位
 * remote          : bit9，CANopen 远程控制有效时置位
 * target_reached  : bit10，由位置/速度/转矩模块提供
 * setpoint_ack    : bit12，位置模式 new set-point acknowledge
 * following_error : bit13，位置模式跟随误差
 */
typedef struct
{
    bool voltage_enabled;
    bool remote;
    bool target_reached;
    bool setpoint_ack;
    bool following_error;
} DS402_StatusBits;

void Ds402_Init(void);
void Ds402_Update(const DS402_Input *in);
DS402_State Ds402_GetState(void);

void Ds402_InitStatusBits(DS402_StatusBits *bits);
uint16_t Ds402_BuildStatusword(const DS402_StatusBits *bits);
void Ds402_PublishStatusword(const DS402_StatusBits *bits);

bool Ds402_IsOperationEnabled(void);
bool Ds402_IsQuickStopActive(void);
bool Ds402_IsFaultReactionActive(void);
bool Ds402_IsFault(void);
bool Ds402_ConsumeFaultResetAccepted(void);

#endif /* __DS402_STATE_H */
