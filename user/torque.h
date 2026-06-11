#ifndef __TORQUE_H
#define __TORQUE_H

#include <stdint.h>

#include "ds402_state.h"

/*
 * torque.h
 *
 * 本模块是 CANopen slave 侧的 Profile Torque 独立控制入口。
 *
 * 边界说明：
 * - 对象字典文件和 CanFestival 协议栈保持原状，不在这里重新定义对象。
 * - 0x6071/0x6074/0x6077 对主站仍然表现为“额定转矩千分比”的有符号值。
 * - 最终下发给 G4/control 的 mode=4 target_value 仍然是转矩千分比，不是电流。
 */

/*
 * Torque_Init
 *
 * 函数作用：
 * 初始化转矩模式的 1 ms 周期参数和内部斜坡状态。
 *
 * 参数：
 * cycle_us : 上层应用节拍，单位 us。当前由 TIM3 提供，典型值 1000。
 */
void Torque_Init(uint32_t cycle_us);

/*
 * Torque_Reset
 *
 * 函数作用：
 * 离开转矩模式或失能时复位转矩斜坡状态，避免下次进入转矩模式继承旧需求值。
 */
void Torque_Reset(void);

/*
 * Torque_FillDs402Input
 *
 * 函数作用：
 * 给 main.c 统一 DS402 调度提供转矩模式下的故障源和停止完成条件。
 *
 * 参数：
 * input : 已由调用者分配的 DS402_Input，函数内部会填满所有字段。
 */
void Torque_FillDs402Input(DS402_Input *input);

/*
 * Torque_FillDs402StatusBits
 *
 * 函数作用：
 * 给统一状态字生成器补充转矩模式 bit10 等模式相关位。
 *
 * 参数：
 * bits : 已经由 Ds402_InitStatusBits() 初始化的状态位结构。
 */
void Torque_FillDs402StatusBits(DS402_StatusBits *bits);

/*
 * Torque_OnDs402FaultReset
 *
 * 函数作用：
 * DS402 接受 fault reset 上升沿后，清除转矩模式本地软状态。
 */
void Torque_OnDs402FaultReset(void);

/*
 * Torque_Run
 *
 * 函数作用：
 * Profile Torque 模式的 1 ms 周期任务。
 *
 * 职责：
 * - 读取 0x6040/0x6060/0x6071/0x6072/0x6087/0x60E0/0x60E1。
 * - 解释正负转矩命令、执行限幅和转矩斜坡。
 * - 更新 0x6074，并提供 bit10 给统一 DS402 状态字生成器。
 * - 以 mode=4 周期下发 G4 私有命令帧。
 */
void Torque_Run(void);

/*
 * Torque_OnG4Feedback
 *
 * 函数作用：
 * 处理 G4/control 返回的反馈帧，并把转矩模式相关反馈回写到对象字典。
 *
 * 参数：
 * mode       : G4 回显的当前命令模式。
 * act_torque : G4 折算后的实际转矩，单位为额定转矩千分比。
 * act_main   : G4 主反馈量；转矩模式下解释为实际速度 rpm。
 */
void Torque_OnG4Feedback(uint8_t mode, int16_t act_torque, int32_t act_main);

#endif
