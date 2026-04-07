#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f10x.h"      // STM32F10x标准外设库
#include "TestSlave.h"      // 包含CANopen从站定义

/* 电机控制模式定义 */
typedef enum {
    MOTOR_MODE_PROFILE_POSITION = 1,  // 轮廓位置模式
    MOTOR_MODE_PROFILE_VELOCITY = 3,  // 轮廓速度模式
    MOTOR_MODE_PROFILE_TORQUE = 4,    // 轮廓转矩模式
    MOTOR_MODE_HOMING = 6             // 回零模式
} MotorMode;

/* 电机状态定义 */
typedef enum {
    MOTOR_STATE_NOT_READY_TO_SWITCH_ON = 0,
    MOTOR_STATE_SWITCH_ON_DISABLED = 1,
    MOTOR_STATE_READY_TO_SWITCH_ON = 2,
    MOTOR_STATE_SWITCHED_ON = 3,
    MOTOR_STATE_OPERATION_ENABLED = 4,
    MOTOR_STATE_QUICK_STOP_ACTIVE = 5,
    MOTOR_STATE_FAULT_REACTION_ACTIVE = 6,
    MOTOR_STATE_FAULT = 7
} MotorState;

/* 电机控制结构体 */
typedef struct {
    MotorMode mode;            // 当前运行模式
    MotorState state;          // 当前状态
    uint16_t controlWord;      // 控制字
    uint16_t statusWord;       // 状态字
    int32_t targetPosition;    // 目标位置
    int32_t actualPosition;    // 实际位置
    int32_t targetVelocity;    // 目标速度
    int32_t actualVelocity;    // 实际速度
    int16_t targetTorque;      // 目标转矩
    int16_t actualTorque;      // 实际转矩
    uint8_t operationMode;     // 操作模式
    uint8_t operationModeDisplay; // 操作模式显示
    uint32_t profileVelocity;  // 轮廓速度
    uint32_t profileAcceleration; // 轮廓加速度
    uint32_t profileDeceleration; // 轮廓减速度
    uint32_t maxMotorSpeed;    // 最大电机速度
    uint8_t homingMethod;      // 回零方法
    uint32_t homingSpeed1;     // 回零速度1
    uint32_t homingSpeed2;     // 回零速度2
    uint32_t homingAcceleration; // 回零加速度
    
    // 硬件相关参数
    GPIO_TypeDef* dirPort;     // 方向控制端口
    uint16_t dirPin;           // 方向控制引脚
    GPIO_TypeDef* pulsePort;   // 脉冲控制端口
    uint16_t pulsePin;         // 脉冲控制引脚
    GPIO_TypeDef* enablePort;  // 使能控制端口
    uint16_t enablePin;        // 使能控制引脚
    TIM_TypeDef* pwmTimer;     // PWM定时器
    uint16_t pwmChannel;       // PWM通道
} MotorControl;

/* 函数声明 */
void MotorControl_Init(MotorControl* motor);
void MotorControl_Update(MotorControl* motor);
void MotorControl_ProcessControlWord(MotorControl* motor);
void MotorControl_UpdateStatusWord(MotorControl* motor);
void MotorControl_SetOperationMode(MotorControl* motor, uint8_t mode);
void MotorControl_SetTargetPosition(MotorControl* motor, int32_t position);
void MotorControl_SetTargetVelocity(MotorControl* motor, int32_t velocity);
void MotorControl_SetTargetTorque(MotorControl* motor, int16_t torque);
void MotorControl_StartHoming(MotorControl* motor, uint8_t method);
void MotorControl_Reset(MotorControl* motor);
void MotorControl_FaultReset(MotorControl* motor);
void MotorControl_Enable(MotorControl* motor);
void MotorControl_Disable(MotorControl* motor);
void MotorControl_SetDirection(MotorControl* motor, uint8_t direction);
void MotorControl_SetPWM(MotorControl* motor, uint16_t pwmValue);

#endif /* __MOTOR_CONTROL_H */
