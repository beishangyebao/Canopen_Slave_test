#include "motor_control.h"
#include "TestSlave.h"  // 包含CANopen从站定义

/* 电机控制实例 */
MotorControl motor;

/* 初始化电机控制 */
void MotorControl_Init(MotorControl* motor)
{
    // 初始化电机参数
    motor->mode = MOTOR_MODE_PROFILE_POSITION;
    motor->state = MOTOR_STATE_NOT_READY_TO_SWITCH_ON;
    motor->controlWord = 0;
    motor->statusWord = 0;
    motor->targetPosition = 0;
    motor->actualPosition = 0;
    motor->targetVelocity = 0;
    motor->actualVelocity = 0;
    motor->targetTorque = 0;
    motor->actualTorque = 0;
    motor->operationMode = 0;
    motor->operationModeDisplay = 0;
    motor->profileVelocity = 1000;  // 默认速度
    motor->profileAcceleration = 1000;  // 默认加速度
    motor->profileDeceleration = 1000;  // 默认减速度
    motor->maxMotorSpeed = 5000;  // 默认最大速度
    motor->homingMethod = 0;
    motor->homingSpeed1 = 1000;  // 默认回零速度1
    motor->homingSpeed2 = 100;   // 默认回零速度2
    motor->homingAcceleration = 1000;  // 默认回零加速度
    
    // 初始化硬件接口 - 根据您的实际连接调整
    motor->dirPort = GPIOA;
    motor->dirPin = GPIO_Pin_0;
    motor->pulsePort = GPIOA;
    motor->pulsePin = GPIO_Pin_1;
    motor->enablePort = GPIOA;
    motor->enablePin = GPIO_Pin_2;
    motor->pwmTimer = TIM3;
    motor->pwmChannel = 1;  // TIM3_CH1
    
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 方向控制引脚
    GPIO_InitStructure.GPIO_Pin = motor->dirPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(motor->dirPort, &GPIO_InitStructure);
    
    // 脉冲控制引脚
    GPIO_InitStructure.GPIO_Pin = motor->pulsePin;
    GPIO_Init(motor->pulsePort, &GPIO_InitStructure);
    
    // 使能控制引脚
    GPIO_InitStructure.GPIO_Pin = motor->enablePin;
    GPIO_Init(motor->enablePort, &GPIO_InitStructure);
    
    // 初始化PWM定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 定时器基本配置
    TIM_TimeBaseStructure.TIM_Period = 999;  // PWM周期
    TIM_TimeBaseStructure.TIM_Prescaler = 71;  // 分频系数，72MHz/72=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(motor->pwmTimer, &TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // 根据通道配置PWM
    switch(motor->pwmChannel) {
        case 1:
            TIM_OC1Init(motor->pwmTimer, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(motor->pwmTimer, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC2Init(motor->pwmTimer, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(motor->pwmTimer, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC3Init(motor->pwmTimer, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(motor->pwmTimer, TIM_OCPreload_Enable);
            break;
        case 4:
            TIM_OC4Init(motor->pwmTimer, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(motor->pwmTimer, TIM_OCPreload_Enable);
            break;
    }
    
    // 使能定时器预装载寄存器
    TIM_ARRPreloadConfig(motor->pwmTimer, ENABLE);
    
    // 启动定时器
    TIM_Cmd(motor->pwmTimer, ENABLE);
    
    // 初始状态：禁用电机
    MotorControl_Disable(motor);
}

/* 更新电机控制 */
void MotorControl_Update(MotorControl* motor)
{
    // 从CANopen对象字典更新电机参数
    motor->controlWord = TestSlave_obj6040;
    motor->operationMode = TestSlave_obj6060;
    motor->targetPosition = TestSlave_obj607A;
    motor->targetVelocity = TestSlave_obj60FF;
    motor->targetTorque = TestSlave_obj6071;
    motor->profileVelocity = TestSlave_obj6081;
    motor->profileAcceleration = TestSlave_obj6083;
    motor->profileDeceleration = TestSlave_obj6084;
    motor->maxMotorSpeed = TestSlave_obj6080;
    motor->homingMethod = TestSlave_obj6098;
    motor->homingSpeed1 = TestSlave_obj6099_speed1;
    motor->homingSpeed2 = TestSlave_obj6099_speed2;
    motor->homingAcceleration = TestSlave_obj609A;
    
    // 处理控制字
    MotorControl_ProcessControlWord(motor);
    
    // 更新状态字
    MotorControl_UpdateStatusWord(motor);
    
    // 更新CANopen对象字典中的电机状态
    TestSlave_obj6041 = motor->statusWord;
    TestSlave_obj6061 = motor->operationModeDisplay;
    TestSlave_obj6064 = motor->actualPosition;
    TestSlave_obj606C = motor->actualVelocity;
    TestSlave_obj6062 = motor->actualTorque;
    
    // 根据当前模式和目标值控制电机
    if (motor->state == MOTOR_STATE_OPERATION_ENABLED) {
        switch (motor->mode) {
            case MOTOR_MODE_PROFILE_POSITION:
                // 位置控制模式
                if (motor->controlWord & 0x0020) {  // 开始运动位
                    // 计算位置误差
                    int32_t positionError = motor->targetPosition - motor->actualPosition;
                    
                    // 简单的比例控制
                    int16_t pwmValue = (int16_t)(positionError / 10);  // 简单的比例系数
                    
                    // 限制PWM值
                    if (pwmValue > 500) pwmValue = 500;
                    if (pwmValue < -500) pwmValue = -500;
                    
                    // 设置方向和PWM
                    if (pwmValue >= 0) {
                        MotorControl_SetDirection(motor, 1);
                        MotorControl_SetPWM(motor, (uint16_t)pwmValue);
                    } else {
                        MotorControl_SetDirection(motor, 0);
                        MotorControl_SetPWM(motor, (uint16_t)(-pwmValue));
                    }
                    
                    // 更新实际位置（简化模型）
                    if (pwmValue > 10) {
                        motor->actualPosition++;
                    } else if (pwmValue < -10) {
                        motor->actualPosition--;
                    }
                    
                    // 更新实际速度（简化模型）
                    motor->actualVelocity = pwmValue * 10;
                }
                break;
                
            case MOTOR_MODE_PROFILE_VELOCITY:
                // 速度控制模式
                if (motor->controlWord & 0x0020) {  // 开始运动位
                    // 计算速度误差
                    int32_t velocityError = motor->targetVelocity - motor->actualVelocity;
                    
                    // 简单的比例控制
                    int16_t pwmValue = (int16_t)(velocityError / 50);  // 简单的比例系数
                    
                    // 限制PWM值
                    if (pwmValue > 500) pwmValue = 500;
                    if (pwmValue < -500) pwmValue = -500;
                    
                    // 设置方向和PWM
                    if (pwmValue >= 0) {
                        MotorControl_SetDirection(motor, 1);
                        MotorControl_SetPWM(motor, (uint16_t)pwmValue);
                    } else {
                        MotorControl_SetDirection(motor, 0);
                        MotorControl_SetPWM(motor, (uint16_t)(-pwmValue));
                    }
                    
                    // 更新实际速度（简化模型）
                    motor->actualVelocity += velocityError / 100;
                    
                    // 更新实际位置（简化模型）
                    motor->actualPosition += motor->actualVelocity / 1000;
                }
                break;
                
            case MOTOR_MODE_PROFILE_TORQUE:
                // 转矩控制模式
                if (motor->controlWord & 0x0020) {  // 开始运动位
                    // 计算转矩误差
                    int16_t torqueError = motor->targetTorque - motor->actualTorque;
                    
                    // 简单的比例控制
                    int16_t pwmValue = torqueError;  // 简单的比例系数
                    
                    // 限制PWM值
                    if (pwmValue > 500) pwmValue = 500;
                    if (pwmValue < -500) pwmValue = -500;
                    
                    // 设置方向和PWM
                    if (pwmValue >= 0) {
                        MotorControl_SetDirection(motor, 1);
                        MotorControl_SetPWM(motor, (uint16_t)pwmValue);
                    } else {
                        MotorControl_SetDirection(motor, 0);
                        MotorControl_SetPWM(motor, (uint16_t)(-pwmValue));
                    }
                    
                    // 更新实际转矩（简化模型）
                    motor->actualTorque += torqueError / 10;
                    
                    // 更新实际速度（简化模型）
                    motor->actualVelocity = motor->actualTorque * 10;
                    
                    // 更新实际位置（简化模型）
                    motor->actualPosition += motor->actualVelocity / 1000;
                }
                break;
                
            case MOTOR_MODE_HOMING:
                // 回零模式
                if (motor->controlWord & 0x0010) {  // 开始回零位
                    // 简单的回零控制
                    MotorControl_SetDirection(motor, 0);  // 向负方向运动
                    MotorControl_SetPWM(motor, 300);  // 设置PWM值
                    
                    // 更新实际位置（简化模型）
                    motor->actualPosition--;
                    
                    // 更新实际速度（简化模型）
                    motor->actualVelocity = -3000;
                    
                    // 检查是否到达零点（简化模型）
                    if (motor->actualPosition <= 0) {
                        motor->actualPosition = 0;
                        motor->actualVelocity = 0;
                        MotorControl_SetPWM(motor, 0);
                        
                        // 设置回零完成标志
                        motor->statusWord |= 0x1000;
                    }
                }
                break;
                
            default:
                // 不支持的模式
                break;
        }
    }
}

/* 处理控制字 */
void MotorControl_ProcessControlWord(MotorControl* motor)
{
    uint16_t controlWord = motor->controlWord;
    
    // 处理关闭
    if ((controlWord & 0x0004) == 0) {
        // 关闭电机
        MotorControl_Disable(motor);
        motor->state = MOTOR_STATE_SWITCH_ON_DISABLED;
        return;
    }
    
    // 处理故障复位
    if ((controlWord & 0x0080) != 0) {
        MotorControl_FaultReset(motor);
        return;
    }
    
    // 根据当前状态处理控制字
    switch (motor->state) {
        case MOTOR_STATE_NOT_READY_TO_SWITCH_ON:
            // 等待就绪
            break;
            
        case MOTOR_STATE_SWITCH_ON_DISABLED:
            // 准备切换到就绪状态
            if ((controlWord & 0x0006) == 0x0006) {
                motor->state = MOTOR_STATE_READY_TO_SWITCH_ON;
            }
            break;
            
        case MOTOR_STATE_READY_TO_SWITCH_ON:
            // 切换到开启状态
            if ((controlWord & 0x0007) == 0x0007) {
                motor->state = MOTOR_STATE_SWITCHED_ON;
            }
            break;
            
        case MOTOR_STATE_SWITCHED_ON:
            // 切换到操作使能状态
            if ((controlWord & 0x000F) == 0x000F) {
                motor->state = MOTOR_STATE_OPERATION_ENABLED;
                MotorControl_Enable(motor);
                // 设置操作模式
                MotorControl_SetOperationMode(motor, motor->operationMode);
            }
            break;
            
        case MOTOR_STATE_OPERATION_ENABLED:
            // 处理操作使能状态下的控制字
            if ((controlWord & 0x0008) == 0) {
                // 禁止操作
                motor->state = MOTOR_STATE_SWITCHED_ON;
                MotorControl_Disable(motor);
            } else if ((controlWord & 0x0010) != 0) {
                // 开始回零
                MotorControl_StartHoming(motor, motor->homingMethod);
            } else if ((controlWord & 0x0020) != 0) {
                // 开始运动
                switch (motor->mode) {
                    case MOTOR_MODE_PROFILE_POSITION:
                        MotorControl_SetTargetPosition(motor, motor->targetPosition);
                        break;
                    case MOTOR_MODE_PROFILE_VELOCITY:
                        MotorControl_SetTargetVelocity(motor, motor->targetVelocity);
                        break;
                    case MOTOR_MODE_PROFILE_TORQUE:
                        MotorControl_SetTargetTorque(motor, motor->targetTorque);
                        break;
                    default:
                        break;
                }
            } else if ((controlWord & 0x0040) != 0) {
                // 快速停止
                MotorControl_SetPWM(motor, 0);
            }
            break;
            
        case MOTOR_STATE_QUICK_STOP_ACTIVE:
            // 处理快速停止状态
            if ((controlWord & 0x0006) == 0x0006) {
                motor->state = MOTOR_STATE_READY_TO_SWITCH_ON;
            }
            break;
            
        case MOTOR_STATE_FAULT_REACTION_ACTIVE:
            // 处理故障反应状态
            break;
            
        case MOTOR_STATE_FAULT:
            // 处理故障状态
            if ((controlWord & 0x0080) != 0) {
                MotorControl_FaultReset(motor);
            }
            break;
            
        default:
            break;
    }
}

/* 更新状态字 */
void MotorControl_UpdateStatusWord(MotorControl* motor)
{
    // 根据当前状态更新状态字
    switch (motor->state) {
        case MOTOR_STATE_NOT_READY_TO_SWITCH_ON:
            motor->statusWord = 0x0000;
            break;
            
        case MOTOR_STATE_SWITCH_ON_DISABLED:
            motor->statusWord = 0x0040;
            break;
            
        case MOTOR_STATE_READY_TO_SWITCH_ON:
            motor->statusWord = 0x0021;
            break;
            
        case MOTOR_STATE_SWITCHED_ON:
            motor->statusWord = 0x0023;
            break;
            
        case MOTOR_STATE_OPERATION_ENABLED:
            motor->statusWord = 0x0027;
            // 检查是否达到目标位置
            if (abs(motor->targetPosition - motor->actualPosition) < 10) {
                motor->statusWord |= 0x0100;
            } else {
                motor->statusWord &= ~0x0100;
            }
            // 检查是否达到目标速度
            if (abs(motor->targetVelocity - motor->actualVelocity) < 100) {
                motor->statusWord |= 0x0200;
            } else {
                motor->statusWord &= ~0x0200;
            }
            break;
            
        case MOTOR_STATE_QUICK_STOP_ACTIVE:
            motor->statusWord = 0x0007;
            break;
            
        case MOTOR_STATE_FAULT_REACTION_ACTIVE:
            motor->statusWord = 0x000F;
            break;
            
        case MOTOR_STATE_FAULT:
            motor->statusWord = 0x0008;
            break;
            
        default:
            motor->statusWord = 0x0000;
            break;
    }
    
    // 设置警告位
    if (0 /* 添加您的警告检测条件 */) {
        motor->statusWord |= 0x0080;
    } else {
        motor->statusWord &= ~0x0080;
    }
}

/* 设置操作模式 */
void MotorControl_SetOperationMode(MotorControl* motor, uint8_t mode)
{
    motor->operationMode = mode;
    motor->operationModeDisplay = mode;
    
    switch (mode) {
        case MOTOR_MODE_PROFILE_POSITION:
            motor->mode = MOTOR_MODE_PROFILE_POSITION;
            break;
            
        case MOTOR_MODE_PROFILE_VELOCITY:
            motor->mode = MOTOR_MODE_PROFILE_VELOCITY;
            break;
            
        case MOTOR_MODE_PROFILE_TORQUE:
            motor->mode = MOTOR_MODE_PROFILE_TORQUE;
            break;
            
        case MOTOR_MODE_HOMING:
            motor->mode = MOTOR_MODE_HOMING;
            break;
            
        default:
            // 不支持的模式
            break;
    }
}

/* 设置目标位置 */
void MotorControl_SetTargetPosition(MotorControl* motor, int32_t position)
{
    motor->targetPosition = position;
}

/* 设置目标速度 */
void MotorControl_SetTargetVelocity(MotorControl* motor, int32_t velocity)
{
    motor->targetVelocity = velocity;
}

/* 设置目标转矩 */
void MotorControl_SetTargetTorque(MotorControl* motor, int16_t torque)
{
    motor->targetTorque = torque;
}

/* 开始回零 */
void MotorControl_StartHoming(MotorControl* motor, uint8_t method)
{
    motor->homingMethod = method;
    motor->mode = MOTOR_MODE_HOMING;
    motor->statusWord &= ~0x1000;  // 清除回零完成标志
}

/* 复位电机 */
void MotorControl_Reset(MotorControl* motor)
{
    MotorControl_Disable(motor);
    motor->state = MOTOR_STATE_NOT_READY_TO_SWITCH_ON;
}

/* 故障复位 */
void MotorControl_FaultReset(MotorControl* motor)
{
    MotorControl_Disable(motor);
    motor->state = MOTOR_STATE_SWITCH_ON_DISABLED;
}

/* 使能电机 */
void MotorControl_Enable(MotorControl* motor)
{
    GPIO_SetBits(motor->enablePort, motor->enablePin);
}

/* 禁用电机 */
void MotorControl_Disable(MotorControl* motor)
{
    GPIO_ResetBits(motor->enablePort, motor->enablePin);
    MotorControl_SetPWM(motor, 0);
}

/* 设置方向 */
void MotorControl_SetDirection(MotorControl* motor, uint8_t direction)
{
    if (direction) {
        GPIO_SetBits(motor->dirPort, motor->dirPin);
    } else {
        GPIO_ResetBits(motor->dirPort, motor->dirPin);
    }
}

/* 设置PWM值 */
void MotorControl_SetPWM(MotorControl* motor, uint16_t pwmValue)
{
    // 限制PWM值
    if (pwmValue > 1000) pwmValue = 1000;
    
    // 根据通道设置PWM
    switch(motor->pwmChannel) {
        case 1:
            TIM_SetCompare1(motor->pwmTimer, pwmValue);
            break;
        case 2:
            TIM_SetCompare2(motor->pwmTimer, pwmValue);
            break;
        case 3:
            TIM_SetCompare3(motor->pwmTimer, pwmValue);
            break;
        case 4:
            TIM_SetCompare4(motor->pwmTimer, pwmValue);
            break;
    }
}
