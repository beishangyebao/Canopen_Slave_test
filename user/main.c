#include "canfestival.h"
#include "stm32f10x.h"
#include "TestSlave.h"
#include "bsp_can.h"
#include "bsp_timer.h"
#include "delay.h"
#include "bsp_led.h"
#include "can_app.h"
#include "generator.h"

/* 主程序入口，负责初始化板级外设、CANopen 节点和轨迹发生器。 */

extern CO_Data TestSlave_Data;
extern volatile uint32_t g_last_g4_heartbeat_time;

#define nodeID 0x01

int main(void)
{
    /* 先配置中断优先级分组，保证后续外设中断优先级关系正确。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 完成芯片时钟和基础延时模块初始化。 */
    SystemInit();
    delay_init();

    /* 初始化 CAN、时间基准和板载 LED 等底层外设。 */
    bsp_can_init();
    bsp_timer_init();
    bsp_led_init();

    /* 注册对象字典回调，让应用层接管运动相关对象。 */
    User_Slave_Init();

    /* 设置从站节点号，并让 CANopen 从站进入预操作态。 */
    setNodeId(&TestSlave_Data, nodeID);
    setState(&TestSlave_Data, Initialisation);
    setState(&TestSlave_Data, Pre_operational);

    /* 用当前系统时基初始化 G4 心跳时间戳，避免刚启动就被判超时。 */
    g_last_g4_heartbeat_time = GetTick();
    
    /* 启动 1 ms 周期的轨迹发生器。 */
    Generator_Init(1000u);

    while (1)
    {
        /* 主循环里持续检查 G4 反馈是否超时。 */
        CAN_App_Check_G4_Watchdog();

        /* 把 TIM3 累积的节拍逐个消费，避免主循环偶尔卡顿时漏掉周期。 */
        while (Generator_ConsumeTick()) {
            /* 每消费一个 1 ms 节拍，就执行一次发生器周期任务。 */
            Generator_Run();
        }
    }
}
