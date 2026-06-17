#include "canfestival.h"
#include "stm32f10x.h"
#include "TestSlave.h"
#include "bsp_can.h"
#include "bsp_timer.h"
#include "delay.h"
#include "bsp_led.h"
#include "can_app.h"
#include "ds402_state.h"
#include "generator.h"
#include "torque.h"


extern CO_Data TestSlave_Data;
extern volatile uint32_t g_last_g4_heartbeat_time;

/*
 * 0x6060 Modes of operation 由 CanFestival 对象字典生成
 * main.c 只读取该对象来做 1 ms 应用节拍分发，不直接修改对象字典定义
 */
extern INTEGER8 TestSlave_obj6060;

#define nodeID 0x01
#define MAIN_OPMODE_PROFILE_TORQUE 4

/*
 * 函数作用：
 * 判断当前 0x6060 是否选择 Profile Torque 模式
 *
 * 返回：
 * true 表示本周期由 torque.c 接管运动输出；false 表示由 generator.c 接管
 */
static bool Main_IsTorqueMode(void)
{
    /* 0x6060 是有符号模式对象，显式转成 int8_t 后再比较 */
    return ((int8_t)TestSlave_obj6060 == MAIN_OPMODE_PROFILE_TORQUE);
}

/*
 * 函数作用：
 * 在每个 1 ms 周期开始时准备输入并更新统一 DS402 状态机
 *
 * 参数：
 * torque_mode : 当前周期是否运行 Profile Torque 模式
 */
static void Main_UpdateDs402State(bool torque_mode)
{
    DS402_Input input;

    /*
     * DS402 状态机每个 1 ms 只能更新一次
     * 这里按当前模式收集故障源和停机完成条件，随后统一调用 Ds402_Update()
     */
    if (torque_mode) {
        Torque_FillDs402Input(&input);
    } else {
        Generator_FillDs402Input(&input);
    }

    Ds402_Update(&input);

    /*
     * fault reset 被状态机接受后，同时通知两个运动模块清理本地软状态
     * 这样即使故障发生后切换了 0x6060，也不会留下跨模式的旧锁存
     */
    if (Ds402_ConsumeFaultResetAccepted()) {
        Generator_OnDs402FaultReset();
        Torque_OnDs402FaultReset();
    }
}

/*
 * 函数作用：
 * 在运动模块完成本周期计算后，统一生成并发布 0x6041 Statusword
 *
 * 参数：
 * torque_mode : 当前周期是否运行 Profile Torque 模式
 */
static void Main_PublishDs402Statusword(bool torque_mode)
{
    DS402_StatusBits bits;

    /*
     * 先由 DS402 模块填充公共位 bit4/bit9，再由当前运动模块补充 bit10/bit12/bit13
     */
    Ds402_InitStatusBits(&bits);

    if (torque_mode) {
        Torque_FillDs402StatusBits(&bits);
    } else {
        Generator_FillDs402StatusBits(&bits);
    }

    Ds402_PublishStatusword(&bits);
}

int main(void)
{
    /* 先配置中断优先级分组，保证后续外设中断优先级关系正确 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 完成芯片时钟和基础延时模块初始化 */
    SystemInit();
    delay_init();

    /* 初始化 CAN、时间基准和板载 LED 等底层外设 */
    bsp_can_init();
    bsp_timer_init();
    bsp_led_init();

    /* 注册对象字典回调，让应用层接管运动相关对象 */
    User_Slave_Init();

    /* 设置从站节点号，并让 CANopen 从站进入预操作态 */
    setNodeId(&TestSlave_Data, nodeID);
    setState(&TestSlave_Data, Initialisation);
    setState(&TestSlave_Data, Pre_operational);

    /* 用当前系统时基初始化 G4 心跳时间戳，避免刚启动就被判超时 */
    g_last_g4_heartbeat_time = GetTick();
    
    /*
     * 初始化统一 DS402 状态机
     * 该状态机必须只在上电流程初始化，后续模式切换不能重置它
     */
    Ds402_Init();

    /*
     * 启动 1 ms 应用层控制节拍
     * TIM3 的节拍由 generator.c 中的既有代码提供，但主循环会按 0x6060
     * 把同一节拍分发给位置/速度轨迹发生器或转矩模式处理器
     */
    Generator_Init(1000u);
    Torque_Init(1000u);

    while (1)
    {
        /* 主循环里持续检查 G4 反馈是否超时 */
        CAN_App_Check_G4_Watchdog();

        /* 把 TIM3 累积的节拍逐个消费，避免主循环偶尔卡顿时漏掉周期 */
        while (Generator_ConsumeTick()) {
            bool torque_mode = Main_IsTorqueMode();

            /*
             * 同一个 1 ms 节拍先更新 DS402 状态机，再运行运动模块
             * 这样 0x6040/0x6041 的标准状态跳转不会被不同模式各自解释
             */
            Main_UpdateDs402State(torque_mode);

            if (torque_mode) {
                Torque_Run();
            } else {
                Torque_Reset();
                Generator_Run();
            }

            /* 运动模块更新完 bit10/bit12/bit13 后，再统一发布 0x6041 */
            Main_PublishDs402Statusword(torque_mode);
        }
    }
}
