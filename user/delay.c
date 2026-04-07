#include "delay.h"
#include "stm32f10x.h"

// 全局系统时间计数器 (在 stm32f10x_it.c 的中断里累加)
// 必须与中断文件里的变量名保持一致
extern volatile uint32_t g_SystemTickCount; 

// 空的 delay_osschedlock (为了兼容旧代码调用)
void delay_osschedlock(void) {}
void delay_osschedunlock(void) {}
void delay_ostimedly(u32 ticks) {}
void SysTick_Handler(void); // 声明一下

// 初始化延迟函数
// 配置 SysTick 为 1ms 中断，且开启中断
void delay_init(void)
{
    // SystemCoreClock 一般为 72000000 (72MHz)
    // 配置为 1ms 中断一次
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // Capture error
        while (1);
    }
}

// 获取系统运行时间 (ms)
uint32_t GetTick(void)
{
    return g_SystemTickCount;
}

// 毫秒级延时
// 原理：读取当前 GetTick，死等直到差值达到 nms
void delay_ms(u16 nms)
{
    uint32_t start = GetTick();
    
    // 注意：即便 g_SystemTickCount 溢出（49天后），
    // 由于是无符号减法，这个逻辑依然是正确的
    while((GetTick() - start) < nms);
}

// 微秒级延时 (粗略实现)
// 因为 SysTick 被 1ms 中断占用了，这里用简单的空循环模拟
// 72MHz 下，大约 72 个周期是 1us
void delay_us(u32 nus)
{
    __IO uint32_t i;
    // 粗略估算：72MHz / 5指令周期 ≈ 14 (根据编译器优化不同调整)
    // 这种实现不精确，但足够驱动 GPIO/I2C
    i = nus * 9; 
    while(i--);
}



