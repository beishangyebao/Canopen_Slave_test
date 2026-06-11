#include "can_app.h"

#include "TestSlave.h"
#include "canfestival.h"
#include "generator.h"

/* CANopen 应用层胶水代码，负责对象字典回调和 G4 通信监督。 */

extern CO_Data TestSlave_Data;
extern uint32_t GetTick(void);
extern volatile uint32_t g_last_g4_heartbeat_time;
extern INTEGER8 TestSlave_obj6061;

volatile uint8_t g_g4_comm_timeout = 0u;
volatile uint8_t g_g4_feedback_seen = 0u;
volatile uint8_t g_g4_feedback_online = 0u;

/*
 * 这几个常量共同定义 G4 反馈链路的“最小可信通信条件”。
 *
 * - STARTUP_GRACE:
 *   上电后给 G4 一个启动宽限期，避免驱动刚上电就立刻被判 timeout。
 *
 * - TIMEOUT:
 *   在线后，超过这个时间没有任何反馈就判定为通信中断。
 *
 * - RECOVERY_FRAMES:
 *   timeout 后不会因为偶尔回来一帧就立刻恢复，
 *   而是要求连续收到多帧有效反馈后才认为链路重新稳定。
 */
#define G4_WATCHDOG_STARTUP_GRACE_MS   1000u
#define G4_WATCHDOG_TIMEOUT_MS         500u
#define G4_WATCHDOG_RECOVERY_FRAMES    3u

typedef enum
{
    G4_LINK_WAIT_FIRST_FEEDBACK = 0,
    G4_LINK_ONLINE,
    G4_LINK_TIMEOUT
} G4_LinkState;

static G4_LinkState s_g4_link_state = G4_LINK_WAIT_FIRST_FEEDBACK;
static volatile uint32_t s_g4_feedback_counter = 0u;
static uint32_t s_g4_last_observed_counter = 0u;
static uint8_t s_g4_recovery_frames = 0u;
static uint8_t s_g4_watchdog_bootstrapped = 0u;
static uint32_t s_g4_watchdog_start_time = 0u;

/*
 * 0x6061 是从站内部产生并上报给主站的模式显示对象。
 * 对主站保持 RO 是对的，但从站内部仍需要更新它。
 *
 * 因此这里不走面向主站访问控制的 setODentry()，
 * 而是走从站内部专用的 writeLocalDict(..., checkAccess = 0)。
 * 这样既保持了对象对主站只读，又允许从站把自身状态写回 OD。
 */
static void CAN_App_WriteSlaveProducedODI8IfChanged(UNS16 index, INTEGER8 *object, INTEGER8 value)
{
    if (*object != value) {
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

/* 工作模式更新后，把相同值同步到 0x6061。 */
static UNS32 OnOperationModeUpdate(CO_Data* d, const indextable *table, UNS8 bSubindex)
{
    INTEGER8 mode = *(INTEGER8*)table->pSubindex[bSubindex].pObject;

    (void)d;

    CAN_App_WriteSlaveProducedODI8IfChanged(0x6061, &TestSlave_obj6061, mode);
    return 0;
}

/*
 * 运动相关对象的写入不在回调里直接执行，而是留给下一个 1 ms 周期处理。
 * 这样能保证控制字、模式和目标值在同一拍里被统一读取，避免半拍状态。
 */
static UNS32 OnMotionCommandUpdate(CO_Data* d, const indextable *table, UNS8 bSubindex)
{
    (void)d;
    (void)table;
    (void)bSubindex;
    return 0;
}

/* 注册运动控制相关对象字典项的应用层回调。 */
void User_Slave_Init(void)
{
    RegisterSetODentryCallBack(&TestSlave_Data, 0x6060, 0x00, OnOperationModeUpdate);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x6040, 0x00, OnMotionCommandUpdate);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x60FF, 0x00, OnMotionCommandUpdate);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x6071, 0x00, OnMotionCommandUpdate);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x607A, 0x00, OnMotionCommandUpdate);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x6098, 0x00, OnMotionCommandUpdate);
}

/*
 * 每收到一帧有效 G4 反馈，都由 CAN 中断调用本函数登记“收到反馈”这个事实。
 *
 * 这里只做轻量记录，不直接在中断里执行 timeout/recover 判定。
 * timeout 状态机统一在主循环中的 CAN_App_Check_G4_Watchdog() 内完成，
 * 这样中断路径更短，也能避免复杂状态在中断上下文里抖动。
 */
void CAN_App_OnG4FeedbackReceived(void)
{
    g_last_g4_heartbeat_time = GetTick();
    g_g4_feedback_seen = 1u;
    ++s_g4_feedback_counter;

    /*
     * 首帧反馈到来时，说明链路至少已经“建立”。
     * timeout 后的恢复不在这里立即完成，而是交给 watchdog
     * 按连续多帧反馈的条件去判定。
     */
    if (s_g4_link_state == G4_LINK_WAIT_FIRST_FEEDBACK) {
        s_g4_link_state = G4_LINK_ONLINE;
        g_g4_feedback_online = 1u;
        g_g4_comm_timeout = 0u;
        s_g4_recovery_frames = 0u;
        s_g4_last_observed_counter = s_g4_feedback_counter;
    }
}

/* 检查 G4 反馈是否超时，用于通信掉线检测。 */
void CAN_App_Check_G4_Watchdog(void)
{
    uint32_t now = GetTick();
    uint32_t elapsed_since_feedback;

    /*
     * main.c 在进入主循环前会把 g_last_g4_heartbeat_time 初始化成当前时刻。
     * 这里第一次运行时把它当作“启动参考时刻”，用于首帧超时判定。
     */
    if (s_g4_watchdog_bootstrapped == 0u) {
        s_g4_watchdog_bootstrapped = 1u;
        s_g4_watchdog_start_time = g_last_g4_heartbeat_time;
        s_g4_last_observed_counter = s_g4_feedback_counter;
    }

    elapsed_since_feedback = now - g_last_g4_heartbeat_time;

    switch (s_g4_link_state) {
        case G4_LINK_WAIT_FIRST_FEEDBACK:
            /*
             * 启动阶段还没见到首帧时，不认为链路在线。
             * 但也不是无限等待；如果宽限期耗尽仍无反馈，就进入 timeout，
             * 彻底消除“首帧前永远不报错”的盲区。
             */
            g_g4_feedback_online = 0u;

            if ((now - s_g4_watchdog_start_time) > G4_WATCHDOG_STARTUP_GRACE_MS) {
                s_g4_link_state = G4_LINK_TIMEOUT;
                g_g4_comm_timeout = 1u;
                s_g4_recovery_frames = 0u;
                s_g4_last_observed_counter = s_g4_feedback_counter;
            } else {
                g_g4_comm_timeout = 0u;
            }
            break;

        case G4_LINK_ONLINE:
            /*
             * 在线阶段只要超过 timeout 窗口没收到反馈，就认为链路掉线。
             * 一旦掉线，立即清掉 online 标志，让运动使能条件同步失效。
             */
            if (elapsed_since_feedback > G4_WATCHDOG_TIMEOUT_MS) {
                s_g4_link_state = G4_LINK_TIMEOUT;
                g_g4_comm_timeout = 1u;
                g_g4_feedback_online = 0u;
                s_g4_recovery_frames = 0u;
                s_g4_last_observed_counter = s_g4_feedback_counter;
            } else {
                g_g4_comm_timeout = 0u;
                g_g4_feedback_online = 1u;
            }
            break;

        case G4_LINK_TIMEOUT:
        default:
            /*
             * timeout 后要求连续收到多帧反馈才能恢复。
             * 这样可以避免总线抖动时 timeout/recover 频繁来回翻转。
             */
            g_g4_comm_timeout = 1u;
            g_g4_feedback_online = 0u;

            if (elapsed_since_feedback > G4_WATCHDOG_TIMEOUT_MS) {
                s_g4_recovery_frames = 0u;
                s_g4_last_observed_counter = s_g4_feedback_counter;
                break;
            }

            if (s_g4_feedback_counter != s_g4_last_observed_counter) {
                uint32_t delta = s_g4_feedback_counter - s_g4_last_observed_counter;

                s_g4_last_observed_counter = s_g4_feedback_counter;

                if (delta >= G4_WATCHDOG_RECOVERY_FRAMES) {
                    s_g4_recovery_frames = G4_WATCHDOG_RECOVERY_FRAMES;
                } else {
                    uint32_t sum = (uint32_t)s_g4_recovery_frames + delta;
                    s_g4_recovery_frames = (sum >= G4_WATCHDOG_RECOVERY_FRAMES) ?
                        G4_WATCHDOG_RECOVERY_FRAMES : (uint8_t)sum;
                }
            }

            if (s_g4_recovery_frames >= G4_WATCHDOG_RECOVERY_FRAMES) {
                s_g4_link_state = G4_LINK_ONLINE;
                g_g4_comm_timeout = 0u;
                g_g4_feedback_online = 1u;
                s_g4_recovery_frames = 0u;
            }
            break;
    }
}
