#include "generator.h"

#include "TestSlave.h"
#include "can_app.h"
#include "canfestival.h"
#include "emcy.h"
#include "g4_motor.h"
#include "stm32f10x.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

/* 轨迹发生器模块，负责在 CANopen 对象字典与 G4 驱动之间做桥接。 */

extern CO_Data TestSlave_Data;
extern volatile uint8_t g_g4_fault_status;
extern volatile uint8_t g_g4_comm_timeout;

/*
 * 下面这些对象字典变量由 CanFestival 生成，轨迹发生器直接读取或回写它们。
 *
 * 常用控制/状态对象：
 * 0x6040 控制字
 * 0x6041 状态字
 * 0x6060 工作模式
 * 0x6061 工作模式显示
 *
 * 常用反馈/目标对象：
 * 0x6062 需求位置
 * 0x6064 实际位置
 * 0x6065 跟随误差窗口
 * 0x6066 跟随误差超时
 * 0x606C 实际速度
 * 0x6071 目标转矩
 * 0x6074 转矩需求值
 * 0x6077 实际转矩
 * 0x607A 目标位置
 * 0x607D 软件位置限位
 * 0x60FF 目标速度
 *
 * 常用轮廓参数：
 * 0x607F 最大轮廓速度
 * 0x6080 最大电机速度
 * 0x6081 位置模式速度
 * 0x6083 加速度
 * 0x6084 减速度
 * 0x6085 急停减速度
 * 0x6087 转矩斜率
 * 0x6098/0x6099/0x609A 回零相关参数
 */
extern UNS16 TestSlave_obj6040;
extern UNS16 TestSlave_obj6041;
extern UNS8 TestSlave_obj6060;
extern UNS8 TestSlave_obj6061;
extern UNS32 TestSlave_obj6062;
extern UNS32 TestSlave_obj6064;
extern UNS32 TestSlave_obj6065;
extern UNS16 TestSlave_obj6066;
extern UNS32 TestSlave_obj606C;
extern UNS16 TestSlave_obj6071;
extern INTEGER16 TestSlave_obj6074;
extern INTEGER16 TestSlave_obj6075;
extern UNS16 TestSlave_obj6077;
extern UNS32 TestSlave_obj607A;
extern UNS32 TestSlave_obj607D_min;
extern UNS32 TestSlave_obj607D_max;
extern UNS16 TestSlave_obj6072;
extern UNS16 TestSlave_obj6073;
extern UNS32 TestSlave_obj607F;
extern UNS32 TestSlave_obj6080;
extern UNS32 TestSlave_obj6081;
extern UNS32 TestSlave_obj6083;
extern UNS32 TestSlave_obj6084;
extern UNS32 TestSlave_obj6085;
extern UNS32 TestSlave_obj6087;
extern UNS8 TestSlave_obj6098;
extern UNS32 TestSlave_obj6099_speed1;
extern UNS32 TestSlave_obj6099_speed2;
extern UNS32 TestSlave_obj609A;
extern UNS32 TestSlave_obj60FF;

/* 当前工程实际用到的模式值。 */
#define OPMODE_PROFILE_POSITION  1
#define OPMODE_PROFILE_VELOCITY  3
#define OPMODE_PROFILE_TORQUE    4
#define OPMODE_HOMING            6

/*
 * 控制字中本模块关心的位定义
 * 只保留当前状态机真正使用到的位
 */
#define CW_QUICK_STOP            0x0004u
#define CW_NEW_SETPOINT          0x0010u
#define CW_RELATIVE              0x0040u
#define CW_HALT                  0x0100u

/*
 * 轨迹发生器内部统一采用 Q24.8 定点格式处理速度和部分中间量。
 * 这样既能保留低速时的小数精度，又避免运行期引入浮点开销。
 */
#define PROFILE_DEFAULT_POS_ERR  1
#define PROFILE_DEFAULT_VEL_ERR  1
#define PROFILE_Q8_SHIFT         8   // 小数位数8位，决定了速度相关参数的精度和范围
#define PROFILE_Q8_ONE           (1 << PROFILE_Q8_SHIFT)         // 1.0
#define PROFILE_Q8_HALF          (1 << (PROFILE_Q8_SHIFT - 1))   // 0.5
#define PROFILE_MIN_RATE_Q8      1   // 最小非零速度，防止除以零

/* EMCY 上报码，分别对应通信超时、驱动故障、跟随误差。 */
#define EMCY_G4_TIMEOUT          0x8130u
#define EMCY_G4_DEVICE_FAULT     0x5000u
#define EMCY_FOLLOWING_ERROR     0x8611u

/* 软件梯形轨迹规划器使用的内部运动状态。 */
typedef enum
{
    idle_state = 0,
    halt_state,
    qstop_state,
    active_state
} TrapProfile_state;

/* 轨迹规划器的运行时状态，包含规划量和各类锁存标志。 */
typedef struct
{
    /*
     * current_xxx 是发生器内部“规划值”：
     * current_pos 使用 UNS32 环形位置计数，允许过零/回绕保持连续
     * current_vel_q8 使用 Q24.8，位置模式下可理解为 pulse/s，速度模式下为速度给定量
     */
    UNS32 current_pos;
    int64_t current_pos_frac_q8;  //当前位置的小数位
    int32_t current_vel_q8;

    /* 从对象字典读取到的目标命令 */
    UNS32 cmd_target_pos;
    int32_t cmd_target_vel_q8;

    /* 从对象字典同步来的速度/加减速参数 */
    int32_t param_max_vel_q8;
    int32_t param_accel_q8;
    int32_t param_decel_q8;
    int32_t param_qstop_dec_q8;

    /* 控制周期统一保存为 us，避免运行期积分时使用浮点 */
    uint32_t cp_us;

    /* 到位判断阈值：位置误差用脉冲，速度误差用 Q24.8 */
    int32_t err_pos;
    int32_t err_vel_q8;

    /* 跟随误差超时计时器，单位 us */
    uint32_t follow_err_timer_us;

    /* 用于检测模式切换和 New Setpoint 上升沿 */
    int8_t last_op_mode;
    bool last_new_setpoint;

    /* 这些标志位会进一步映射到状态字和 EMCY 处理 */
    bool target_reached;
    bool setpoint_ack;
    bool following_error;
} TrapProfile_par;

static TrapProfile_par s_tp;
static TrapProfile_state s_state = idle_state;
/* TIM3 中断每来一次就累加一个节拍，主循环再慢慢消费。 */
static volatile uint8_t s_tick_pending = 0;
/* 这些标志用于保证同一个故障只上报一次 EMCY，恢复后再清除。 */
static bool s_g4_timeout_emcy_active = false;
static bool s_g4_fault_emcy_active = false;
static bool s_following_emcy_active = false;

/* 基础辅助函数，统一处理符号和溢出细节。 */
/*
 * 对象字典里很多物理量类型是 UNS32，但语义上其实是有符号量。
 * 这里统一转成 int32_t，避免后面每次都在逻辑里显式强转。
 */
static inline int32_t TrapProfile_OdToS32(UNS32 raw)
{
    return (int32_t)raw;
}

static inline int32_t TrapProfile_AbsS32(int32_t x)
{
    return (x >= 0) ? x : ((x == INT32_MIN) ? INT32_MAX : -x);
}

static inline int64_t TrapProfile_AbsS64(int64_t x)
{
    return (x >= 0) ? x : ((x == INT64_MIN) ? INT64_MAX : -x);
}

/*
 * 位置差和位置累加都按 UNS32 环形数处理。
 * 这样编码器位置过零、回零或自然回绕时仍然能保持连续。
 */
static inline int32_t TrapProfile_PosDiff(UNS32 from, UNS32 to)
{
    return (int32_t)(to - from);
}

static inline UNS32 TrapProfile_PosAdd(UNS32 pos, int32_t delta)
{
    return (UNS32)(pos + (UNS32)delta);
}

/* 整数转 Q24.8，先做边界保护，避免左移后溢出 */
static inline int32_t TrapProfile_I32ToQ8(int32_t x)
{
    if (x > (INT32_MAX >> PROFILE_Q8_SHIFT)) {
        return INT32_MAX;
    }
    if (x < (INT32_MIN >> PROFILE_Q8_SHIFT)) {
        return INT32_MIN;
    }

    return (x << PROFILE_Q8_SHIFT);
}

/* Q24.8 四舍五入回 int32，正负数分别处理，避免单纯右移带来的偏差 */
static inline int32_t TrapProfile_Q8ToI32Round(int32_t x_q8)
{
    if (x_q8 >= 0) {
        return (x_q8 + PROFILE_Q8_HALF) >> PROFILE_Q8_SHIFT;
    }

    return -(((-x_q8) + PROFILE_Q8_HALF) >> PROFILE_Q8_SHIFT);
}

/*
 * 从对象字典读取正值参数
 * 若参数为 0 或非法，则回退到 fallback，避免轨迹参数失效导致不动或异常抖动
 */
static inline int32_t TrapProfile_ReadPositive(UNS32 raw, int32_t fallback_q8)
{
    if (raw > 0u) {
        if (raw > (UNS32)(INT32_MAX >> PROFILE_Q8_SHIFT)) {
            return INT32_MAX;
        }
        return ((int32_t)raw << PROFILE_Q8_SHIFT);
    }

    return fallback_q8;
}

/*
 * 最终速度上限取 0x607F 和 0x6080 中更小的那个正值
 * 如果两个都没配置或者过小，则退回到 1.0(Q8) 作为保底值
 */
static inline int32_t TrapProfile_GetMaxVelLimit(void)
{
    bool has_limit = false;
    int32_t limit_q8 = 0;

    if (TestSlave_obj607F > 0u) {
        limit_q8 = TrapProfile_ReadPositive(TestSlave_obj607F, PROFILE_Q8_ONE);
        has_limit = true;
    }

    if (TestSlave_obj6080 > 0u) {
        int32_t limit_6080_q8 = TrapProfile_ReadPositive(TestSlave_obj6080, PROFILE_Q8_ONE);
        limit_q8 = has_limit ? ((limit_q8 < limit_6080_q8) ? limit_q8 : limit_6080_q8) : limit_6080_q8;
        has_limit = true;
    }

    if (!has_limit || limit_q8 < PROFILE_MIN_RATE_Q8) {
        limit_q8 = PROFILE_Q8_ONE;
    }

    return limit_q8;
}

/* 用驱动最新反馈重新对齐软件规划器状态 */
static inline void TrapProfile_ReSyncFromFeedback(TrapProfile_par *tp)
{
    tp->current_pos = TestSlave_obj6064;
    tp->current_pos_frac_q8 = 0;
    tp->current_vel_q8 = TrapProfile_I32ToQ8(TrapProfile_OdToS32(TestSlave_obj606C));
}

/*
 * 把目标位置限制到软件限位 0x607D 内
 * 这里只限制“命令目标”，防止主站下发的目标本身越界了，但不干预发生器内部的规划位置
 */
static inline void TrapProfile_ClampTargetPos(TrapProfile_par *tp)
{
    int32_t min_pos = TrapProfile_OdToS32(TestSlave_obj607D_min);
    int32_t max_pos = TrapProfile_OdToS32(TestSlave_obj607D_max);

    int32_t target_pos = (int32_t)tp->cmd_target_pos;

    if (max_pos > min_pos) {
        if (target_pos < min_pos) {
            target_pos = min_pos;
        }
        if (target_pos > max_pos) {
            target_pos = max_pos;
        }
        tp->cmd_target_pos = (UNS32)target_pos;
    }
}

/*
 * 发生器积分出的规划位置也要做限位保护
 * 一旦已经顶到限位且速度还想继续往外走，就把对应方向的速度直接清零
 */
static inline void TrapProfile_ClampPlannedPos(TrapProfile_par *tp)
{
    int32_t min_pos = TrapProfile_OdToS32(TestSlave_obj607D_min);
    int32_t max_pos = TrapProfile_OdToS32(TestSlave_obj607D_max);
    int32_t current_pos = (int32_t)tp->current_pos;

    if (max_pos > min_pos) {
        if (current_pos < min_pos) {
            tp->current_pos = (UNS32)min_pos;
            tp->current_pos_frac_q8 = 0;
            if (tp->current_vel_q8 < 0) {
                tp->current_vel_q8 = 0;
            }
        }

        if (current_pos > max_pos) {
            tp->current_pos = (UNS32)max_pos;
            tp->current_pos_frac_q8 = 0;
            if (tp->current_vel_q8 > 0) {
                tp->current_vel_q8 = 0;
            }
        }
    }
}

/* 按给定加减速度限制，把当前速度斜坡逼近目标速度 */
static inline int32_t TrapProfile_RampTo(int32_t current_q8,
                                         int32_t target_q8,
                                         int32_t accel_q8,
                                         int32_t decel_q8,
                                         uint32_t dt_us)
{
    int64_t delta = (int64_t)target_q8 - (int64_t)current_q8;
    bool same_direction;
    bool magnitude_increasing;
    bool use_accel;
    int32_t rate_q8;
    int64_t max_step;

    /* 已经足够接近目标时直接返回目标，避免在附近来回抖动 */
    if (TrapProfile_AbsS64(delta) <= 1) {
        return target_q8;
    }

    /*
     * 同向且目标幅值更大，说明当前是在“提速”，使用 accel
     * 其余情况都按减速逻辑处理，包括反向切换和减到零
     */
    same_direction = ((current_q8 > 0 && target_q8 > 0) ||
                      (current_q8 < 0 && target_q8 < 0) ||
                      (current_q8 == 0));

    magnitude_increasing = (TrapProfile_AbsS32(target_q8) > TrapProfile_AbsS32(current_q8));

    //同时满足"同方向"和"速度大小增加" 才视为加速 
    use_accel = same_direction && magnitude_increasing;

    /* 同向提速用 accel，其他情况用 decel */
    rate_q8 = use_accel ? accel_q8 : decel_q8;

    /* 下限保护，防止参数极小导致速度几乎不动 */
    if (rate_q8 < PROFILE_MIN_RATE_Q8) {
        rate_q8 = PROFILE_MIN_RATE_Q8;
    }

    /* rate_q8 为对象字典读取的速度，乘 dt_us 后除 1e6 得到每个控制周期内的速度步长 */
    max_step = ((int64_t)rate_q8 * (int64_t)dt_us) / 1000000LL;

    // 如果步长小于1，则步长为1
    if (max_step < 1) {
        max_step = 1;
    }

    /* 一个周期就能到目标时，直接到目标 */
    if (TrapProfile_AbsS64(delta) <= max_step) {
        return target_q8;
    }

    if (delta > 0) {
        int64_t next = (int64_t)current_q8 + max_step;
        return (next > INT32_MAX) ? INT32_MAX : (int32_t)next;
    }

    {
        int64_t next = (int64_t)current_q8 - max_step;
        return (next < INT32_MIN) ? INT32_MIN : (int32_t)next;
    }
}

/* 用当前反馈和保守默认参数初始化规划器 */
static void TrapProfile_Init(TrapProfile_par *tp,
                             TrapProfile_state *ts,
                             uint32_t cp_us)
{
    /*
     * 初始化时先把规划器对齐到当前反馈
     * 这样上电、复位或模式切换后不会从错误的内部状态起步
     */
    TrapProfile_ReSyncFromFeedback(tp);

    tp->cmd_target_pos = tp->current_pos;
    tp->cmd_target_vel_q8 = 0;

    tp->param_max_vel_q8 = PROFILE_Q8_ONE;
    tp->param_accel_q8 = PROFILE_Q8_ONE;
    tp->param_decel_q8 = PROFILE_Q8_ONE;
    tp->param_qstop_dec_q8 = PROFILE_Q8_ONE;

    tp->cp_us = (cp_us == 0u) ? 1000u : cp_us;

    /* 按编码器分辨率、机械精度调整 */
    tp->err_pos = PROFILE_DEFAULT_POS_ERR;
    tp->err_vel_q8 = TrapProfile_I32ToQ8(PROFILE_DEFAULT_VEL_ERR);

    tp->follow_err_timer_us = 0u;

    tp->last_op_mode = (int8_t)TestSlave_obj6060;
    tp->last_new_setpoint = ((TestSlave_obj6040 & CW_NEW_SETPOINT) != 0u);

    tp->target_reached = false;
    tp->setpoint_ack = false;
    tp->following_error = false;

    /* 初始化完成后默认处于 idle，等待主站下发新命令 */
    *ts = idle_state;
}

/* 当模式或使能条件不再满足时，清空当前运动状态 */
static void TrapProfile_ResetState(TrapProfile_par *tp,
                                   TrapProfile_state *ts)
{
    /* 重新贴合当前反馈 */
    TrapProfile_ReSyncFromFeedback(tp);

    tp->cmd_target_pos = tp->current_pos;
    tp->cmd_target_vel_q8 = 0;

    tp->follow_err_timer_us = 0u;

    tp->target_reached = false;
    tp->setpoint_ack = false;
    tp->following_error = false;

    tp->last_new_setpoint = ((TestSlave_obj6040 & CW_NEW_SETPOINT) != 0u);

    *ts = idle_state;
}

/* 急停时把指令参考钉到当前反馈，并按急停减速度快速减速 */
static void TrapProfile_EnterQuickStop(TrapProfile_par *tp,
                                       TrapProfile_state *ts,
                                       UNS32 fb_pos) //反馈位置
{
    if (*ts != qstop_state) {
        /* 把目标位置改成当前反馈，避免停住后规划位置与反馈继续拉开 */
        tp->cmd_target_pos = fb_pos;
        tp->cmd_target_vel_q8 = 0;
        tp->target_reached = false;
        tp->setpoint_ack = false;
        *ts = qstop_state;
    }
}

/* Halt 保持当前位置参考，并按常规减速度减速。 */
static void TrapProfile_EnterHalt(TrapProfile_par *tp,
                                  TrapProfile_state *ts)
{
    if (*ts != qstop_state && *ts != halt_state) {
       
        tp->cmd_target_pos = tp->current_pos;
        tp->cmd_target_vel_q8 = 0;
        tp->target_reached = false;
        tp->setpoint_ack = false;
        *ts = halt_state;
    }
}

/* 执行一次规划器步进，并根据 OD 命令更新内部状态机。 */
static bool TrapProfile_Update(TrapProfile_par *tp,
                               TrapProfile_state *ts)
{
    /* ---------- 第 0 步：读取控制字、模式和当前反馈 ---------- */


    uint16_t cw = TestSlave_obj6040;
    int8_t mode = (int8_t)TestSlave_obj6060;
    UNS32 fb_pos = TestSlave_obj6064;

    bool operational = (TestSlave_Data.nodeState == Operational);

    /*
     * 低 4 位全 1 只是主站给出的“允许使能”请求。
     * 真正的本地 enable 还必须满足 G4 反馈链路已经建立，
     * 否则从站不允许在“驱动还未确认在线”的情况下开始执行运动命令。
     */
    bool enabled = operational &&
                   g_g4_feedback_online &&
                   ((cw & 0x000Fu) == 0x000Fu);

    /* Quick Stop / Halt 只有在 enable 后才有意义。 */
    bool qstop_req = enabled && ((cw & CW_QUICK_STOP) == 0u);
    bool halt_req = enabled && ((cw & CW_HALT) != 0u);

    /* New Setpoint 用上升沿检测；relative_move 决定 0x607A 的解释方式。 */
    bool new_setpoint = ((cw & CW_NEW_SETPOINT) != 0u);
    bool relative_move = ((cw & CW_RELATIVE) != 0u);

    /* ---------- 第 1 步：同步最新轨迹参数，并做下限保护 ---------- */


    /* 每个周期都从 OD 读取最新限制，使 SDO/PDO 改动立刻生效。 */
    tp->param_max_vel_q8 = TrapProfile_GetMaxVelLimit();

    tp->param_accel_q8 = TrapProfile_ReadPositive(TestSlave_obj6083, PROFILE_Q8_ONE);
    tp->param_decel_q8 = TrapProfile_ReadPositive(TestSlave_obj6084, PROFILE_Q8_ONE);
    tp->param_qstop_dec_q8 = TrapProfile_ReadPositive(TestSlave_obj6085, tp->param_decel_q8);

    //保证减速度和急停减速度不会小到离谱
    if (tp->param_decel_q8 < PROFILE_MIN_RATE_Q8) {
        tp->param_decel_q8 = PROFILE_MIN_RATE_Q8;
    }
    if (tp->param_qstop_dec_q8 < PROFILE_MIN_RATE_Q8) {
        tp->param_qstop_dec_q8 = PROFILE_MIN_RATE_Q8;
    }

    /* ---------- 第 2 步：模式变化先重对齐，旧轨迹直接作废 ---------- */


    /* 模式切换会使当前轨迹失效，按反馈重新起算。 */
    if (mode != tp->last_op_mode) {
        TrapProfile_ResetState(tp, ts);
        tp->last_op_mode = mode;
        return true;
    }

    /* ---------- 第 3 步：只在位置/速度模式下运行本地轨迹发生器 ---------- */


    /* 只有位置/速度模式使用本地轨迹发生器。 */
    if (mode != OPMODE_PROFILE_POSITION && mode != OPMODE_PROFILE_VELOCITY) {
        TrapProfile_ResetState(tp, ts);
        tp->last_op_mode = mode;
        return true;
    }

    /* ---------- 第 4 步：安全条件判断，必要时转入急停 ---------- */

    /*
     * 只要节点不在 Operational、G4 已报故障、反馈链掉线，
     * 或者链路尚未真正建立，都不允许继续保持运动态。
     */
    if (!operational || !g_g4_feedback_online || g_g4_fault_status || g_g4_comm_timeout) {
        if (TrapProfile_AbsS32(tp->current_vel_q8) > tp->err_vel_q8 ||
            *ts == active_state ||
            *ts == halt_state) {
            TrapProfile_EnterQuickStop(tp, ts, fb_pos);
        } else {
            TrapProfile_ResetState(tp, ts);
            tp->last_op_mode = mode;
            return true;
        }
    }

    //主站下发急停命令
    if (qstop_req) {
        TrapProfile_EnterQuickStop(tp, ts, fb_pos);
    }

    /* 已失能且不在急停中，直接回到跟随反馈的 idle */
    if (!enabled && *ts != qstop_state) {
        TrapProfile_ResetState(tp, ts);
        tp->last_op_mode = mode;
        tp->last_new_setpoint = new_setpoint;
        return true;
    }

    /* ---------- 第 5 步：解释主站新命令 ---------- */


    if (mode == OPMODE_PROFILE_POSITION) {

        /* 在 new set-point 位上升沿启动新的位置运动。 */
        if (new_setpoint && !tp->last_new_setpoint && *ts != qstop_state) {
            UNS32 raw_target_pos = TestSlave_obj607A;

            /*
             * 绝对模式：0x607A 直接就是目标位置。
             * 相对模式：把 0x607A 当有符号增量，加到当前规划位置上。
             * 这里故意用 current_pos 而不是 fb_pos 做基准，是为了保持轨迹连续
             */
            tp->cmd_target_pos = relative_move
                ? TrapProfile_PosAdd(tp->current_pos, TrapProfile_OdToS32(raw_target_pos))
                : raw_target_pos;

            TrapProfile_ClampTargetPos(tp);

            /* 位置模式下的目标速度来自 0x6081，而不是 0x60FF。 */
            tp->cmd_target_vel_q8 = TrapProfile_ReadPositive(TestSlave_obj6081, tp->param_max_vel_q8);

            tp->target_reached = false;
            tp->setpoint_ack = true;
            tp->following_error = false;

            tp->follow_err_timer_us = 0u;

            *ts = active_state;
        }

        /* bit4 清掉后把内部 ack 也清掉，便于主站做握手判断。 */
        if (!new_setpoint) {
            tp->setpoint_ack = false;
        }
    } else {
        int32_t raw_target_vel_q8 = TrapProfile_I32ToQ8(TrapProfile_OdToS32(TestSlave_obj60FF));

        /* 将速度模式目标限制在当前最严格的速度上限内。 */
        if (raw_target_vel_q8 > tp->param_max_vel_q8) {
            raw_target_vel_q8 = tp->param_max_vel_q8;
        }
        if (raw_target_vel_q8 < -tp->param_max_vel_q8) {
            raw_target_vel_q8 = -tp->param_max_vel_q8;
        }

        tp->cmd_target_vel_q8 = raw_target_vel_q8;
        tp->target_reached = false;

        /* 速度模式下，只要目标速度或当前速度还明显非零，就保持 active。 */
        if (*ts != qstop_state && !halt_req) {
            if (TrapProfile_AbsS32(tp->cmd_target_vel_q8) > tp->err_vel_q8 ||
                TrapProfile_AbsS32(tp->current_vel_q8) > tp->err_vel_q8) {
                *ts = active_state;
            } else {
                *ts = idle_state;
            }
        }
    }

    /* ---------- 第 6 步：Halt 请求优先转入 halt_state ---------- */


    if (halt_req && *ts != qstop_state) {
        TrapProfile_EnterHalt(tp, ts);
    }

    /* ---------- 第 7 步：idle 时只跟随反馈，不再做轨迹积分 ---------- */


    /* 空闲态直接跟随实际反馈，并清除临时锁存标志。 */
    if (*ts == idle_state) {
        tp->current_pos = fb_pos;
        tp->current_pos_frac_q8 = 0;
        tp->current_vel_q8 = 0;
        tp->follow_err_timer_us = 0u;
        tp->last_new_setpoint = new_setpoint;
        tp->last_op_mode = mode;
        return true;
    }

    /* ---------- 第 8 步：根据状态和模式更新当前规划速度 ---------- */


    /* 急停、Halt 和正常运动三种状态采用不同的斜坡目标。 */
    if (*ts == qstop_state) {
        tp->current_vel_q8 = TrapProfile_RampTo(tp->current_vel_q8,
                                                0,
                                                tp->param_qstop_dec_q8,
                                                tp->param_qstop_dec_q8,
                                                tp->cp_us);
    } else if (*ts == halt_state) {
        tp->current_vel_q8 = TrapProfile_RampTo(tp->current_vel_q8,
                                                0,
                                                tp->param_decel_q8,
                                                tp->param_decel_q8,
                                                tp->cp_us);
    } else if (mode == OPMODE_PROFILE_POSITION) {

        /*距离误差通过环形差分计算*/
        int32_t dist = TrapProfile_PosDiff(tp->current_pos, tp->cmd_target_pos);

        int32_t abs_dist = TrapProfile_AbsS32(dist);

        // 计算基于剩余距离的“最大允许速度” (v_limit)
        // 如果 dist 很小，v_limit 也会很小，强制电机减速
        /* 速度从v->0 ,安全刹车距离是 v^2/2a
         *  刹车距离大于安全距离时，才允许以当前速度运行，否则就必须减速
         *  v方大于 2*a*dist 就必须减速
         */
        int32_t desired_vel_q8 = 0;


        /* 当剩余距离不足以继续加速时，切换到制动逻辑 */
        if (abs_dist > tp->err_pos) {

             // 决定方向
            int32_t target_dir_vel_q8 = (dist > 0) ? tp->cmd_target_vel_q8 : -tp->cmd_target_vel_q8;

            //当前速度
            int64_t vel_abs_q8 = TrapProfile_AbsS32(tp->current_vel_q8);

            //当前v^2 
            int64_t lhs = vel_abs_q8 * vel_abs_q8;

            //最小安全刹车距离对应的 v^2 
            int64_t rhs = 512LL * (int64_t)tp->param_decel_q8 * (int64_t)abs_dist;

            //当当前速度的 v^2 大于安全刹车距离对应的 v^2 时，必须减速；否则就可以继续以目标速度运行
            desired_vel_q8 = (lhs > rhs) ? 0 : target_dir_vel_q8;
        }

        tp->current_vel_q8 = TrapProfile_RampTo(tp->current_vel_q8,
                                                desired_vel_q8,
                                                tp->param_accel_q8,
                                                tp->param_decel_q8,
                                                tp->cp_us);
    } else {
        tp->current_vel_q8 = TrapProfile_RampTo(tp->current_vel_q8,
                                                tp->cmd_target_vel_q8,
                                                tp->param_accel_q8,
                                                tp->param_decel_q8,
                                                tp->cp_us);
    }

    /* ---------- 第 9 步：最后再做一次总速度限幅 ---------- */


    if (tp->current_vel_q8 > tp->param_max_vel_q8) {
        tp->current_vel_q8 = tp->param_max_vel_q8;
    }
    if (tp->current_vel_q8 < -tp->param_max_vel_q8) {
        tp->current_vel_q8 = -tp->param_max_vel_q8;
    }

    /* ---------- 第 10 步：位置模式按速度积分；其他模式直接跟随反馈 ---------- */


    if (mode == OPMODE_PROFILE_POSITION) {
        /* 将规划速度按 Q8 分数精度积分成规划位置 */

        // 理论位移，即本周期内应该走的位移
        int64_t delta_pos_q8 = ((int64_t)tp->current_vel_q8 * (int64_t)tp->cp_us) / 1000000LL;

        // 总累积量 = 旧零头 + 新位移；acc_q8 代表了从上一次发脉冲到现在，总共累积了多少个脉冲（包含小数）
        int64_t acc_q8 = tp->current_pos_frac_q8 + delta_pos_q8;

        int32_t delta_pulse;

       // 正向运动时，直接右移八位得到整数脉冲，小数点后的余量保留在 current_pos_frac_q8
        if (acc_q8 >= 0) {
            delta_pulse = (int32_t)(acc_q8 >> PROFILE_Q8_SHIFT);
            tp->current_pos_frac_q8 = (acc_q8 & (PROFILE_Q8_ONE - 1));
        } else {
            // 负向运动向下取整 保证了余量永远为正
            // 这里将负数变为正数向上取整，再取反得到向下取整的结果
            delta_pulse = (int32_t)(-(((-acc_q8) + PROFILE_Q8_ONE - 1) >> PROFILE_Q8_SHIFT));
             //小数余量 = 原始值 - 整数部分对应的值
            tp->current_pos_frac_q8 = acc_q8 - ((int64_t)delta_pulse << PROFILE_Q8_SHIFT);
        }

        //更新规划器内部位置，允许自然回绕
        tp->current_pos = TrapProfile_PosAdd(tp->current_pos, delta_pulse);

        /*
        * 积分后立刻做软件限位保护
        * 这样内部规划位置不会漂到限位外面
        */
        TrapProfile_ClampPlannedPos(tp);
    } else {
        tp->current_pos = fb_pos;
        tp->current_pos_frac_q8 = 0;
    }

    /* ---------- 第 11 步：位置模式执行跟随误差监控 ---------- */

    /* 跟随误差依据规划位置与反馈位置的偏差及持续时间判定。 */
    if (mode == OPMODE_PROFILE_POSITION &&
        !tp->following_error &&
        TestSlave_obj6065 > 0u &&
        TestSlave_obj6066 > 0u) {

        /* 跟随误差为环形差分比较，处理编码器回零时不误报 */
        int32_t plan_fb_err = TrapProfile_AbsS32(TrapProfile_PosDiff(tp->current_pos, fb_pos));

        if (plan_fb_err > (int32_t)TestSlave_obj6065) {

            uint64_t next_timer = (uint64_t)tp->follow_err_timer_us + (uint64_t)tp->cp_us;

            tp->follow_err_timer_us = (next_timer > UINT32_MAX) ? UINT32_MAX : (uint32_t)next_timer;

            if (tp->follow_err_timer_us >= ((uint32_t)TestSlave_obj6066 * 1000u)) {
                tp->following_error = true;
                TrapProfile_EnterQuickStop(tp, ts, fb_pos);
            }
        } else {
            tp->follow_err_timer_us = 0u;
        }
    }

    /* ---------- 第 12 步：收尾判断，决定是否回到 idle ---------- */


    /* 停止过程完成后，将内部状态重新贴合反馈再回到空闲态。 */
    if (*ts == qstop_state || *ts == halt_state) {
        if (TrapProfile_AbsS32(tp->current_vel_q8) <= tp->err_vel_q8) {
            tp->current_vel_q8 = 0;
            tp->current_pos = fb_pos;
            tp->current_pos_frac_q8 = 0;

            tp->cmd_target_pos = fb_pos;
            tp->cmd_target_vel_q8 = 0;

            tp->target_reached = false;
            tp->setpoint_ack = false;

            *ts = idle_state;
        }
    } else if (mode == OPMODE_PROFILE_POSITION) {

        /*
         * 位置模式到位条件，建议同时看三件事：
         * 1. 规划器已经跑到目标附近
         * 2. 规划速度已经接近 0
         * 3. 实际反馈位置也已经在目标附近
         * 到位误差同样使用环形差分，保证过零连续
         */
        int32_t plan_err = TrapProfile_AbsS32(TrapProfile_PosDiff(tp->current_pos, tp->cmd_target_pos));
        int32_t fb_err = TrapProfile_AbsS32(TrapProfile_PosDiff(fb_pos, tp->cmd_target_pos));

        if (plan_err <= tp->err_pos &&
            fb_err <= tp->err_pos &&
            TrapProfile_AbsS32(tp->current_vel_q8) <= tp->err_vel_q8) {

            /*
             * 到位后，不是把 current_pos 硬设成 cmd_target_pos，
             * 而是直接同步到反馈位置。
             * 这样能把最后的静态漂移一起消掉。
             */
            tp->target_reached = true;

            tp->current_pos = fb_pos;
            tp->current_pos_frac_q8 = 0;
            tp->current_vel_q8 = 0;
            tp->follow_err_timer_us = 0u;

            *ts = idle_state;
        }
    } else {
        /*
         * 速度模式里没有严格意义上的“位置到位”。
         * 这里把“目标速度已经跟上”看成 target_reached。
         */
        if (TrapProfile_AbsS32(tp->current_vel_q8 - tp->cmd_target_vel_q8) <= tp->err_vel_q8) {
            tp->target_reached = true;
        }

        /* 速度目标和当前速度都足够接近 0 时，速度模式才真正回到 idle。 */
        if (TrapProfile_AbsS32(tp->cmd_target_vel_q8) <= tp->err_vel_q8 &&
            TrapProfile_AbsS32(tp->current_vel_q8) <= tp->err_vel_q8) {
            tp->current_vel_q8 = 0;
            tp->current_pos = fb_pos;
            tp->current_pos_frac_q8 = 0;
            *ts = idle_state;
        }
    }

    tp->last_new_setpoint = new_setpoint;
    tp->last_op_mode = mode;

    return (*ts == idle_state);
}

/// 获取规划器当前规划位置
static inline int32_t TrapProfile_GetPlannedPos(const TrapProfile_par *tp)
{
    /* 内部位置虽然按 UNS32 保存，但接口维持 int32_t 以兼容现有下游 */
    return (int32_t)tp->current_pos;
}

/// 获取规划器当前规划速度
static inline int32_t TrapProfile_GetPlannedVel(const TrapProfile_par *tp)
{
    /* 对外输出前做一次 Q24.8 到整数的四舍五入。 */
    return TrapProfile_Q8ToI32Round(tp->current_vel_q8);
}

/*
 * 下面这些写接口只服务于“从站内部生成、主站只读观察”的对象，
 * 例如 0x6041 / 0x6061 / 0x6064 / 0x606C。
 *
 * 这些对象在 OD 中保持 RO 是正确的 DS402 语义，
 * 因为主站不应该去写状态字、模式显示和反馈值。
 * 但从站内部仍然必须能够把运行结果写回对象字典。
 *
 * 因此这里统一使用 writeLocalDict(..., checkAccess = 0)：
 * - 对主站仍保持只读
 * - 对从站内部允许更新
 * - 继续复用 CanFestival 的对象字典写入路径
 */
static void Generator_WriteODU8IfChanged(UNS16 index, UNS8 *object, UNS8 value)
{
    if (*object != value) {
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

static void Generator_WriteODU16IfChanged(UNS16 index, UNS16 *object, UNS16 value)
{
    if (*object != value) {
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

static void Generator_WriteODU32IfChanged(UNS16 index, UNS32 *object, UNS32 value)
{
    if (*object != value) {
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}


/* 判断当前模式是否属于本模块认识并能正确下发到 G4 的模式。 */
static bool Generator_IsSupportedMode(int8_t mode)
{
    return (mode == OPMODE_PROFILE_POSITION) ||
           (mode == OPMODE_PROFILE_VELOCITY) ||
           (mode == OPMODE_HOMING);
}

/* 只有模式仍有有效命令需要维持时才持续下发指令帧。 */
static bool Generator_ShouldStreamCommand(const G4_CommandFrame *command)
{
    //检查当前模式是否被支持
    if (!Generator_IsSupportedMode(command->mode)) {
        return false;
    }

    //急停和暂停需要下发
    if (command->cmd_type == CMD_TYPE_QUICKSTOP ||
        command->cmd_type == CMD_TYPE_HALT) {
        return true;
    }

    //非使能状态不下发
    if (command->cmd_type != CMD_TYPE_ENABLE) {
        return false;
    }


    //根据模式判断
    switch (command->mode) {

        //位置模式下只要状态机在 active 就继续下发
        case OPMODE_PROFILE_POSITION:
            return (s_state == active_state);

            //速度模式下只要状态不在 idle，或者目标速度或当前速度明显非零就继续下发
        case OPMODE_PROFILE_VELOCITY:
            return (s_state != idle_state) ||
                   (TrapProfile_AbsS32(s_tp.cmd_target_vel_q8) > s_tp.err_vel_q8) ||
                   (TrapProfile_AbsS32(s_tp.current_vel_q8) > s_tp.err_vel_q8);

        //急停模式下只要 new set-point 位还在就继续下发
        //该位为1时表示正在执行回零操作，需要持续下发命令；为0时表示回零完成，不需要再下发
        case OPMODE_HOMING:
            return ((TestSlave_obj6040 & CW_NEW_SETPOINT) != 0u);

        default:
            return false;
    }
}

/* 将 OD 状态和规划器状态转换成一帧发往 G4 的命令。 */
static G4_CommandFrame Generator_BuildCommandFrame(void)
{
    G4_CommandFrame command;

    //获取当前控制字和工作模式
    uint16_t cw = TestSlave_obj6040;
    int8_t mode = (int8_t)TestSlave_obj6060;

    //判断是否处于运行状态
    bool operational = (TestSlave_Data.nodeState == Operational);

    //判断是否处于使能状态：运行状态 + 已建立G4反馈链路 + 控制字低4位全1 + G4无故障 + 无通信超时 + 无跟随误差
    bool enabled = operational &&
                   g_g4_feedback_online &&
                   ((cw & 0x000Fu) == 0x000Fu) &&
                   !g_g4_fault_status &&
                   !g_g4_comm_timeout &&
                   !s_tp.following_error;

    //判断是否处于急停状态：处于急停状态/已使能且控制字急停位为0/有故障/通信超时/有跟随误差
    bool qstop_active = (s_state == qstop_state) ||
                        (enabled && ((cw & CW_QUICK_STOP) == 0u)) ||
                        g_g4_fault_status ||
                        g_g4_comm_timeout ||
                        s_tp.following_error;

    //判断是否处于暂停状态
    bool halt_active = (s_state == halt_state);

    /*
    *   命令帧初始化：
    *   默认设置为禁用状态
    *   设置工作模式
    *   清除控制标志
    *   设置回零方法
    *   目标值初始化为0
    */
    command.cmd_type = CMD_TYPE_DISABLE;
    command.mode = mode;
    command.control_flags = 0u;
    command.homing_method = (int8_t)TestSlave_obj6098;
    command.target_value = 0;


    /* 急停优先级最高，其次是 Halt，最后才是正常使能。 */
    if (qstop_active) {
        command.cmd_type = CMD_TYPE_QUICKSTOP;
    } else if (halt_active) {
        command.cmd_type = CMD_TYPE_HALT;
    } else if (enabled) {
        command.cmd_type = CMD_TYPE_ENABLE;
    }

    // 根据模式设置目标值
    switch (mode) {

        case OPMODE_PROFILE_POSITION:
            /* 位置模式下发送的是软件规划位置，而不是原始目标位置对象 */
            command.target_value = TrapProfile_GetPlannedPos(&s_tp);
            if (command.cmd_type != CMD_TYPE_DISABLE) {
                command.control_flags = G4_FLAG_TRIGGER;
            }
            break;

        case OPMODE_PROFILE_VELOCITY:
            command.target_value = TrapProfile_GetPlannedVel(&s_tp);
            break;
        
        //目标值固定为0 控制字New Setpoint位为1时设置触发标志
        case OPMODE_HOMING:
            command.target_value = 0;
            if ((cw & CW_NEW_SETPOINT) != 0u) {
                command.control_flags = G4_FLAG_TRIGGER;
            }
            break;

        default:
            command.cmd_type = CMD_TYPE_DISABLE;
            command.target_value = 0;
            break;
    }

    return command;
}

/* 生成符合DS402标准的状态字，向主站反馈驱动器的运行状态 */
static UNS16 Generator_BuildStatusWord(void)
{
    //获取当前控制字判断当前命令状态
    uint16_t cw = TestSlave_obj6040;

    //判断是否处于运行状态
    bool operational = (TestSlave_Data.nodeState == Operational);

    bool enabled = operational &&
                   g_g4_feedback_online &&
                   ((cw & 0x000Fu) == 0x000Fu) &&
                   !g_g4_fault_status &&
                   !g_g4_comm_timeout &&
                   !s_tp.following_error;
    bool qstop_active = (s_state == qstop_state) ||
                        (enabled && ((cw & CW_QUICK_STOP) == 0u)) ||
                        g_g4_fault_status ||
                        g_g4_comm_timeout ||
                        s_tp.following_error;

    
    UNS16 status;

    /*
     * 这里返回的是面向主站的“类 DS402 状态字”。
     * 不是完整标准实现，但保留了使能、急停、到位、ack、跟随误差这些关键位。
     */
    if (g_g4_fault_status || g_g4_comm_timeout || s_tp.following_error) {
        status = 0x0208u;
    } else if (enabled) {
        status = 0x0237u;
    } else {
        status = 0x0231u;
    }

    if (qstop_active) {
        /* bit5 体现 quick stop 状态。 */
        status &= (UNS16)(~0x0020u);
    } else {
        status |= 0x0020u;
    }

    if (s_tp.target_reached) {
        /* bit10：目标到达。 */
        status |= 0x0400u;
    }

    if (s_tp.setpoint_ack && (int8_t)TestSlave_obj6060 == OPMODE_PROFILE_POSITION) {
        /* bit12：位置模式下收到并接受了新的 set-point。 */
        status |= 0x1000u;
    }

    if (s_tp.following_error) {
        /* bit13：跟随误差。 */
        status |= 0x2000u;
    }

    return status;
}

/* 随着通信故障和跟随故障状态变化，上报或清除 EMCY */
static void Generator_UpdateEmergencyState(void)
{
    // 通信超时故障
    bool timeout_fault = (g_g4_comm_timeout != 0u);
    // G4驱动设备故障
    bool g4_fault = (g_g4_fault_status != 0u);
    // 跟随误差故障
    bool following_fault = s_tp.following_error;


    /* 每种故障都单独维护 */

    // 故障出现且未上报时：上报EMCY并标记已上报
    if (timeout_fault && !s_g4_timeout_emcy_active) {
        EMCY_setError(&TestSlave_Data, EMCY_G4_TIMEOUT, 0x10u, 0u);
        s_g4_timeout_emcy_active = true;
    } 
        //故障消失且已上报时：清除EMCY并清除上报标记
        else if (!timeout_fault && s_g4_timeout_emcy_active) {
            EMCY_errorRecovered(&TestSlave_Data, EMCY_G4_TIMEOUT);
            s_g4_timeout_emcy_active = false;
    }

    if (g4_fault && !s_g4_fault_emcy_active) {
        EMCY_setError(&TestSlave_Data, EMCY_G4_DEVICE_FAULT, 0x04u, 0u);
        s_g4_fault_emcy_active = true;
    } else if (!g4_fault && s_g4_fault_emcy_active) {
        EMCY_errorRecovered(&TestSlave_Data, EMCY_G4_DEVICE_FAULT);
        s_g4_fault_emcy_active = false;
    }

    if (following_fault && !s_following_emcy_active) {
        EMCY_setError(&TestSlave_Data, EMCY_FOLLOWING_ERROR, 0x20u, 0u);
        s_following_emcy_active = true;
    } else if (!following_fault && s_following_emcy_active) {
        EMCY_errorRecovered(&TestSlave_Data, EMCY_FOLLOWING_ERROR);
        s_following_emcy_active = false;
    }
}

/* 将规划器生成的命令值更新到对象字典中，使主站能够观察到当前的命令状态 */
static void Generator_UpdateDemandObjects(int8_t mode)
{
    if (mode == OPMODE_PROFILE_POSITION) {
        /* 0x6062 需求位置跟随当前规划位置，便于主站观察软件发生器输出。 */
        TestSlave_obj6062 = (UNS32)TrapProfile_GetPlannedPos(&s_tp);
    }

    if (mode == OPMODE_PROFILE_TORQUE) {
        /* 转矩模式下把目标转矩镜像到 0x6074。 */
        TestSlave_obj6074 = (INTEGER16)TestSlave_obj6071;
    }

    if (mode == OPMODE_PROFILE_VELOCITY) {
        /* 速度模式下把目标速度镜像到 0x6075。 */
        TestSlave_obj6075 = (INTEGER16)TrapProfile_GetPlannedVel(&s_tp);
    }
}

/* 配置规划器以及驱动它的 1 ms 定时器。 */
void Generator_Init(uint32_t cycle_us)
{
    TIM_TimeBaseInitTypeDef TimeBaseStruct;

    NVIC_InitTypeDef  NVIC_InitStructure;

    /* 初始化时规划器状态和故障上报状态 */
    TrapProfile_Init(&s_tp, &s_state, cycle_us);

    //初始化定时器节拍计数器
    s_tick_pending = 0u;

    s_g4_timeout_emcy_active = false;
    s_g4_fault_emcy_active = false;
    s_following_emcy_active = false;

    /* 在第一个定时节拍到来前，先发布一次初始状态字。 */
    Generator_WriteODU16IfChanged(0x6041, &TestSlave_obj6041, Generator_BuildStatusWord());

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_DeInit(TIM3);
    TIM_TimeBaseStructInit(&TimeBaseStruct);
    TimeBaseStruct.TIM_Prescaler = 72u - 1u;
    TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TimeBaseStruct.TIM_Period = ((cycle_us == 0u) ? 1000u : cycle_us) - 1u;
    TIM_TimeBaseInit(TIM3, &TimeBaseStruct);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 取走一个由 TIM3 中断产生的待处理节拍。 */
bool Generator_ConsumeTick(void)
{
    bool has_tick = false;

    /* 该计数器会被中断异步修改，因此这里要关中断保护。 */
    __disable_irq();
    
    if (s_tick_pending > 0u) {
        /* 每次只消费一个节拍，主循环可通过 while 把积压节拍慢慢清空。 */
        --s_tick_pending;
        has_tick = true;
    }
    __enable_irq();

    return has_tick;
}

/* 1 ms 主任务：更新轨迹、镜像状态并发送命令。 */
void Generator_Run(void)
{
    int8_t mode = (int8_t)TestSlave_obj6060;
    G4_CommandFrame command;
    bool should_stream;

    /*
     * 位置/速度模式走本地轨迹发生器。
     * 其他模式不做本地规划，但仍要维护状态字和 EMCY。
     */
    if (mode == OPMODE_PROFILE_POSITION || mode == OPMODE_PROFILE_VELOCITY) {
        TrapProfile_Update(&s_tp, &s_state);
    } else {
        /* 不受本地规划器支持的模式直接旁路，并清除运动相关锁存标志 */
        TrapProfile_ResetState(&s_tp, &s_state);
        s_tp.last_op_mode = mode;
        s_tp.target_reached = false;
        s_tp.setpoint_ack = false;
    }

    Generator_UpdateDemandObjects(mode);
    Generator_UpdateEmergencyState();
    Generator_WriteODU16IfChanged(0x6041, &TestSlave_obj6041, Generator_BuildStatusWord());

    command = Generator_BuildCommandFrame();
    should_stream = Generator_ShouldStreamCommand(&command);

    /* 只有在确实需要维持命令流时才发帧，减少总线占用。 */
    if (should_stream) {
        Driver_Send_CommandFrame(&command);
    }

}

/* 将解析后的 G4 反馈回写到 CANopen 主站期望的 OD 对象中。 */
void Generator_OnG4Feedback(uint8_t mode,  int32_t act_main)
{
    Generator_WriteODU8IfChanged(0x6061, &TestSlave_obj6061, (UNS8)mode);

    /*
     * G4 反馈帧里的主反馈字段 act_main 复用了同一组字节：
     * 位置模式解释成实际位置；
     * 其余模式解释成实际速度。
     */
    if (mode == OPMODE_PROFILE_POSITION) {
        /* 位置模式下，表示实际位置。 */
        Generator_WriteODU32IfChanged(0x6064, &TestSlave_obj6064, (UNS32)act_main);
    } else {
        /* 速度模式下，表示实际速度 */
        Generator_WriteODU32IfChanged(0x606C, &TestSlave_obj606C, (UNS32)act_main);
    }
}

/* 定时器中断把硬件时间基准累积成一个待处理节拍计数。 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        /* 饱和计数，避免主循环长时间阻塞时计数器继续溢出回绕。 */
        if (s_tick_pending < UINT8_MAX) {
            ++s_tick_pending;
        }
    }
}
