#include "torque.h"

#include "TestSlave.h"
#include "can_app.h"
#include "canfestival.h"
#include "emcy.h"
#include "g4_motor.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * torque.c
 *
 * Profile Torque 独立控制模块。
 *
 * 设计边界：
 * - generator.c 只负责位置/速度/回零相关的运动轨迹。
 * - 本文件独立负责转矩模式的对象字典读取、转矩限幅、转矩斜坡、状态字、
 *   G4 命令下发和 G4 反馈回写。
 * - 转矩相关对象按 DS402 语义使用有符号 INTEGER16，应用层直接读写 signed 值。
 */

extern CO_Data TestSlave_Data;
extern volatile uint8_t g_g4_fault_status;

/*
 * 这些对象字典变量由 CanFestival 生成。
 * 本模块只引用和读写现有对象，不改变对象字典文件和协议栈本身。
 */
extern UNS16 TestSlave_obj6040;
extern INTEGER16 TestSlave_obj605A;
extern INTEGER16 TestSlave_obj605B;
extern INTEGER16 TestSlave_obj605C;
extern INTEGER16 TestSlave_obj605D;
extern INTEGER16 TestSlave_obj605E;
extern INTEGER8 TestSlave_obj6060;
extern INTEGER8 TestSlave_obj6061;
extern INTEGER32 TestSlave_obj606C;
extern INTEGER16 TestSlave_obj6071;
extern UNS16 TestSlave_obj6072;
extern INTEGER16 TestSlave_obj6074;
extern INTEGER16 TestSlave_obj6077;
extern UNS32 TestSlave_obj6087;
extern INTEGER16 TestSlave_obj60E0;
extern INTEGER16 TestSlave_obj60E1;

/* Profile Torque 的 CiA402 模式值。 */
#define TORQUE_OPMODE_PROFILE_TORQUE        4

/*
 * 控制字中本模块关心的位。
 * bit2 属于 quick stop 组合命令；bit8 用于 halt。
 */
#define TORQUE_CW_HALT                      0x0100u

/*
 * 转矩斜坡内部使用 Q16 定点数。
 * 这样 0x6087=200 permille/s、1 ms 周期时的 0.2 permille 增量不会被截断为 0。
 */
#define TORQUE_Q16_SHIFT                    16
#define TORQUE_Q16_ONE                      (1L << TORQUE_Q16_SHIFT)
#define TORQUE_Q16_HALF                     (1L << (TORQUE_Q16_SHIFT - 1))

/*
 * 外部转矩单位为额定转矩千分比。
 * 1000 表示 100% 额定转矩；负值表示反向转矩。
 */
#define TORQUE_FULL_SCALE_PERMILLE          1000
#define TORQUE_DEFAULT_LIMIT_PERMILLE       1000
#define TORQUE_DEFAULT_SLOPE_PER_SEC        1000u
#define TORQUE_MAX_ABS_PERMILLE             32767
#define TORQUE_TARGET_WINDOW_PERMILLE       1
#define TORQUE_STOP_SPEED_WINDOW_RPM        1
#define TORQUE_OPTION_DISABLE               0

/* EMCY 上报码，与 generator.c 中已有故障语义保持一致。 */
#define TORQUE_EMCY_G4_TIMEOUT              0x8130u
#define TORQUE_EMCY_G4_DEVICE_FAULT         0x5000u

/*
 * 转矩模式的运行时状态。
 *
 * demand_q16:
 *   当前已经过斜坡处理的转矩需求值，单位为 permille 的 Q16 定点数
 *
 * target_reached:
 *   表示当前 demand 已经追上限幅后的 target，用于映射 0x6041 bit10
 */
typedef struct
{
    int64_t demand_q16;
    uint32_t cycle_us;
    bool target_reached;
} TorqueProfile;

static TorqueProfile s_torque;
static bool s_g4_timeout_emcy_active = false;
static bool s_g4_fault_emcy_active = false;

/*
 * Torque_AbsS32
 *
 * 函数作用：
 * 返回 int32_t 绝对值，并对最小负数做饱和保护
 */
static int32_t Torque_AbsS32(int32_t value)
{
    /*
     * 非负数直接返回，避免进入下面的最小负数特殊处理
     * 这里使用 int32_t 是为了后续比较 demand 与 target 的误差
     */
    if (value >= 0) {
        return value;
    }

    /*
     * int32_t 最小值 -2147483648 无法用同类型正数精确表示
     * 如果直接取反会溢出，因此饱和到 int32_t 最大值。
     */
    return (value == (int32_t)0x80000000L) ? (int32_t)0x7fffffffL : -value;
}

/*
 * Torque_ClampS32
 *
 * 函数作用：
 * 将 value 限制在 [min_value, max_value] 之间
 */
static int32_t Torque_ClampS32(int32_t value, int32_t min_value, int32_t max_value)
{
    /* 先处理下边界，保证调用者传入的目标不会低于允许负向转矩。 */
    if (value < min_value) {
        return min_value;
    }

    /* 再处理上边界，保证调用者传入的目标不会高于允许正向转矩。 */
    if (value > max_value) {
        return max_value;
    }

    /* 已在合法区间内时保持原值，避免引入额外量化误差。 */
    return value;
}

/*
 * Torque_I32ToQ16
 *
 * 函数作用：
 * 将整数转矩千分比转换为 Q16 定点格式
 */
static int64_t Torque_I32ToQ16(int32_t value)
{
    /*
     * 左移 16 位表示乘以 2^16
     * 结果放在 int64_t 中，给后续斜坡累加留下足够余量
     */
    return ((int64_t)value << TORQUE_Q16_SHIFT);
}

/*
 * Torque_Q16ToI16Round
 *
 * 函数作用：
 * 将 Q16 转矩需求值四舍五入成 int16_t 千分比
 *
 * 说明：
 * 正负数分别处理，避免 C 语言右移负数时的实现差异影响四舍五入语义
 */
static int16_t Torque_Q16ToI16Round(int64_t value_q16)
{
    int64_t rounded;

    if (value_q16 >= 0) {
        /*
         * 正数四舍五入：加 0.5 个 Q16 单位后再右移
         * 例如 1.6 permille 会被转换为 2 permille
         */
        rounded = (value_q16 + TORQUE_Q16_HALF) >> TORQUE_Q16_SHIFT;
    } else {
        /*
         * 负数先转成正数做同样的四舍五入，再恢复符号
         * 这样 -1.6 会得到 -2，而不是依赖编译器对负数右移的实现细节
         */
        rounded = -(((-value_q16) + TORQUE_Q16_HALF) >> TORQUE_Q16_SHIFT);
    }

    /* 对象字典和 G4 命令最终只承载 int16_t 转矩千分比，超出时做饱和 */
    if (rounded > 32767) {
        return 32767;
    }

    /* int16_t 负向边界是 -32768，单独判断避免强转后回绕 */
    if (rounded < -32768) {
        return -32768;
    }

    /* 正常路径：Q16 内部需求已经转换成外部可见的整数千分比 */
    return (int16_t)rounded;
}

/*
 * Torque_GetMaxAbsLimit
 *
 * 函数作用：
 * 读取 0x6072 Max torque，得到绝对转矩上限
 *
 * 处理规则：
 * - 0 表示未配置，采用 1000 permille 的安全默认值
 * - 超过 int16_t 正范围时饱和，避免后续有符号计算溢出
 */
static int32_t Torque_GetMaxAbsLimit(void)
{
    /*
     * 0x6072 是 DS402 Max torque，本工程按“额定转矩千分比”的无符号值使用
     * 后续限幅计算需要有符号边界，因此先提升到 int32_t
     */
    int32_t max_abs = (int32_t)TestSlave_obj6072;

    /*
     * 0 通常意味着主站没有配置该对象
     * 转矩控制不能在无限幅状态下运行，因此回退到 100% 额定转矩
     */
    if (max_abs <= 0) {
        max_abs = TORQUE_DEFAULT_LIMIT_PERMILLE;
    }

    /*
     * 目标转矩最终要写入 INTEGER16/G4 int16 语义
     * 即使对象字典给了更大的无符号值，也必须压到可表达的正向最大值
     */
    if (max_abs > TORQUE_MAX_ABS_PERMILLE) {
        max_abs = TORQUE_MAX_ABS_PERMILLE;
    }

    /* 返回正数绝对上限，调用者再按正负方向生成 signed limits */
    return max_abs;
}

/*
 * Torque_GetSignedLimits
 *
 * 函数作用：
 * 综合 0x6072、0x60E0 和 0x60E1 得到最终正负方向转矩限制
 *
 * 参数：
 * negative_limit : 输出，负方向最小允许转矩
 * positive_limit : 输出，正方向最大允许转矩
 *
 * 说明：
 * - 0x60E0 为正方向限制，<=0 时认为未配置，回退到 +0x6072
 * - 0x60E1 为负方向限制，>=0 时认为未配置，回退到 -0x6072
 * - 两个方向最终都被 0x6072 的绝对上限再次夹住。
 */
static void Torque_GetSignedLimits(int32_t *negative_limit, int32_t *positive_limit)
{
    int32_t max_abs = Torque_GetMaxAbsLimit();
    /*
     * 0x60E0/0x60E1 在这里作为正/负方向的软件限矩
     * 它们是有符号对象，允许主站分别收窄两个方向的可用转矩
     */
    int32_t pos_limit = (int32_t)TestSlave_obj60E0;
    int32_t neg_limit = (int32_t)TestSlave_obj60E1;

    /*
     * 正方向限制如果没有给出正值，就使用 0x6072 的全局绝对限矩
     * 这样旧主站即使只写 0x6072，也能得到对称限幅
     */
    if (pos_limit <= 0) {
        pos_limit = max_abs;
    }

    /*
     * 负方向限制如果没有给出负值，也回退到 -0x6072
     * 注意这里期望对象中保存的是负数，而不是“负向绝对值”
     */
    if (neg_limit >= 0) {
        neg_limit = -max_abs;
    }

    /*
     * 分方向限制不能突破 0x6072 这个总上限
     * 正向范围被压在 [0, +max_abs]，负向范围被压在 [-max_abs, 0]
     */
    pos_limit = Torque_ClampS32(pos_limit, 0, max_abs);
    neg_limit = Torque_ClampS32(neg_limit, -max_abs, 0);

    if (neg_limit > pos_limit) {
        /* 配置互相矛盾时回退为对称限制，避免出现不可解释的夹紧区间 */
        neg_limit = -max_abs;
        pos_limit = max_abs;
    }

    /* 通过输出参数返回最终可用于夹紧 0x6071 的闭区间 */
    *negative_limit = neg_limit;
    *positive_limit = pos_limit;
}

/*
 * Torque_ReadLimitedTarget
 *
 * 函数作用：
 * 读取 0x6071 目标转矩，并按 0x6072/0x60E0/0x60E1 做应用层限幅
 */
static int16_t Torque_ReadLimitedTarget(void)
{
    int32_t negative_limit;
    int32_t positive_limit;
    /*
     * 0x6071 Target torque 是主站给出的原始目标值
     * 它可能超出当前软件/额定限矩，因此不能直接用于斜坡和下发
     */
    int32_t target = (int32_t)TestSlave_obj6071;

    /* 先汇总所有限矩对象，得到允许的区间 */
    Torque_GetSignedLimits(&negative_limit, &positive_limit);

    /* 再把主站目标夹到安全区间内，后续斜坡只追踪这个受限目标 */
    target = Torque_ClampS32(target, negative_limit, positive_limit);

    /* 前面已经限制到 int16_t 可表达范围，这里强转不会回绕 */
    return (int16_t)target;
}

/*
 * Torque_GetSlopePermillePerSec
 *
 * 函数作用：
 * 读取 0x6087 Torque slope，单位约定为 permille/s
 *
 * 处理规则：
 * - 0 表示未配置，为安全起见采用 1000 permille/s 的默认斜坡
 * - 过大的配置做上限保护，避免计算步长时失去限流意义
 */
static uint32_t Torque_GetSlopePermillePerSec(void)
{
    /*
     * 0x6087 Profile torque slope
     * 单位约定为 permille/s，即每秒变化多少“额定转矩千分比”
     */
    uint32_t slope = (uint32_t)TestSlave_obj6087;

    /*
     * 主站不写斜坡时，使用 1000 permille/s
     * 这相当于从 0 到 100% 额定转矩约 1 秒，属于保守默认值
     */
    if (slope == 0u) {
        slope = TORQUE_DEFAULT_SLOPE_PER_SEC;
    }

    /*
     * 过大的斜坡会近似变成阶跃，削弱转矩限流意义
     * 这里设上限也避免 step_q16 中间乘法过度放大
     */
    if (slope > 1000000u) {
        slope = 1000000u;
    }

    /* 返回已经过默认值和上限保护的有效斜坡 */
    return slope;
}

/*
 * 函数作用：
 * 按 0x6087 斜坡把当前转矩需求逼近目标转矩
 *
 * 参数：
 * current_q16 : 当前转矩需求，Q16
 * target_q16  : 目标转矩需求，Q16
 * slope       : 转矩斜率，permille/s
 * cycle_us    : 本次周期长度，us
 *
 * 返回：
 * 更新后的 Q16 转矩需求
 */
static int64_t Torque_RampToTarget(int64_t current_q16,
                                   int64_t target_q16,
                                   uint32_t slope,
                                   uint32_t cycle_us)
{
    int64_t delta_q16 = target_q16 - current_q16;
    int64_t step_q16;

    /*
     * delta 为 0 表示当前内部需求已经等于目标，直接返回可以避免无意义计算
     * 也让 target_reached 的判断保持稳定
     */
    if (delta_q16 == 0) {
        return current_q16;
    }

    /*
     * 最大允许步长 step = slope(permille/s) * cycle(us) * Q16 / 1e6
     * 用 int64_t 计算，避免大斜率或长周期时中间结果溢出
     */
    step_q16 = ((int64_t)slope * (int64_t)cycle_us * TORQUE_Q16_ONE) / 1000000LL;

    if (step_q16 <= 0) {
        /*
         * 当斜率极小或周期极短时，整数除法可能会向下取整为0，导致 step_q16 为0
         * 此时当前值将永远无法逼近目标值（卡死） 强制将其设为 1
         * 保证了微小的斜率也能缓慢逼近目标
         */
        step_q16 = 1;
    }

    /*
     * 目标在当前值上方且距离大于本周期最大步长时，只前进一个 step
     * 这样限制的是“每周期最大变化量”，而不是目标本身
     */
    if (delta_q16 > step_q16) {
        return current_q16 + step_q16;
    }

    /*
     * 目标在当前值下方时同理反向前进一个 step
     * 使用 -step_q16 可以保持上升/下降斜率对称
     */
    if (delta_q16 < -step_q16) {
        return current_q16 - step_q16;
    }

    /*
     * 剩余距离已经小于等于一个周期步长，直接吸附到目标
     * 这可以避免在目标附近来回量化抖动
     */
    return target_q16;
}

/*
 * 函数作用：
 * 从应用层安全写回 int8 对象，避免无变化时反复触发对象字典写路径
 */
static void Torque_WriteODI8IfChanged(UNS16 index, INTEGER8 *object, INTEGER8 value)
{
    /*
     * 对象字典当前值已经一致时不写入
     * 这样可以减少 CanFestival 本地字典写路径和可能的回调开销
     */
    if (*object != value) {
        /*
         * writeLocalDict 需要传入数据长度指针
         * 真正写入仍走协议栈接口，而不是直接改生成变量，保持 OD 访问语义一致
         */
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

/*
 * 函数作用：
 * 从应用层安全写回 int16 对象，例如 0x6074 Torque demand
 */
static void Torque_WriteODI16IfChanged(UNS16 index, INTEGER16 *object, INTEGER16 value)
{
    /* 转矩 demand/actual 等 int16 对象按变化写回，降低周期任务负担 */
    if (*object != value) {
        /* 这里不直接解引用赋值，是为了复用 CanFestival 的本地写入路径 */
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

/*
 * 函数作用：
 * 从应用层安全写回 int32 对象
 */
static void Torque_WriteODI32IfChanged(UNS16 index, INTEGER32 *object, INTEGER32 value)
{
    /* 速度反馈等 int32 对象同样只在数值变化时写入 OD */
    if (*object != value) {
        /* size 使用实际变量类型大小，避免平台类型宽度变化时写错长度 */
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

/*
 * Torque_UpdateEmergencyState
 *
 * 函数作用：
 * 在转矩模式运行期间维护 G4 通信超时和 G4 设备故障 EMCY
 *
 * 说明：
 * 位置/速度模式下同类故障由 generator.c 维护；转矩模式不再进入 generator.c
 * 因此这里必须独立完成故障上报和恢复
 */
static void Torque_UpdateEmergencyState(void)
{
    /*
     * 将底层全局故障标志转成 bool，只关心当前是否处于该故障
     * 具体错误码和恢复边沿在本函数内统一管理。
     */
    bool timeout_fault = (g_g4_comm_timeout != 0u);
    bool g4_fault = (g_g4_fault_status != 0u);

    /*
     * 通信超时从无到有时上报 EMCY
     * s_g4_timeout_emcy_active 用来记住已经上报过，避免每 1 ms 重复发同一个 EMCY
     */
    if (timeout_fault && !s_g4_timeout_emcy_active) {
        EMCY_setError(&TestSlave_Data, TORQUE_EMCY_G4_TIMEOUT, 0x10u, 0u);
        s_g4_timeout_emcy_active = true;
    } else if (!timeout_fault && s_g4_timeout_emcy_active) {
        /*
         * 通信恢复后发 recovered，并清除本地活动标志
         * 这样下一次再次超时时还能重新上报
         */
        EMCY_errorRecovered(&TestSlave_Data, TORQUE_EMCY_G4_TIMEOUT);
        s_g4_timeout_emcy_active = false;
    }

    /*
     * G4 设备故障与通信超时独立维护
     * 两类故障可能同时存在，因此不能用 if/else 把第二类跳过
     */
    if (g4_fault && !s_g4_fault_emcy_active) {
        EMCY_setError(&TestSlave_Data, TORQUE_EMCY_G4_DEVICE_FAULT, 0x04u, 0u);
        s_g4_fault_emcy_active = true;
    } else if (!g4_fault && s_g4_fault_emcy_active) {
        /* 设备故障解除时通知协议栈清除对应 EMCY */
        EMCY_errorRecovered(&TestSlave_Data, TORQUE_EMCY_G4_DEVICE_FAULT);
        s_g4_fault_emcy_active = false;
    }
}

/*
 * Torque_IsStopDoneForDs402
 *
 * 函数作用：
 * 判断转矩模式是否已经满足 DS402 quick stop/fault reaction 的停稳条件。
 *
 * 返回：
 * true 表示内部转矩需求已经斜坡到 0，且实际速度接近 0 或 G4 反馈不可用。
 */
static bool Torque_IsStopDoneForDs402(void)
{
    bool demand_zero = (s_torque.demand_q16 == 0);
    bool actual_speed_zero =
        (g_g4_feedback_online == 0u) ||
        (Torque_AbsS32((int32_t)TestSlave_obj606C) <= TORQUE_STOP_SPEED_WINDOW_RPM);

    /*
     * 文档要求转矩模式优先结合实际速度。
     * 通信超时时实际速度无法继续更新，所以离线情况下只用 demand 回零作为完成条件。
     */
    return demand_zero && actual_speed_zero;
}

/*
 * Torque_FillDs402Input
 *
 * 函数作用：
 * 为统一 DS402 状态机准备转矩模式下的周期输入。
 *
 * 参数：
 * input : 调用者提供的 DS402_Input 输出结构。
 */
void Torque_FillDs402Input(DS402_Input *input)
{
    bool hard_fault = (g_g4_fault_status != 0u) || (g_g4_comm_timeout != 0u);
    bool stop_done = Torque_IsStopDoneForDs402();

    input->controlword = TestSlave_obj6040;
    input->startup_ready = true;
    input->nmt_operational = (TestSlave_Data.nodeState == Operational);
    input->drive_online = (g_g4_feedback_online != 0u);
    input->local_fault = hard_fault;
    input->fault_reset_allowed = !hard_fault;
    input->fault_reaction_done = stop_done;
    input->quick_stop_done = stop_done;
}

/*
 * Torque_FillDs402StatusBits
 *
 * 函数作用：
 * 给统一状态字生成器补充转矩模式相关位。
 *
 * 参数：
 * bits : 已经由 Ds402_InitStatusBits() 初始化的状态位结构。
 */
void Torque_FillDs402StatusBits(DS402_StatusBits *bits)
{
    /* 转矩模式只使用 bit10 target reached，其余 bit12/bit13 不叠加。 */
    bits->target_reached = s_torque.target_reached;
}

/*
 * Torque_OnDs402FaultReset
 *
 * 函数作用：
 * DS402 接受 fault reset 上升沿后，复位转矩软状态。
 */
void Torque_OnDs402FaultReset(void)
{
    /* fault reset 后从 0 转矩重新开始，避免恢复后继承故障前的需求。 */
    Torque_Reset();
}

/*
 * Torque_BuildCommandFrame
 *
 * 函数作用：
 * 根据当前安全状态和转矩需求生成一帧发往 G4/control 的私有命令。
 *
 * 参数：
 * command      : 输出命令帧。
 * enabled               : 是否正常使能。
 * quick_stop_active     : 是否处于 quick stop active。
 * fault_reaction_active : 是否处于 fault reaction active。
 * fault_active          : 是否处于 fault 锁存。
 * ds402_state           : 当前 DS402 基础状态，用于解释 shutdown/disable operation option code。
 * halt_active           : 是否执行 halt。
 * demand                : 已经过限幅和斜坡处理的转矩需求，单位 permille。
 */
static void Torque_BuildCommandFrame(G4_CommandFrame *command,
                                     bool enabled,
                                     bool quick_stop_active,
                                     bool fault_reaction_active,
                                     bool fault_active,
                                     DS402_State ds402_state,
                                     bool halt_active,
                                     int16_t demand)
{
    /*
     * 先把命令帧初始化成最安全的 disable/零目标。
     * 后续只有在明确满足条件时才提升为 quickstop/halt/enable。
     */
    command->cmd_type = CMD_TYPE_DISABLE;
    command->mode = TORQUE_OPMODE_PROFILE_TORQUE;
    command->control_flags = 0u;
    command->homing_method = 0;
    command->target_value = 0;

    /*
     * DS402 停机优先级：
     * - Fault 锁存后直接 disable。
     * - Fault reaction 按 0x605E 选择 quick stop 或 disable。
     * - Quick stop 按 0x605A 选择 quick stop 或 disable。
     * - Halt 按 0x605D 选择 halt 或 disable。
     */
    if (fault_active) {
        command->cmd_type = CMD_TYPE_DISABLE;
    } else if (fault_reaction_active) {
        command->cmd_type = (TestSlave_obj605E == TORQUE_OPTION_DISABLE) ?
            CMD_TYPE_DISABLE : CMD_TYPE_QUICKSTOP;
    } else if (quick_stop_active) {
        command->cmd_type = (TestSlave_obj605A == TORQUE_OPTION_DISABLE) ?
            CMD_TYPE_DISABLE : CMD_TYPE_QUICKSTOP;
    } else if (halt_active) {
        command->cmd_type = (TestSlave_obj605D == TORQUE_OPTION_DISABLE) ?
            CMD_TYPE_DISABLE : CMD_TYPE_HALT;
    } else if (ds402_state == DS402_STATE_SWITCHED_ON) {
        command->cmd_type = (TestSlave_obj605C == TORQUE_OPTION_DISABLE) ?
            CMD_TYPE_DISABLE : CMD_TYPE_HALT;
    } else if (ds402_state == DS402_STATE_READY_TO_SWITCH_ON) {
        command->cmd_type = (TestSlave_obj605B == TORQUE_OPTION_DISABLE) ?
            CMD_TYPE_DISABLE : CMD_TYPE_HALT;
    } else if (enabled) {
        /*
         * 只有完全使能且没有 halt/qstop 时才下发转矩需求。
         * demand 已经完成限幅和斜坡处理，可以直接作为 mode=4 target_value。
         */
        command->cmd_type = CMD_TYPE_ENABLE;
        command->target_value = (int32_t)demand;
    }
}

/*
 * Torque_Init
 *
 * 函数作用：
 * 初始化转矩模式状态。
 */
void Torque_Init(uint32_t cycle_us)
{
    /*
     * 上层理论上传入 1000 us。
     * 如果传入 0，使用 1000 us 防止斜坡步长永远为 0。
     */
    s_torque.cycle_us = (cycle_us == 0u) ? 1000u : cycle_us;

    /*
     * 初始化时也复位转矩斜坡和 0x6074，保证上电/重新初始化后没有残留需求。
     */
    Torque_Reset();

    /*
     * EMCY 活动标志只表示“本模块是否已经上报过对应故障”。
     * 初始化时清零，后续 Torque_UpdateEmergencyState 会按当前硬件状态重新同步。
     */
    s_g4_timeout_emcy_active = false;
    s_g4_fault_emcy_active = false;
}

/*
 * Torque_Reset
 *
 * 函数作用：
 * 清空转矩需求斜坡。
 */
void Torque_Reset(void)
{
    /*
     * 内部斜坡状态清零。
     * 使用 Q16 的 0，等价于 0 permille 转矩需求。
     */
    s_torque.demand_q16 = 0;

    /*
     * 刚复位后不声明 target reached。
     * 下一次 Torque_Run 会基于当时的目标重新计算 bit10。
     */
    s_torque.target_reached = false;

    /*
     * 0x6074 是主站可见的 Torque demand。
     * 离开/重置转矩模式时立即写 0，避免主站看到旧的内部需求。
     */
    Torque_WriteODI16IfChanged(0x6074, &TestSlave_obj6074, 0);
}

/*
 * Torque_Run
 *
 * 函数作用：
 * Profile Torque 模式 1 ms 周期任务。
 */
void Torque_Run(void)
{
    /*
     * 读取本周期要用到的对象字典和协议栈状态。
     * 局部快照可以避免同一周期内多次读取到不同值导致判断不一致。
     */
    uint16_t cw = TestSlave_obj6040;
    int8_t mode = (int8_t)TestSlave_obj6060;
    bool enabled;
    bool quick_stop_active;
    bool fault_reaction_active;
    bool fault_active;
    DS402_State ds402_state;
    bool stop_active;
    bool halt_active;
    int16_t limited_target = 0;
    int16_t demand = 0;
    G4_CommandFrame command;

    if (mode != TORQUE_OPMODE_PROFILE_TORQUE) {
        /*
         * main.c 正常不会在非转矩模式调用本函数。
         * 这里保留防御路径，避免调度错误时继续保持旧转矩需求。
         */
        Torque_Reset();
        return;
    }

    /*
     * DS402 状态机统一决定能否运行以及是否需要停机。
     * 转矩模块只根据状态机结果推进转矩斜坡和选择 G4 命令。
     */
    enabled = Ds402_IsOperationEnabled();
    quick_stop_active = Ds402_IsQuickStopActive();
    fault_reaction_active = Ds402_IsFaultReactionActive();
    fault_active = Ds402_IsFault();
    ds402_state = Ds402_GetState();
    stop_active = quick_stop_active || fault_reaction_active || fault_active;

    /*
     * halt 只在正常使能时生效。
     * 未使能时即使 bit8 置位，也仍按 disable 处理。
     */
    halt_active = enabled && ((cw & TORQUE_CW_HALT) != 0u);

    if (enabled && !stop_active && !halt_active) {
        /* 只有正常使能时才读取主站目标并推进转矩斜坡。 */
        limited_target = Torque_ReadLimitedTarget();
    } else {
        /*
         * 失能、halt、quick stop 或故障时，转矩需求都向 0 收敛。
         * 失能路径稍后会立即清零，halt/qstop 则保留斜坡状态用于 OD 观察。
         */
        limited_target = 0;
    }

    /*
     * 如果只是普通失能，没有 quick stop/halt/fault stop，则立即清零。
     * 这是“去使能”语义：不保留任何旧转矩，也不等待斜坡慢慢归零。
     */
    if (!enabled && !stop_active && !halt_active) {
        /*
         * 普通失能不需要慢慢斜坡，直接把需求清零并发送 disable。
         * 这样不会在重新使能时保留旧转矩。
         */
        s_torque.demand_q16 = 0;
    } else {
        /*
         * 使能、halt、quick stop 或故障停机时都走斜坡。
         * 正常使能时斜坡追踪受限目标；停机类状态下目标为 0，实现受控回零。
         */
        s_torque.demand_q16 = Torque_RampToTarget(s_torque.demand_q16,
                                                  Torque_I32ToQ16(limited_target),
                                                  Torque_GetSlopePermillePerSec(),
                                                  s_torque.cycle_us);
    }

    /*
     * 将内部 Q16 demand 转回 INTEGER16，供对象字典和 G4 命令使用。
     * 四舍五入可以减少小斜坡下长期向 0 偏置的问题。
     */
    demand = Torque_Q16ToI16Round(s_torque.demand_q16);

    /*
     * target_reached 允许 1 permille 窗口。
     * 这样 Q16 到整数的量化误差不会导致 0x6041 bit10 在目标附近反复变化。
     */
    s_torque.target_reached =
        (Torque_AbsS32((int32_t)demand - (int32_t)limited_target) <=
         TORQUE_TARGET_WINDOW_PERMILLE);

    /* 0x6074 反映已经过限幅和斜坡处理的内部转矩需求值。 */
    Torque_WriteODI16IfChanged(0x6074, &TestSlave_obj6074, (INTEGER16)demand);

    /*
     * 先同步 EMCY。
     * 0x6041 已改为由 main.c 调用 DS402 状态机统一发布，本模块不再单独拼状态字。
     */
    Torque_UpdateEmergencyState();

    /*
     * 按最终安全状态构造 G4 私有命令帧。
     * command 中的目标值使用已经写入 0x6074 的同一个 demand，保证主站可见值和下发值一致。
     */
    Torque_BuildCommandFrame(&command,
                             enabled,
                             quick_stop_active,
                             fault_reaction_active,
                             fault_active,
                             ds402_state,
                             halt_active,
                             demand);

    /*
     * 转矩模式也按 1 ms 持续下发命令。
     * G4 侧有 50 ms 命令超时保护，持续流式下发可以避免旧转矩被保持。
     */
    Driver_Send_CommandFrame(&command);
}

/*
 * Torque_OnG4Feedback
 *
 * 函数作用：
 * 转矩模式下处理 G4/control 反馈。
 */
void Torque_OnG4Feedback(uint8_t mode, int16_t act_torque, int32_t act_main)
{
    /* 模式显示仍然以 G4 回显为准，便于主站看到下游实际执行模式。 */
    Torque_WriteODI8IfChanged(0x6061, &TestSlave_obj6061, (INTEGER8)mode);

    /*
     * G4 返回的 act_torque 已经是额定转矩千分比，不是电流 A。
     * 直接写入 0x6077 Actual torque，供主站按 DS402 转矩对象读取。
     */
    Torque_WriteODI16IfChanged(0x6077,
                               &TestSlave_obj6077,
                               (INTEGER16)act_torque);

    /*
     * 转矩模式下反馈主字段 act_main 解释为实际机械速度 rpm。
     * 写入 0x606C Velocity actual value，方便主站在转矩模式下同时观察转速。
     */
    Torque_WriteODI32IfChanged(0x606C, &TestSlave_obj606C, (INTEGER32)act_main);
}
