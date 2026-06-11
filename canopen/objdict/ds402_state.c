#include "ds402_state.h"

#include "TestSlave.h"
#include "canfestival.h"

/*
 * DS402 状态机模块。
 *
 * 设计边界：
 * - 本文件只解释 0x6040、维护 PDS 状态、生成 0x6041 基础状态位。
 * - 不直接计算位置、速度、转矩轨迹，也不直接发送 G4 私有命令。
 * - main.c 每 1 ms 调用一次 Ds402_Update()，generator.c/torque.c 只查询状态。
 */

extern CO_Data TestSlave_Data;
extern UNS16 TestSlave_obj6041;

/*
 * 当前没有真实母线预充、电源检测和驱动自检信号。
 * 这里保留一个最小 Not ready 驻留窗口，让主站上电后能观察到标准初态。
 */
#define DS402_MIN_NOT_READY_TICKS      2u

/* 0x6041 基础状态位。 */
#define DS402_SW_NOT_READY             0x0000u
#define DS402_SW_SWITCH_DISABLED       0x0040u
#define DS402_SW_READY                 0x0021u
#define DS402_SW_SWITCHED_ON           0x0023u
#define DS402_SW_OPERATION_ENABLED     0x0027u
#define DS402_SW_QUICK_STOP_ACTIVE     0x0007u
#define DS402_SW_FAULT_REACTION        0x000Fu
#define DS402_SW_FAULT                 0x0008u

/* 0x6041 公共叠加位。 */
#define DS402_SW_VOLTAGE_ENABLED       0x0010u
#define DS402_SW_QUICK_STOP_BIT        0x0020u
#define DS402_SW_REMOTE                0x0200u
#define DS402_SW_TARGET_REACHED        0x0400u
#define DS402_SW_SETPOINT_ACK          0x1000u
#define DS402_SW_FOLLOWING_ERROR       0x2000u

static DS402_State s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
static uint16_t s_prev_controlword = 0u;
static uint16_t s_not_ready_ticks = 0u;
static bool s_last_nmt_operational = false;
static bool s_last_drive_online = false;
static bool s_fault_reset_accepted = false;

/*
 * Ds402_CwShutdown
 *
 * 函数作用：
 * 判断控制字是否为 DS402 shutdown 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示命令匹配 shutdown，典型值为 0x0006。
 */
static bool Ds402_CwShutdown(uint16_t cw)
{
    /* 使用标准 mask 识别命令，允许 bit4/bit5/bit6/bit8 等模式位同时存在。 */
    return ((cw & 0x0087u) == 0x0006u);
}

/*
 * Ds402_CwSwitchOn
 *
 * 函数作用：
 * 判断控制字是否为 DS402 switch on 或 disable operation 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示命令匹配 switch on，典型值为 0x0007。
 */
static bool Ds402_CwSwitchOn(uint16_t cw)
{
    /* 0x0007 在 READY 状态表示 switch on，在 OPERATION_ENABLED 状态表示 disable operation。 */
    return ((cw & 0x008Fu) == 0x0007u);
}

/*
 * Ds402_CwEnableOperation
 *
 * 函数作用：
 * 判断控制字是否为 DS402 enable operation 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示命令匹配 enable operation，典型值为 0x000F。
 */
static bool Ds402_CwEnableOperation(uint16_t cw)
{
    /* enable operation 只比较低位命令组合，不要求整个控制字完全等于 0x000F。 */
    return ((cw & 0x008Fu) == 0x000Fu);
}

/*
 * Ds402_CwDisableVoltage
 *
 * 函数作用：
 * 判断控制字是否为 DS402 disable voltage 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示主站要求撤销电压使能，典型低位组合为 0。
 */
static bool Ds402_CwDisableVoltage(uint16_t cw)
{
    /* bit1 与 bit7 共同参与识别，避免 fault reset 位干扰普通 disable voltage。 */
    return ((cw & 0x0082u) == 0x0000u);
}

/*
 * Ds402_CwQuickStop
 *
 * 函数作用：
 * 判断控制字是否为 DS402 quick stop 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示主站清 bit2 发起 quick stop，典型值为 0x0002。
 */
static bool Ds402_CwQuickStop(uint16_t cw)
{
    /* quick stop 依赖 bit2 清零，不能用低 4 位全等判断。 */
    return ((cw & 0x0086u) == 0x0002u);
}

/*
 * Ds402_CwDisableOperation
 *
 * 函数作用：
 * 判断控制字是否为 DS402 disable operation 命令。
 *
 * 参数：
 * cw : 当前 0x6040 Controlword。
 *
 * 返回：
 * true 表示从 Operation enabled 回到 Switched on，典型值为 0x0007。
 */
static bool Ds402_CwDisableOperation(uint16_t cw)
{
    /* 与 switch on 命令同一低位组合，实际含义由当前状态决定。 */
    return ((cw & 0x008Fu) == 0x0007u);
}

/*
 * Ds402_CwFaultResetRising
 *
 * 函数作用：
 * 判断 0x6040 bit7 是否产生 fault reset 上升沿。
 *
 * 参数：
 * cw      : 当前控制字。
 * prev_cw : 上一拍控制字。
 *
 * 返回：
 * true 表示 bit7 从 0 变成 1。
 */
static bool Ds402_CwFaultResetRising(uint16_t cw, uint16_t prev_cw)
{
    /*
     * 故障复位必须使用上升沿。
     * 如果主站一直保持 bit7=1，这里只在第一次置位时返回 true。
     */
    return (((cw & 0x0080u) != 0u) && ((prev_cw & 0x0080u) == 0u));
}

/*
 * Ds402_RecordCommonState
 *
 * 函数作用：
 * 保存最近一次公共输入条件，供状态字公共位生成使用。
 *
 * 参数：
 * in : 当前 1 ms 周期输入。
 */
static void Ds402_RecordCommonState(const DS402_Input *in)
{
    /* remote 对应 CANopen Operational；voltage_enabled 以 G4 在线作为当前近似条件。 */
    s_last_nmt_operational = in->nmt_operational;
    s_last_drive_online = in->drive_online;
}

/*
 * Ds402_GetEffectiveStartupReady
 *
 * 函数作用：
 * 在上层 startup_ready 的基础上，叠加软件最小 Not ready 驻留时间。
 *
 * 参数：
 * startup_ready : 上层软件初始化是否已经完成。
 *
 * 返回：
 * true 表示状态机允许从 NOT_READY_TO_SWITCH_ON 自动进入 SWITCH_ON_DISABLED。
 */
static bool Ds402_GetEffectiveStartupReady(bool startup_ready)
{
    /*
     * 上层条件未完成时计数器清零。
     * 这样如果后续增加真实自检信号，任一自检撤销都会重新进入 Not ready 窗口。
     */
    if (!startup_ready) {
        s_not_ready_ticks = 0u;
        return false;
    }

    /* 每次 Ds402_Update() 只加一次，保证该窗口和 1 ms 调度节拍一致。 */
    if (s_not_ready_ticks < DS402_MIN_NOT_READY_TICKS) {
        ++s_not_ready_ticks;
        return false;
    }

    return true;
}

/*
 * Ds402_GetBaseStatusword
 *
 * 函数作用：
 * 根据当前 PDS 状态返回基础 0x6041 位。
 *
 * 返回：
 * 只包含 DS402 基础状态位的 statusword。
 */
static uint16_t Ds402_GetBaseStatusword(void)
{
    switch (s_state) {
        case DS402_STATE_NOT_READY_TO_SWITCH_ON:
            return DS402_SW_NOT_READY;

        case DS402_STATE_SWITCH_ON_DISABLED:
            return DS402_SW_SWITCH_DISABLED;

        case DS402_STATE_READY_TO_SWITCH_ON:
            return DS402_SW_READY;

        case DS402_STATE_SWITCHED_ON:
            return DS402_SW_SWITCHED_ON;

        case DS402_STATE_OPERATION_ENABLED:
            return DS402_SW_OPERATION_ENABLED;

        case DS402_STATE_QUICK_STOP_ACTIVE:
            return DS402_SW_QUICK_STOP_ACTIVE;

        case DS402_STATE_FAULT_REACTION_ACTIVE:
            return DS402_SW_FAULT_REACTION;

        case DS402_STATE_FAULT:
            return DS402_SW_FAULT;

        default:
            return DS402_SW_NOT_READY;
    }
}

/*
 * Ds402_WriteODU16IfChanged
 *
 * 函数作用：
 * 从从站内部安全写回 uint16 对象字典对象。
 *
 * 参数：
 * index  : 对象索引。
 * object : CanFestival 生成的对象变量地址。
 * value  : 需要写入的新值。
 */
static void Ds402_WriteODU16IfChanged(UNS16 index, UNS16 *object, UNS16 value)
{
    /* 状态字只有变化时才写入，减少 1 ms 周期内的对象字典写路径开销。 */
    if (*object != value) {
        UNS32 size = sizeof(value);
        (void)writeLocalDict(&TestSlave_Data, index, 0x00, &value, &size, 0);
    }
}

/*
 * Ds402_Init
 *
 * 函数作用：
 * 初始化 DS402 状态机内部状态。
 *
 * 说明：
 * 该函数只在上电启动流程调用一次；模式切换、轨迹复位、转矩复位都不能调用它，
 * 否则会破坏 DS402 故障锁存和标准状态跳转。
 */
void Ds402_Init(void)
{
    /* 按文档要求，上电初态固定为 Not ready to switch on。 */
    s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
    s_prev_controlword = 0u;
    s_not_ready_ticks = 0u;
    s_last_nmt_operational = false;
    s_last_drive_online = false;
    s_fault_reset_accepted = false;
}

/*
 * Ds402_Update
 *
 * 函数作用：
 * 执行一次 DS402 PDS 状态机更新。
 *
 * 参数：
 * in : 当前 1 ms 周期状态机输入。
 *
 * 说明：
 * - 故障优先级高于 drive_online 判定，通信超时必须能够进入 fault reaction。
 * - fault reset 使用 bit7 上升沿，保持 bit7 不会重复复位。
 * - NOT_READY_TO_SWITCH_ON 只在上电/重新初始化流程出现，正常 fault reset 回到 SWITCH_ON_DISABLED。
 */
void Ds402_Update(const DS402_Input *in)
{
    uint16_t cw = in->controlword;
    uint16_t prev_cw = s_prev_controlword;
    bool fault_reset = Ds402_CwFaultResetRising(cw, prev_cw);
    bool startup_ready = Ds402_GetEffectiveStartupReady(in->startup_ready);
    bool unavailable = (!in->nmt_operational || !in->drive_online);

    Ds402_RecordCommonState(in);
    s_fault_reset_accepted = false;

    /*
     * 本地故障优先处理。
     * G4 通信超时时 drive_online 往往已经是 false，如果先按 unavailable 早退，
     * 通信超时就无法进入 FAULT_REACTION_ACTIVE。
     */
    if (in->local_fault &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_FAULT_REACTION_ACTIVE;
    }

    /*
     * 上电初始化未完成时保持 Not ready。
     * 该分支不覆盖故障反应和故障锁存，避免故障被启动条件掩盖。
     */
    if (!startup_ready &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
        s_prev_controlword = cw;
        return;
    }

    /*
     * CANopen 未 Operational 或 G4 未在线时，只能停留在 Switch on disabled。
     * 故障态不受该分支影响，由 fault reset 标准流程恢复。
     */
    if (unavailable &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_SWITCH_ON_DISABLED;
        s_prev_controlword = cw;
        return;
    }

    switch (s_state) {
        case DS402_STATE_NOT_READY_TO_SWITCH_ON:
            if (startup_ready) {
                /* 软件启动窗口结束后，自动进入 Switch on disabled。 */
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            }
            break;

        case DS402_STATE_SWITCH_ON_DISABLED:
            if (Ds402_CwShutdown(cw)) {
                /* shutdown 命令使设备准备好进入开机流程。 */
                s_state = DS402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case DS402_STATE_READY_TO_SWITCH_ON:
            if (Ds402_CwDisableVoltage(cw)) {
                /* 主站撤销电压请求时回到 Switch on disabled。 */
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (Ds402_CwSwitchOn(cw)) {
                /* 标准三步使能中的第二步。 */
                s_state = DS402_STATE_SWITCHED_ON;
            } else if (Ds402_CwEnableOperation(cw)) {
                /* 允许主站合并 switch on + enable operation。 */
                s_state = DS402_STATE_OPERATION_ENABLED;
            }
            break;

        case DS402_STATE_SWITCHED_ON:
            if (Ds402_CwDisableVoltage(cw)) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (Ds402_CwShutdown(cw)) {
                s_state = DS402_STATE_READY_TO_SWITCH_ON;
            } else if (Ds402_CwEnableOperation(cw)) {
                s_state = DS402_STATE_OPERATION_ENABLED;
            }
            break;

        case DS402_STATE_OPERATION_ENABLED:
            if (Ds402_CwDisableVoltage(cw)) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (Ds402_CwShutdown(cw)) {
                s_state = DS402_STATE_READY_TO_SWITCH_ON;
            } else if (Ds402_CwDisableOperation(cw)) {
                s_state = DS402_STATE_SWITCHED_ON;
            } else if (Ds402_CwQuickStop(cw)) {
                s_state = DS402_STATE_QUICK_STOP_ACTIVE;
            }
            break;

        case DS402_STATE_QUICK_STOP_ACTIVE:
            if (in->local_fault) {
                /* 急停过程中出现本地故障，升级到故障反应。 */
                s_state = DS402_STATE_FAULT_REACTION_ACTIVE;
            } else if (in->quick_stop_done && Ds402_CwDisableVoltage(cw)) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (in->quick_stop_done && Ds402_CwEnableOperation(cw)) {
                s_state = DS402_STATE_OPERATION_ENABLED;
            }
            break;

        case DS402_STATE_FAULT_REACTION_ACTIVE:
            if (in->fault_reaction_done) {
                /* 故障反应完成后锁存到 Fault，等待主站按上升沿复位。 */
                s_state = DS402_STATE_FAULT;
            }
            break;

        case DS402_STATE_FAULT:
            if (fault_reset && in->fault_reset_allowed) {
                /*
                 * G4 硬故障/通信超时等源头消失后，才接受 bit7 上升沿。
                 * 接受标志供运动模块清除跟随误差这类软件锁存。
                 */
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
                s_fault_reset_accepted = true;
            }
            break;

        default:
            /* 防御异常枚举值，重新回到标准上电初态。 */
            s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
            s_not_ready_ticks = 0u;
            break;
    }

    s_prev_controlword = cw;
}

/*
 * Ds402_GetState
 *
 * 函数作用：
 * 返回当前 DS402 PDS 状态。
 */
DS402_State Ds402_GetState(void)
{
    return s_state;
}

/*
 * Ds402_InitStatusBits
 *
 * 函数作用：
 * 初始化状态字叠加位结构，并填入公共 bit4/bit9 条件。
 *
 * 参数：
 * bits : 待初始化的状态位结构。
 */
void Ds402_InitStatusBits(DS402_StatusBits *bits)
{
    bits->voltage_enabled = s_last_drive_online;
    bits->remote = s_last_nmt_operational;
    bits->target_reached = false;
    bits->setpoint_ack = false;
    bits->following_error = false;
}

/*
 * Ds402_BuildStatusword
 *
 * 函数作用：
 * 按当前 DS402 状态和调用方传入的模式相关位生成 0x6041。
 *
 * 参数：
 * bits : 公共位和模式相关位；可由 Ds402_InitStatusBits() 初始化后再补充。
 *
 * 返回：
 * 最终写入 0x6041 的状态字。
 */
uint16_t Ds402_BuildStatusword(const DS402_StatusBits *bits)
{
    uint16_t status = Ds402_GetBaseStatusword();

    /*
     * bit4 表示电压已使能。
     * 当前工程没有独立母线检测，因此以 G4 反馈在线作为允许驱动电源的近似条件。
     */
    if (bits->voltage_enabled) {
        status |= DS402_SW_VOLTAGE_ENABLED;
    }

    /*
     * bit5 在 DS402 中是“Quick stop 未激活”。
     * 因此除了 Quick stop active 这个状态外，其余状态都置 1。
     */
    if (s_state != DS402_STATE_QUICK_STOP_ACTIVE) {
        status |= DS402_SW_QUICK_STOP_BIT;
    } else {
        status &= (uint16_t)(~DS402_SW_QUICK_STOP_BIT);
    }

    /* bit9 remote：节点进入 Operational 后认为远程控制有效。 */
    if (bits->remote) {
        status |= DS402_SW_REMOTE;
    }

    /* bit10 target reached：由位置/速度/转矩模块按各自语义提供。 */
    if (bits->target_reached) {
        status |= DS402_SW_TARGET_REACHED;
    }

    /* bit12 set-point acknowledge：当前只由位置模式使用。 */
    if (bits->setpoint_ack) {
        status |= DS402_SW_SETPOINT_ACK;
    }

    /* bit13 following error：当前只由位置模式使用。 */
    if (bits->following_error) {
        status |= DS402_SW_FOLLOWING_ERROR;
    }

    return status;
}

/*
 * Ds402_PublishStatusword
 *
 * 函数作用：
 * 生成并写回 0x6041。
 *
 * 参数：
 * bits : 模式相关状态位。
 */
void Ds402_PublishStatusword(const DS402_StatusBits *bits)
{
    Ds402_WriteODU16IfChanged(0x6041, &TestSlave_obj6041, Ds402_BuildStatusword(bits));
}

/*
 * Ds402_IsOperationEnabled
 *
 * 函数作用：
 * 查询当前是否处于 Operation enabled。
 */
bool Ds402_IsOperationEnabled(void)
{
    return (s_state == DS402_STATE_OPERATION_ENABLED);
}

/*
 * Ds402_IsQuickStopActive
 *
 * 函数作用：
 * 查询当前是否处于 Quick stop active。
 */
bool Ds402_IsQuickStopActive(void)
{
    return (s_state == DS402_STATE_QUICK_STOP_ACTIVE);
}

/*
 * Ds402_IsFaultReactionActive
 *
 * 函数作用：
 * 查询当前是否处于 Fault reaction active。
 */
bool Ds402_IsFaultReactionActive(void)
{
    return (s_state == DS402_STATE_FAULT_REACTION_ACTIVE);
}

/*
 * Ds402_IsFault
 *
 * 函数作用：
 * 查询当前是否处于 Fault。
 */
bool Ds402_IsFault(void)
{
    return (s_state == DS402_STATE_FAULT);
}

/*
 * Ds402_ConsumeFaultResetAccepted
 *
 * 函数作用：
 * 读取并清除“本周期已接受 fault reset 上升沿”的一次性标志。
 *
 * 返回：
 * true 表示刚从 Fault 恢复到 Switch on disabled。
 */
bool Ds402_ConsumeFaultResetAccepted(void)
{
    bool accepted = s_fault_reset_accepted;

    /* 该标志只允许被消费一次，防止运动模块跨多个周期重复清锁存。 */
    s_fault_reset_accepted = false;
    return accepted;
}
