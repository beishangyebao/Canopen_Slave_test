# DS402 状态机改造方案（修订版）

## 1. 改造目标

当前从站的 `0x6041 Statusword` 是“类 DS402 状态字”：代码主要根据 `0x6040` 低 4 位、CANopen NMT 状态、G4 反馈在线状态、G4 故障和通信超时直接拼出状态字。

这种方式能满足自定义主站联调，但不是完整 DS402 Power Drive System finite state automaton。
标准主站通常会按 DS402 的状态跳转流程操作驱动器，如果从站状态字没有严格反映状态机，容易出现以下问题：

- 主站发送 `shutdown/switch on/enable operation` 后，等待不到预期状态。
- 主站执行 `quick stop`、`disable voltage`、`fault reset` 时，从站状态字跳转不符合标准。
- 位置、速度、转矩三个模式分别拼状态字，长期会产生不一致。
- 故障恢复、急停恢复、模式切换时，主站看到的状态不稳定。

改造目标是新增一个统一 DS402 状态机模块，让 `generator.c` 和 `torque.c` 不再各自拼完整状态字，而是统一调用状态机生成 `0x6041`。

## 1.1 标准化改造结论

本次改造目标是把 slave 和 master 统一切换到真正的 DS402 状态机及响应机制。文档按这个目标约束实现，不再保留“类 DS402 状态字”或“主站盲发控制字”的兼容路径。

- slave 当前确实是“类 DS402 状态字”：`generator.c` 和 `torque.c` 分别用 `0x0237/0x0231/0x0208` 等模板拼 `0x6041`，没有统一的 DS402 PDS 状态机。
- master 和 slave 的 PDO 布局是匹配的：RPDO1 为 `6040 + 6060 + 607A`，RPDO2 为 `60FF + 6071`，TPDO1 为 `6041 + 606C`，TPDO2 为 `6064 + 6077`。标准化改造保持这套 PDO 映射不变，改变的是 `6040/6041` 的行为语义和主站响应流程。
- 本次修订明确要求：slave 上电后的 DS402 初始状态必须是 `NOT_READY_TO_SWITCH_ON`。即使当前没有母线预充、电源检测、驱动自检，也不能把初态简化为 `SWITCH_ON_DISABLED`。
- master 必须从“连续盲发 `0x0006 -> 0x0007 -> 0x000F`”改为“写一步控制字、等待对应状态字、再写下一步控制字”。
- slave 必须实现故障反应、故障锁存和 fault reset 上升沿；master 必须实现状态字解析、状态等待和 fault reset 上升沿命令。
- 本文档中的状态字数值只应作为“基础状态位”参考。叠加 bit4、bit5、bit9、bit10、bit12、bit13 后，主站测试应使用 mask 判断状态，而不是完整等值比较。

## 2. 必须新增文件

新增两个文件：

- `user/ds402_state.c`
- `user/ds402_state.h`
放在canopen-objdict文件夹下 和TestSlave.c在一起

职责边界：

- `ds402_state.c` 只负责 DS402 状态机、控制字解释、状态字基础位生成
- `generator.c` 继续负责位置/速度轨迹、跟随误差、目标到达、set-point ack
- `torque.c` 继续负责转矩限幅、转矩斜坡、转矩目标到达
- G4 通信超时、G4 设备故障、跟随误差等故障源由运动模块提供给状态机

## 3. DS402 状态枚举

定义如下状态：

```c
typedef enum
{
    DS402_STATE_NOT_READY_TO_SWITCH_ON = 0,
    DS402_STATE_SWITCH_ON_DISABLED,
    DS402_STATE_READY_TO_SWITCH_ON,
    DS402_STATE_SWITCHED_ON,
    DS402_STATE_OPERATION_ENABLED,
    DS402_STATE_QUICK_STOP_ACTIVE,
    DS402_STATE_FAULT_REACTION_ACTIVE,
    DS402_STATE_FAULT
} DS402_State;
```

本工程启动后必须先进入 `NOT_READY_TO_SWITCH_ON`。当前虽然没有母线预充、电源检测、驱动自检等真实硬件步骤，但仍把 `NOT_READY_TO_SWITCH_ON` 作为软件启动初始化窗口：对象字典、应用模块、G4 watchdog 起始时间戳、DS402 状态机内部变量完成初始化，并满足最小 Not ready 驻留时间后，才自动进入 `SWITCH_ON_DISABLED`。

后续如果加入母线预充、电源检测、驱动自检，则这些真实检查也并入同一个启动条件；检查未完成时保持 `NOT_READY_TO_SWITCH_ON`，检查完成后进入 `SWITCH_ON_DISABLED`。`0x6040 Controlword` 不应让设备从 `NOT_READY_TO_SWITCH_ON` 直接跳到 Ready/Switched/Operation enabled。

## 4. 控制字命令识别

`0x6040 Controlword` 的低位组合需要按命令识别，而不是只判断低 4 位是否等于 `0x000F`。

实现这些 helper：

```c
static bool Ds402_CwShutdown(uint16_t cw)
{
    return (cw & 0x0087u) == 0x0006u;
}

static bool Ds402_CwSwitchOn(uint16_t cw)
{
    return (cw & 0x008Fu) == 0x0007u;
}

static bool Ds402_CwEnableOperation(uint16_t cw)
{
    return (cw & 0x008Fu) == 0x000Fu;
}

static bool Ds402_CwDisableVoltage(uint16_t cw)
{
    return (cw & 0x0082u) == 0x0000u;
}

static bool Ds402_CwQuickStop(uint16_t cw)
{
    return (cw & 0x0086u) == 0x0002u;
}

static bool Ds402_CwDisableOperation(uint16_t cw)
{
    return (cw & 0x008Fu) == 0x0007u;
}

static bool Ds402_CwFaultResetRising(uint16_t cw, uint16_t prev_cw)
{
    return ((cw & 0x0080u) != 0u) && ((prev_cw & 0x0080u) == 0u);
}
```

说明：

- `shutdown` 通常由主站写 `0x0006`。
- `switch on` 通常由主站写 `0x0007`。
- `enable operation` 通常由主站写 `0x000F`。
- `quick stop` 常见命令是 bit2 清零，典型值 `0x0002`。
- `fault reset` 是 bit7 上升沿。不要长期按“bit7 置位”反复复位故障锁存，否则主站如果保持 bit7，会让故障恢复行为变得不稳定。
- bit4、bit5、bit6、bit8 等模式相关位可能和低 4 位同时出现，命令识别必须用 mask，不能做完整等值比较。

## 5. 状态跳转逻辑

状态机每 1 ms 调用一次，输入当前控制字和安全条件。

输入结构如下：

```c
typedef struct
{
    uint16_t controlword;
    bool startup_ready;
    bool nmt_operational;
    bool drive_online;
    bool local_fault;
    bool fault_reaction_done;
    bool quick_stop_done;
} DS402_Input;
```

含义：

- `startup_ready`：上电初始化条件。当前工程没有母线预充、电源检测、驱动自检时，也必须由软件显式给出该条件，例如状态机初始化完成且最小 Not ready 驻留时间已满足。复位后初值为 false，满足条件后置 true。
- `nmt_operational`：`TestSlave_Data.nodeState == Operational`。
- `drive_online`：G4 反馈链路已经稳定在线。
- `local_fault`：G4 设备故障、G4 通信超时、跟随误差等任一故障。注意：G4 通信超时通常会同时导致 `drive_online == false`，状态机必须先处理 `local_fault`，不能因为 `drive_online == false` 直接早退。
- `fault_reaction_done`：故障反应动作已经完成。当前 G4 反馈没有独立的“动作完成”位，因此本工程定义为：已持续发送 quick stop 或 disable 命令，且规划速度、实际速度或转矩需求已经接近 0。不能直接常置 true。
- `quick_stop_done`：急停减速已经完成。位置/速度模式使用规划速度和实际速度接近 0 判断；转矩模式使用需求转矩接近 0，并优先结合实际速度接近 0 判断。不能直接常置 true。

状态机内部还应保存上一拍 `controlword`，用于识别 fault reset 上升沿。

核心跳转逻辑：

```c
void Ds402_Update(const DS402_Input *in)
{
    uint16_t cw = in->controlword;
    uint16_t prev_cw = s_prev_controlword;
    bool fault_reset = Ds402_CwFaultResetRising(cw, prev_cw);
    bool unavailable = (!in->nmt_operational || !in->drive_online);

    /*
     * 故障优先级高于 drive_online。
     * G4 通信超时时 drive_online 往往已经是 false，如果这里先早退，
     * 通信超时就进不了 FAULT_REACTION_ACTIVE。
     */
    if (in->local_fault &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_FAULT_REACTION_ACTIVE;
    }

    if (!in->startup_ready &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
        s_prev_controlword = cw;
        return;
    }

    if (unavailable &&
        s_state != DS402_STATE_FAULT &&
        s_state != DS402_STATE_FAULT_REACTION_ACTIVE) {
        s_state = DS402_STATE_SWITCH_ON_DISABLED;
        s_prev_controlword = cw;
        return;
    }

    switch (s_state) {
        case DS402_STATE_NOT_READY_TO_SWITCH_ON:
            if (in->startup_ready) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            }
            break;

        case DS402_STATE_SWITCH_ON_DISABLED:
            if (Ds402_CwShutdown(cw)) {
                s_state = DS402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case DS402_STATE_READY_TO_SWITCH_ON:
            if (Ds402_CwDisableVoltage(cw)) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (Ds402_CwSwitchOn(cw)) {
                s_state = DS402_STATE_SWITCHED_ON;
            } else if (Ds402_CwEnableOperation(cw)) {
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
                s_state = DS402_STATE_FAULT_REACTION_ACTIVE;
            } else if (in->quick_stop_done && Ds402_CwDisableVoltage(cw)) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            } else if (in->quick_stop_done && Ds402_CwEnableOperation(cw)) {
                s_state = DS402_STATE_OPERATION_ENABLED;
            }
            break;

        case DS402_STATE_FAULT_REACTION_ACTIVE:
            if (in->fault_reaction_done) {
                s_state = DS402_STATE_FAULT;
            }
            break;

        case DS402_STATE_FAULT:
            if (!in->local_fault && fault_reset) {
                s_state = DS402_STATE_SWITCH_ON_DISABLED;
            }
            break;

        default:
            s_state = DS402_STATE_NOT_READY_TO_SWITCH_ON;
            break;
    }

    s_prev_controlword = cw;
}
```

注意：上面是实现骨架。`startup_ready` 不能在复位后直接常置 true；即使当前没有真实硬件自检，也要至少经过一次明确的软件初始化完成条件或最小驻留窗口，让主站能够看到 `NOT_READY_TO_SWITCH_ON` 初态。`quick_stop_done` 和 `fault_reaction_done` 必须由运动模块根据实际停止条件提供，不能为了让状态跳转更快而直接置 true。

## 6. 状态字生成

`ds402_state.c` 提供：

```c
uint16_t Ds402_BuildStatusword(uint16_t mode_bits);
bool Ds402_IsOperationEnabled(void);
bool Ds402_IsQuickStopActive(void);
bool Ds402_IsFault(void);
```

基础状态字位如下。这里的数值是“基础状态位”，不是最终一定等于的完整 `0x6041`：

| 状态 | 基础 statusword |
| --- | --- |
| Not ready to switch on | `0x0000` |
| Switch on disabled | `0x0040` |
| Ready to switch on | `0x0021` |
| Switched on | `0x0023` |
| Operation enabled | `0x0027` |
| Quick stop active | `0x0007` |
| Fault reaction active | `0x000F` |
| Fault | `0x0008` |

然后统一叠加公共位：

- bit4 `Voltage enabled`：G4 在线且允许驱动电源时置 1。
- bit5 `Quick stop`：非 quick stop active 时置 1；quick stop active 时清 0。
- bit9 `Remote`：CANopen 远程控制有效时置 1，当前工程可在 NMT Operational 时置 1。
- bit10 `Target reached`：由 `generator.c` 或 `torque.c` 传入。
- bit12：位置模式 set-point acknowledge 或 homing attained 等模式相关位。
- bit13：位置模式 following error 或 homing error 等模式相关位。

因此，master 或测试脚本判断 DS402 状态时应按 mask 判断，而不是完整等值比较：

| 状态 | mask | value |
| --- | --- | --- |
| Not ready to switch on | `0x004F` | `0x0000` |
| Switch on disabled | `0x004F` | `0x0040` |
| Ready to switch on | `0x006F` | `0x0021` |
| Switched on | `0x006F` | `0x0023` |
| Operation enabled | `0x006F` | `0x0027` |
| Quick stop active | `0x006F` | `0x0007` |
| Fault reaction active | `0x004F` | `0x000F` |
| Fault | `0x004F` | `0x0008` |

设计约束是状态机只生成基础状态和公共位，模式相关位由调用方传入：

```c
typedef struct
{
    bool voltage_enabled;
    bool remote;
    bool target_reached;
    bool setpoint_ack;
    bool following_error;
} DS402_StatusBits;

uint16_t Ds402_BuildStatusword(const DS402_StatusBits *bits);
```

## 7. generator.c 的改法

当前位置/速度模式里有这些逻辑：

- `enabled = operational && g_g4_feedback_online && ((cw & 0x000F) == 0x000F) && ...`
- `qstop_active = ...`
- `Generator_BuildStatusWord()` 直接拼 `0x0237/0x0231/0x0208`

改成：

1. 每个 1 ms 周期先构造 `DS402_Input`。
2. 调用 `Ds402_Update(&input)`。
3. 用 `Ds402_IsOperationEnabled()` 决定是否允许运动。
4. 用 `Ds402_IsQuickStopActive()` 决定是否进入 quick stop 轨迹。
5. 用 `Ds402_BuildStatusword(&bits)` 生成 `0x6041`。

也就是说，原来的：

```c
bool enabled = operational &&
               g_g4_feedback_online &&
               ((cw & 0x000Fu) == 0x000Fu) &&
               !g_g4_fault_status &&
               !g_g4_comm_timeout &&
               !s_tp.following_error;
```

替换为：

```c
bool enabled = Ds402_IsOperationEnabled() &&
               !s_tp.following_error;
```

故障条件由 generator 提供：

```c
input.local_fault = g_g4_fault_status ||
                    g_g4_comm_timeout ||
                    s_tp.following_error;
```

落地时要注意两点：

- DS402 状态机必须是独立单例模块，不能把状态枚举放进 `TrapProfile_state`。模式切换、`TrapProfile_ResetState()` 或 `Torque_Reset()` 只能清运动规划状态，不能清 DS402 的 fault 锁存。
- 当前 `TrapProfile_ResetState()` 会清 `s_tp.following_error`。标准化改造必须把“跟随误差故障源/锁存”从普通规划复位里拆出来，或在 DS402 模块里另设故障锁存位，只有故障源消失且收到 fault reset 上升沿才清。

## 8. torque.c 的改法

转矩模式也不要自己拼 `0x6041`。改为：

1. `Torque_Run()` 开头构造同样的 `DS402_Input`。
2. 调用 `Ds402_Update(&input)`。
3. `enabled = Ds402_IsOperationEnabled()`。
4. `qstop_active = Ds402_IsQuickStopActive() || Ds402_IsFault()`。
5. `Torque_BuildStatusWord()` 删除，改为调用 `Ds402_BuildStatusword()`。

这样位置、速度、转矩三种模式的基础状态完全一致，只有 bit10、bit12、bit13 等模式相关位不同。

转矩模式的 `quick_stop_done` 使用以下条件组合判断：

- `s_torque.demand_q16` 已经斜坡到 0。
- G4 回传的实际速度接近 0。

## 9. main.c 的统一调度要求

当前 `main.c` 按 `6060` 分发：

- mode 4 调 `Torque_Run()`
- 其他调 `Generator_Run()`

标准化后，DS402 状态机每个 1 ms 只能更新一次，并且必须先更新 DS402 状态，再由运动模块根据 `Ds402_IsOperationEnabled()`、`Ds402_IsQuickStopActive()`、`Ds402_IsFault()` 决定 G4 命令。结构如下：

```c
while (Generator_ConsumeTick()) {
    Ds402_PrepareCommonInput();
    Ds402_Update(&input);

    if ((int8_t)TestSlave_obj6060 == MAIN_OPMODE_PROFILE_TORQUE) {
        Torque_Run();
    } else {
        Torque_Reset();
        Generator_Run();
    }
}
```

`ds402_state.c` 内部状态必须只有一份。`Ds402_Update()` 不能同时在 `main.c`、`Generator_Run()`、`Torque_Run()` 中重复调用。最终实现以统一调度为准：`main.c` 或一个公共调度函数负责更新 DS402，运动模块只提供故障源、停止完成条件和模式相关状态位。

## 10. 与 G4 命令帧的关系

状态机只决定 CANopen 层是否允许 operation enabled，不直接发 G4 命令。

G4 命令仍由原模块生成：

- `OPERATION_ENABLED`：允许发送 `CMD_TYPE_ENABLE`。
- `QUICK_STOP_ACTIVE`：发送 `CMD_TYPE_QUICKSTOP`。
- `FAULT_REACTION_ACTIVE`：按 `0x605E Fault reaction option code` 决定发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`；默认策略为先 quick stop 到停稳，再进入 `FAULT`。
- `FAULT`：发送 `CMD_TYPE_DISABLE`。
- `NOT_READY_TO_SWITCH_ON/SWITCH_ON_DISABLED/READY_TO_SWITCH_ON/SWITCHED_ON`：发送 `CMD_TYPE_DISABLE` 或保持下游非使能。

这样 CANopen 状态和下游真实执行动作不会混在一起。

## 11. 标准化实施清单

### 11.1 DS402 状态机

- 新增 `ds402_state.c/h`。
- DS402 状态机每 1 ms 只更新一次。
- 状态字基础位由状态机生成，`generator.c` 和 `torque.c` 不再拼完整 `0x6041`。
- 位置/速度/转矩轨迹算法保持原样。
- `quick_stop_done` 和 `fault_reaction_done` 必须由真实停止条件提供。
- `fault reset` 按 bit7 上升沿实现。

目标：slave 对外表现为标准 DS402 状态机。

### 11.2 故障反应和锁存

- G4 通信超时进入 `FAULT_REACTION_ACTIVE`。
- G4 故障进入 `FAULT_REACTION_ACTIVE`。
- 跟随误差进入 `FAULT_REACTION_ACTIVE`。
- 故障反应完成后锁存到 `FAULT`。
- 主站必须清除故障源并写 bit7 fault reset 才能恢复。
- 跟随误差、G4 超时、G4 故障的锁存与普通运动复位解耦，不能被模式切换或 disable voltage 自动清除。

目标：故障不再自动无感恢复，符合工业预期。

### 11.3 option code 对象

加入这些对象，并让它们真正影响停机策略：

- `0x605A Quick stop option code`
- `0x605B Shutdown option code`
- `0x605C Disable operation option code`
- `0x605D Halt option code`
- `0x605E Fault reaction option code`

目标：主站通过 option code 配置不同停机策略。

### 11.4 master 配套

- master 必须按状态字 mask 解析 DS402 状态。
- master 必须按“写控制字 -> 等待状态字确认”的方式执行 shutdown、switch on、enable operation。
- master 必须按 bit7 上升沿执行 fault reset。
- master 的 `status` 命令必须显示 DS402 状态名和关键状态位。

## 12. 最小测试清单

每次改完都用主站按顺序测试。状态判断要用第 6 节的 mask，不要用完整等值：

1. 刚上电、`startup_ready` 未满足时读取 `6041`，应能按 mask 判断为 `Not ready to switch on`。
2. `startup_ready` 满足后，若没有故障锁存，读取 `6041` 应能按 mask 判断为 `Switch on disabled`。
3. G4 在线、NMT Operational 后写 `6040=0x0006`，应进入 `Ready to switch on`。
4. 写 `6040=0x0007`，应进入 `Switched on`。
5. 写 `6040=0x000F`，应进入 `Operation enabled`。
6. 清 bit2，例如写 `6040=0x000B` 或 quick stop 命令组合，应进入 `Quick stop active`。
7. 制造 G4 通信超时，应进入 `Fault reaction active`，随后进入 `Fault`。
8. 故障未清除时写 bit7 上升沿，不应恢复。
9. 故障清除后写 bit7 上升沿，应回到 `Switch on disabled`，不回到 `Not ready to switch on`。
10. 只有重新上电或复位启动流程时，状态机才重新从 `Not ready to switch on` 开始。
11. 保持 bit7 不放时，不应反复触发 fault reset；必须先清 bit7 再重新置位才算下一次 reset。
12. 在 `Operation enabled` 下切换 `6060=1/3/4`，基础状态不应乱跳。
13. 位置模式 new set-point 时，bit12 set-point ack 仍应按原逻辑变化。
14. master 侧必须每写一步控制字都等待对应状态，而不是连续盲发三步。

## 12.1 对 master 的强制配套要求

当前 master 的 PDO 映射和 slave 匹配，标准化改造保持 PDO 和 COB-ID 不变。但 master 的控制流程必须同步修改，否则 slave 进入真正 DS402 状态机后，主站无法可靠完成使能、急停、故障恢复和状态诊断。

master 必须实现以下能力：

- 对 `recv_statusword` 做 DS402 mask 解析。
- 执行 `Ready to switch on -> Switched on -> Operation enabled` 时，每一步都等待状态字确认。
- 对 G4 超时、G4 故障、跟随误差引起的 `Fault` 执行标准 fault reset 上升沿。
- `status` 命令显示 DS402 状态名，而不仅是十六进制值。
- 连续位置、速度、转矩命令发送前检查 `Operation enabled`。

master 侧具体改法见 `canopen-master/ds402_master_update_plan.md`。

## 13. 最关键的注意点

不要让 `generator.c` 和 `torque.c` 再各自决定“现在是不是 DS402 enabled”。它们应该只问状态机：

```c
if (Ds402_IsOperationEnabled()) {
    /* 允许执行位置/速度/转矩命令 */
}
```

也不要让两个模块各自拼完整 `6041`。它们只负责把模式相关位交给状态机：

```c
bits.target_reached = s_tp.target_reached;
bits.setpoint_ack = s_tp.setpoint_ack;
bits.following_error = s_tp.following_error;
status = Ds402_BuildStatusword(&bits);
```

这样改完后，三模控制会共享同一套 DS402 行为，主站也更容易按标准流程控制从站。
