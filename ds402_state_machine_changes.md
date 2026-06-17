# DS402 状态机改造修改说明

## 1. 新增文件

### `canopen/objdict/ds402_state.h`

- 新增 `DS402_State` 状态枚举，覆盖 `NOT_READY_TO_SWITCH_ON`、`SWITCH_ON_DISABLED`、`READY_TO_SWITCH_ON`、`SWITCHED_ON`、`OPERATION_ENABLED`、`QUICK_STOP_ACTIVE`、`FAULT_REACTION_ACTIVE`、`FAULT`。
- 新增 `DS402_Input`，用于 main.c 每 1 ms 向状态机传入控制字、NMT 状态、G4 在线状态、本地故障和停机完成条件。
- 新增 `DS402_StatusBits`，用于由运动模块传入 bit10、bit12、bit13 等模式相关状态位。
- 对外提供 `Ds402_Update()`、`Ds402_BuildStatusword()`、`Ds402_PublishStatusword()`、`Ds402_IsOperationEnabled()`、`Ds402_IsQuickStopActive()`、`Ds402_IsFaultReactionActive()`、`Ds402_IsFault()` 等接口。

### `canopen/objdict/ds402_state.c`

- 实现统一 DS402 PDS 状态机，状态机由 `main.c` 每 1 ms 只调用一次。
- 按 mask 识别 `shutdown`、`switch on`、`enable operation`、`disable voltage`、`quick stop`、`disable operation` 和 `fault reset` 上升沿。
- 上电初态固定为 `NOT_READY_TO_SWITCH_ON`，并增加最小软件驻留窗口，之后才允许进入 `SWITCH_ON_DISABLED`。
- 本地故障优先进入 `FAULT_REACTION_ACTIVE`，故障反应完成后锁存到 `FAULT`。
- `fault reset` 使用 bit7 上升沿，并通过 `Ds402_ConsumeFaultResetAccepted()` 通知运动模块清除软件锁存。
- 统一生成并写回 `0x6041 Statusword`，基础状态位由状态机生成，bit4/bit5/bit9/bit10/bit12/bit13 统一叠加。

## 2. 修改文件

### `user/main.c`

- 引入 `ds402_state.h`。
- 增加 `Main_UpdateDs402State()`，每个 1 ms 周期先根据当前 `0x6060` 调用 `Generator_FillDs402Input()` 或 `Torque_FillDs402Input()`，再统一调用 `Ds402_Update()`。
- 增加 `Main_PublishDs402Statusword()`，运动模块完成本周期计算后统一发布 `0x6041`。
- `fault reset` 被状态机接受后，同时通知 `generator.c` 和 `torque.c` 清除本地软锁存，避免模式切换后残留故障状态。

### `user/generator.c/.h`

- 位置/速度轨迹算法保持原样。
- 原来的低四位控制字使能判断和 `Generator_BuildStatusWord()` 已删除。
- 新增 `Generator_FillDs402Input()`，向状态机提供 G4 故障、通信超时、跟随误差、quick stop done、fault reaction done。
- 新增 `Generator_FillDs402StatusBits()`，只提供 bit10 target reached、bit12 set-point acknowledge、bit13 following error。
- 新增 `Generator_OnDs402FaultReset()`，只在 DS402 接受 fault reset 后清除跟随误差软锁存。
- `TrapProfile_ResetState()` 不再清除 `following_error`，避免模式切换或普通失能自动清故障。
- G4 命令选择改为查询 `Ds402_IsOperationEnabled()`、`Ds402_IsQuickStopActive()`、`Ds402_IsFaultReactionActive()`、`Ds402_IsFault()`。
- `0x605A/0x605B/0x605C/0x605D/0x605E` 会影响 quick stop、shutdown、disable operation、halt、fault reaction 的 G4 停机命令选择。

### `user/torque.c/.h`

- 转矩限幅和 Q16 斜坡算法保持原样。
- 原来的 `Torque_BuildStatusWord()` 已删除，转矩模块不再直接写 `0x6041`。
- 新增 `Torque_FillDs402Input()`，向状态机提供 G4 故障、通信超时、转矩 demand 回零和实际速度接近 0 的停机完成条件。
- 新增 `Torque_FillDs402StatusBits()`，只提供 bit10 target reached。
- 新增 `Torque_OnDs402FaultReset()`，在 fault reset 被接受后复位转矩软状态。
- G4 命令选择改为由 DS402 状态驱动，并接入 `0x605A~0x605E` option code。

### `canopen/objdict/TestSlave.c/.h`

- 新增对象字典索引：
  - `0x605A Quick stop option code`
  - `0x605B Shutdown option code`
  - `0x605C Disable operation option code`
  - `0x605D Halt option code`
  - `0x605E Fault reaction option code`
- 五个对象均为 `INTEGER16`、`RW`。
- 已同步插入 `TestSlave_objdict[]` 和 `TestSlave_scanIndexOD()` 查表。

### `project/canopen-slave.uvprojx`

- 在 `canopen-objdict` 分组加入 `ds402_state.h` 和 `ds402_state.c`，确保 Keil 命令行编译会包含 DS402 状态机。

### `project/canopen-slave.uvoptx`

- 同步加入 `ds402_state.h` 和 `ds402_state.c`，方便 Keil 图形界面打开工程时显示新文件。



## 4. 状态机整体工作流程

### 4.1 一句话说明

DS402 状态机是 slave 对外表现为标准 CiA402 伺服驱动器的统一入口
主站写 `0x6040 Controlword` 和模式/目标对象
slave 在 1 ms 周期内把 CANopen 状态、G4 在线状态、运动模块故障源、停机完成条件送入 `Ds402_Update()`
状态机再输出当前 DS402 状态和 `0x6041 Statusword`。
位置、转速、转矩三个模式不再各自决定 `6041`，而是只向状态机提供模式相关位

### 4.2 每 1 ms 的执行顺序

主循环在 `user/main.c` 中执行，顺序固定如下：

1. `CAN_App_Check_G4_Watchdog()` 检查 G4 反馈是否超时，更新 `g_g4_feedback_online` 和 `g_g4_comm_timeout`。
2. `Generator_ConsumeTick()` 消费 TIM3 产生的 1 ms 节拍。
3. `Main_IsTorqueMode()` 读取 `0x6060 Modes of operation`，判断本周期是转矩模式还是由 generator 处理的位置/转速模式
4. `Main_UpdateDs402State()` 准备 `DS402_Input`，并且只调用一次 `Ds402_Update()`。
5. 如果 DS402 接受了 `fault reset` 上升沿，`main.c` 调用 `Generator_OnDs402FaultReset()` 和 `Torque_OnDs402FaultReset()` 清理本地软锁存。
6. 按模式运行运动模块：
   - `0x6060 == 4`：调用 `Torque_Run()`。
   - 其他模式：调用 `Torque_Reset()`，再调用 `Generator_Run()`。
7. `Main_PublishDs402Statusword()` 初始化 `DS402_StatusBits`，让当前运动模块补充 bit10/bit12/bit13，最后调用 `Ds402_PublishStatusword()` 写回 `0x6041`。

### 4.3 状态机不直接做的事情

`ds402_state.c` 不直接发送 G4 命令，不计算位置轨迹，不计算转速轨迹，也不计算转矩斜坡。它只负责：

- 解释 `0x6040` 控制字命令。
- 保存 DS402 PDS 状态。
- 判断 `fault reset` 上升沿。
- 生成 `0x6041` 基础状态位和公共位。
- 给运动模块提供查询接口，例如 `Ds402_IsOperationEnabled()`、`Ds402_IsQuickStopActive()`、`Ds402_IsFaultReactionActive()`、`Ds402_IsFault()`。

真正下发给 G4/control 的命令仍由 `generator.c` 或 `torque.c` 生成，再通过 `Driver_Send_CommandFrame()` 发到 CAN ID `0x010`。

## 5. 状态机输入来源

`DS402_Input` 是状态机每 1 ms 的完整输入。每个字段的来源和作用如下：

`controlword` 由 `Generator_FillDs402Input()` 或 `Torque_FillDs402Input()` 填写
数据来自 OD 对象 `TestSlave_obj6040`，也就是主站写入的 `0x6040 Controlword`
状态机用它识别 `shutdown`、`switch on`、`enable operation`、`quick stop`、`disable voltage` 和 `fault reset`

`startup_ready` 由当前运动模块填写为 true，表示对象字典、应用模块、G4 watchdog 起始时间戳等软件初始化工作已经完成
状态机收到该输入后，仍会额外执行最小 Not ready 驻留窗口，确保主站能看到 `NOT_READY_TO_SWITCH_ON` 初态

`nmt_operational` 由当前运动模块填写，来源是 `TestSlave_Data.nodeState == Operational`
它表示 CANopen 节点是否进入 Operational；如果不是 Operational，状态机不会进入 ready、switched on 或 operation enabled

`drive_online` 由当前运动模块填写，来源是 `g_g4_feedback_online`
该标志由 `CAN_App_Check_G4_Watchdog()` 根据 G4 反馈帧维护，表示 G4 反馈链路是否稳定在线
如果未在线，状态机保持或回到 `SWITCH_ON_DISABLED`，但故障反应和故障锁存状态不会被这个条件覆盖

`local_fault` 由当前运动模块填写，来源包括 G4 设备故障、G4 通信超时和位置模式跟随误差
只要该输入为 true，状态机优先进入 `FAULT_REACTION_ACTIVE`

`fault_reset_allowed` 由当前运动模块填写，含义是硬故障和通信超时是否已经消失
该输入为 false 时，即使主站发出 `0x6040 bit7` 上升沿，状态机也会继续保持 `FAULT`

`fault_reaction_done` 由当前运动模块根据真实停机条件填写
位置/转速模式主要看规划速度和实际速度是否接近 0；转矩模式主要看转矩需求是否回到 0，并尽量结合实际速度是否接近 0
该输入为 true 后，状态机从 `FAULT_REACTION_ACTIVE` 锁存到 `FAULT`

`quick_stop_done` 也由当前运动模块根据停机条件填写。它表示 quick stop 已经停稳
该输入为 true 后，状态机才允许从 `QUICK_STOP_ACTIVE` 恢复到 `OPERATION_ENABLED` 或退回 `SWITCH_ON_DISABLED`

## 6. 状态机输出去向

状态机有三类输出：DS402 状态、状态字、fault reset 接受事件

当前 `DS402_State` 由 `ds402_state.c` 维护
`generator.c` 和 `torque.c` 不直接修改这个状态
只通过 `Ds402_IsOperationEnabled()`、`Ds402_IsQuickStopActive()`、`Ds402_IsFaultReactionActive()`、`Ds402_IsFault()` 等查询函数读取它
运动模块读取状态后，决定本周期是允许正常运动、执行 quick stop、执行 fault reaction，还是直接 disable

`0x6041 Statusword` 由 `Ds402_PublishStatusword()` 生成并写入 OD 对象 `TestSlave_obj6041`
主站可以通过 TPDO 或 SDO 读取这个对象，用它判断当前是否处于 Not ready、Switch on disabled、Ready、Switched on、Operation enabled、Quick stop active、Fault reaction active 或 Fault

`fault reset accepted` 是 `Ds402_ConsumeFaultResetAccepted()` 给 `main.c` 的一次性事件
`main.c` 收到后，会转发给 `Generator_OnDs402FaultReset()` 和 `Torque_OnDs402FaultReset()`
它只用于清除跟随误差或转矩软状态，G4 硬故障和通信超时不能在这里被伪造清除

状态机不会直接输出 G4 命令。G4 命令是运动模块读取 DS402 状态后生成的间接输出

## 7. 主站看到的状态字来源

`0x6041 Statusword` 分两层生成：

1. `ds402_state.c` 根据当前 `DS402_State` 生成基础状态位：
   - `NOT_READY_TO_SWITCH_ON` -> `0x0000`
   - `SWITCH_ON_DISABLED` -> `0x0040`
   - `READY_TO_SWITCH_ON` -> `0x0021`
   - `SWITCHED_ON` -> `0x0023`
   - `OPERATION_ENABLED` -> `0x0027`
   - `QUICK_STOP_ACTIVE` -> `0x0007`
   - `FAULT_REACTION_ACTIVE` -> `0x000F`
   - `FAULT` -> `0x0008`

2. `Ds402_BuildStatusword()` 叠加公共位和模式位：
   - bit4 `Voltage enabled`：来自最近一次 `drive_online`
   - bit5 `Quick stop`：除了 `QUICK_STOP_ACTIVE` 外都置 1
   - bit9 `Remote`：来自最近一次 `nmt_operational`
   - bit10 `Target reached`：由当前运动模块提供
   - bit12 `Set-point acknowledge`：位置模式由 `generator.c` 提供
   - bit13 `Following error`：位置模式由 `generator.c` 提供

因此主站判断 DS402 状态时，应按 mask 判断基础状态，不要对整个 `0x6041` 做完整等值比较

## 8. 状态跳转工作方式

### 8.1 上电和初始化

上电后 `Ds402_Init()` 把状态固定为 `NOT_READY_TO_SWITCH_ON`。
即使 `startup_ready` 已经为 true，状态机也会保留一个最小软件驻留窗口，让主站能看到 Not ready 初态
驻留结束后，如果没有 fault，状态进入 `SWITCH_ON_DISABLED`

如果 NMT 还不是 Operational，或者 G4 反馈还没稳定在线，状态机会保持或回到 `SWITCH_ON_DISABLED`
这表示 CANopen 层可以通信，但驱动还不允许执行标准使能流程

### 8.2 标准使能流程

主站需要按标准 DS402 顺序写控制字：

1. 写 `0x6040 = 0x0006`，状态从 `SWITCH_ON_DISABLED` 进入 `READY_TO_SWITCH_ON`
2. 写 `0x6040 = 0x0007`，状态进入 `SWITCHED_ON`
3. 写 `0x6040 = 0x000F`，状态进入 `OPERATION_ENABLED`

只有 `Ds402_IsOperationEnabled()` 为 true 时，`generator.c` 或 `torque.c` 才会允许下发正常 G4 运行命令

### 8.3 Quick stop

主站清 bit2，例如写 quick stop 命令组合后，状态机进入 `QUICK_STOP_ACTIVE`。
运动模块查询到 `Ds402_IsQuickStopActive()` 后，不再执行正常目标，而是把规划速度或转矩需求向 0 收敛
并按 `0x605A Quick stop option code` 选择发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`

停稳条件由运动模块反馈给 `quick_stop_done`。
停稳后，主站可以通过 disable voltage 回到 `SWITCH_ON_DISABLED`
也可以重新写 enable operation 回到 `OPERATION_ENABLED`

### 8.4 Fault reaction 和 Fault

G4 故障、G4 通信超时、跟随误差任一出现时，运动模块把 `local_fault` 置 true
状态机优先进入 `FAULT_REACTION_ACTIVE`，运动模块按 `0x605E Fault reaction option code` 发送 quick stop 或 disable，并把速度/转矩向 0 收敛

运动模块判断故障反应已经完成后，把 `fault_reaction_done` 置 true。
状态机随后锁存到 `FAULT`。进入 `FAULT` 后，运动模块发送 disable，不再发送正常运行命令

### 8.5 Fault reset

`fault reset` 只认 `0x6040 bit7` 上升沿
主站如果一直保持 bit7 为 1，不会反复复位
状态机只有在 `fault_reset_allowed` 为 true 时才接受复位

- G4 硬故障还在：不允许复位
- G4 通信超时还在：不允许复位
- 只有跟随误差这类 slave 软件锁存时：允许状态机接受复位，然后 `Generator_OnDs402FaultReset()` 清除跟随误差锁存

复位成功后状态从 `FAULT` 回到 `SWITCH_ON_DISABLED`，不会回到 `NOT_READY_TO_SWITCH_ON`
只有重新上电或重新初始化状态机才会从 Not ready 开始


## 9. 位置模式数据流

位置模式对应 `0x6060 = 1`，由 `generator.c` 负责轨迹、位置外环、状态位和 G4 命令

### 9.1 输入来源

主站给 slave：

主站写 `0x6040 Controlword` 后，数据进入 `TestSlave_obj6040`
DS402 状态机读取它完成标准状态跳转；`generator.c` 还会读取其中的 new set-point、relative 和 halt 位，用来启动新位置目标、判断相对运动和暂停请求

主站写 `0x6060 = 1` 后，数据进入 `TestSlave_obj6060`。`main.c` 据此选择 Profile Position 模式，本周期由 `generator.c` 执行位置轨迹

主站写 `0x607A Target position` 后，数据进入 `TestSlave_obj607A`。`generator.c` 在 new set-point 上升沿到来时读取它，作为新的位置目标

主站写 `0x6081 Profile velocity` 后，数据进入 `TestSlave_obj6081`。位置模式把它作为规划最大速度使用

主站写 `0x6083 Profile acceleration` 后，数据进入 `TestSlave_obj6083`，用于位置轨迹加速段

主站写 `0x6084 Profile deceleration` 后，数据进入 `TestSlave_obj6084`，用于位置轨迹常规减速段

主站写 `0x6085 Quick stop deceleration` 后，数据进入 `TestSlave_obj6085`，用于 quick stop 时的急停减速度

主站写 `0x6065 Following error window` 和 `0x6066 Following error timeout` 后，数据进入 `TestSlave_obj6065` 和 `TestSlave_obj6066`。`generator.c` 用它们判断规划位置与实际位置偏差是否已经形成跟随误差

主站写 `0x607D Software position limit` 后，数据进入 `TestSlave_obj607D_min` 和 `TestSlave_obj607D_max`。`generator.c` 用它们限制目标位置和规划位置，防止软件轨迹越过限位

G4/control 给 slave：

G4 反馈帧里的 mode 字段由 `Generator_OnG4Feedback()` 写入 `0x6061`，主站读取 `0x6061` 就能看到 G4/control 实际回显的执行模式

G4 反馈帧里的实际转矩由 `Generator_OnG4Feedback()` 写入 `0x6077`，主站通过该对象观察 actual torque

G4 反馈帧里的主反馈字段在位置模式下解释为实际位置，并写入 `0x6064`。`generator.c` 用 `0x6064` 作为位置外环反馈和跟随误差判断依据，主站也通过它观察 actual position。

G4 反馈帧状态字节的 bit0 写入全局 `g_g4_fault_status`。`Generator_FillDs402Input()` 会把它作为 `local_fault` 的来源之一送给 DS402 状态机。

G4 是否持续有反馈由 `CAN_App_Check_G4_Watchdog()` 维护成 `g_g4_feedback_online` 和 `g_g4_comm_timeout`。前者作为 DS402 的 `drive_online` 输入，后者作为 `local_fault` 输入。

### 9.2 给 DS402 的输入

`Generator_FillDs402Input()` 给状态机以下输入：

- `controlword`：来自 `0x6040`
- `nmt_operational`：来自 `TestSlave_Data.nodeState`
- `drive_online`：来自 `g_g4_feedback_online`
- `local_fault`：`g_g4_fault_status || g_g4_comm_timeout || s_tp.following_error`
- `fault_reset_allowed`：G4 硬故障和通信超时都消失后为 true
- `quick_stop_done/fault_reaction_done`：规划速度接近 0；位置模式没有独立实际速度反馈，因此以本地规划停稳为主

### 9.3 DS402 给位置模式的输出

位置模式通过查询 DS402 得到：

- `Ds402_IsOperationEnabled()`：true 时允许接受位置目标并运行轨迹
- `Ds402_IsQuickStopActive()`：true 时进入急停减速
- `Ds402_IsFaultReactionActive()`：true 时进入故障反应停机
- `Ds402_IsFault()`：true 时发送 disable，不再运行

### 9.4 位置模式给 G4/control 的输出

`Generator_Run()` 先更新梯形轨迹，再用 slave 本地位置外环计算速度命令。最终通过 `Generator_BuildCommandFrame()` 生成 G4 命令：

当 DS402 处于 `OPERATION_ENABLED`，并且没有 halt 或 fault 时，`generator.c` 生成 `CMD_TYPE_ENABLE`
位置模式下发给 G4/control 的目标值不是位置，而是 slave 本地位置外环计算出的速度给定

当 DS402 处于 `QUICK_STOP_ACTIVE` 时，`generator.c` 按 `0x605A Quick stop option code` 选择发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`
此时目标值为 0 或当前停机目标，目的都是让下游受控停住

当 DS402 处于 `FAULT_REACTION_ACTIVE` 时，`generator.c` 按 `0x605E Fault reaction option code` 选择发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`
此时不再接受新的位置目标，只执行故障反应停机

当 DS402 处于 `FAULT` 时，`generator.c` 发送 `CMD_TYPE_DISABLE`，目标值为 0，下游保持非使能

当 DS402 尚未使能时，`generator.c` 默认发送 `CMD_TYPE_DISABLE`；如果当前状态和 option code 要求受控停机，则按 option code 选择 `CMD_TYPE_HALT`

命令经 `Driver_Send_CommandFrame()` 发到 G4/control

### 9.5 位置模式给主站的输出

位置模式通过 OD/TPDO 给主站：

- `0x6041 Statusword`：由 DS402 统一生成
- `0x6061 Modes of operation display`：G4 反馈 mode
- `0x6062 Position demand value`：内部规划位置
- `0x6064 Position actual value`：G4 实际位置
- `0x606C Velocity actual value`：非位置反馈时更新
- `0x6077 Torque actual value`：G4 实际转矩
- `0x6041 bit10`：位置到位
- `0x6041 bit12`：new set-point acknowledge
- `0x6041 bit13`：following error

## 10. 转速模式数据流

转速模式对应 `0x6060 = 3`，也由 `generator.c` 负责。它和位置模式共用 DS402 输入输出框架，但运动目标从位置目标变成速度目标

### 10.1 输入来源

主站给 slave：

主站写 `0x6040 Controlword` 后，数据进入 `TestSlave_obj6040`。DS402 状态机用它完成状态跳转
`generator.c` 同时读取 halt 和 quick stop 相关控制位，用于转速模式停机处理

主站写 `0x6060 = 3` 后，数据进入 `TestSlave_obj6060`。`main.c` 据此选择 Profile Velocity 模式，本周期仍由 `generator.c` 负责

主站写 `0x60FF Target velocity` 后，数据进入 `TestSlave_obj60FF`。`generator.c` 把它作为速度目标，并按加减速参数斜坡到该目标

主站写 `0x607F Max profile velocity` 后，数据进入 `TestSlave_obj607F`，作为 profile velocity 的速度上限之一

主站写 `0x6080 Max motor speed` 后，数据进入 `TestSlave_obj6080`，作为电机允许速度上限。`generator.c` 会综合 `0x607F` 和 `0x6080`，取更严格的限制

主站写 `0x6083 Profile acceleration` 后，数据进入 `TestSlave_obj6083`，用于速度目标上升时的加速度限制

主站写 `0x6084 Profile deceleration` 后，数据进入 `TestSlave_obj6084`，用于速度目标下降或普通停机时的减速度限制

主站写 `0x6085 Quick stop deceleration` 后，数据进入 `TestSlave_obj6085`，用于 quick stop 时的急停减速度

G4/control 给 slave：

G4 反馈帧里的 mode 字段写入 `0x6061`，主站用它观察 G4/control 实际执行模式

G4 反馈帧里的实际转矩写入 `0x6077`，主站用它观察 actual torque

G4 反馈帧里的主反馈字段在转速模式下解释为实际速度，并写入 `0x606C`
`Generator_FillDs402Input()` 会用 `0x606C` 判断 quick stop 或 fault reaction 是否已经停稳，主站也通过它观察 actual velocity

G4 反馈帧状态字节的 bit0 写入 `g_g4_fault_status`，作为 DS402 `local_fault` 的来源

G4 反馈是否持续到达由 `CAN_App_Check_G4_Watchdog()` 维护成 `g_g4_feedback_online` 和 `g_g4_comm_timeout`
`g_g4_feedback_online` 送给 DS402 的 `drive_online`，`g_g4_comm_timeout` 送给 DS402 的 `local_fault`

### 10.2 给 DS402 的输入

转速模式同样由 `Generator_FillDs402Input()` 给状态机输入

- `local_fault`：G4 故障或通信超时。转速模式本身不产生跟随误差 bit13
- `quick_stop_done/fault_reaction_done`：本地规划速度接近 0，并且实际速度 `0x606C` 接近 0
如果 G4 已离线，则只使用本地规划速度回零作为完成条件，避免故障反应永远卡住

### 10.3 DS402 给转速模式的输出

转速模式读取：

- `Ds402_IsOperationEnabled()`：允许把 `0x60FF` 斜坡成速度命令
- `Ds402_IsQuickStopActive()`：按 `0x6085` 急停减速度把速度目标收敛到 0
- `Ds402_IsFaultReactionActive()`：按 `0x605E` 选择 quick stop 或 disable
- `Ds402_IsFault()`：发送 disable

### 10.4 转速模式给 G4/control 的输出

当 DS402 处于 `OPERATION_ENABLED` 时，`generator.c` 发送 `CMD_TYPE_ENABLE`，目标值是经过梯形斜坡和速度限幅后的速度给定

当 DS402 处于 `QUICK_STOP_ACTIVE` 时，`generator.c` 按 `0x605A Quick stop option code` 选择 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`，目标值为 0

当 DS402 处于 `FAULT_REACTION_ACTIVE` 时，`generator.c` 按 `0x605E Fault reaction option code` 选择 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`，目标值为 0

当 DS402 处于 `FAULT` 时，`generator.c` 发送 `CMD_TYPE_DISABLE`，目标值为 0

当 DS402 尚未使能时，`generator.c` 默认发送 `CMD_TYPE_DISABLE`；如果当前状态和 option code 要求受控停机，则按 option code 选择 `CMD_TYPE_HALT`

### 10.5 转速模式给主站的输出

- `0x6041 Statusword`：由 DS402 统一生成
- `0x6061 Modes of operation display`：G4 反馈 mode
- `0x606B Velocity demand value`：内部速度需求
- `0x606C Velocity actual value`：G4 实际速度
- `0x6077 Torque actual value`：G4 实际转矩
- `0x6041 bit10`：目标速度已经追上，或目标速度和规划速度都接近 0

## 11. 转矩模式数据流

转矩模式对应 `0x6060 = 4`，由 `torque.c` 负责
它不走 `generator.c` 的位置/速度轨迹，而是独立读取转矩对象、执行限幅和 Q16 斜坡

### 11.1 输入来源

主站给 slave：

主站写 `0x6040 Controlword` 后，数据进入 `TestSlave_obj6040`
DS402 状态机用它完成状态跳转、quick stop 和 fault reset 上升沿识别；`torque.c` 同时读取 halt 位，用于转矩模式暂停处理

主站写 `0x6060 = 4` 后，数据进入 `TestSlave_obj6060`
`main.c` 据此选择 Profile Torque 模式，本周期由 `torque.c` 负责

主站写 `0x6071 Target torque` 后，数据进入 `TestSlave_obj6071`
`torque.c` 读取它作为主站原始目标转矩

主站写 `0x6072 Max torque` 后，数据进入 `TestSlave_obj6072`
`torque.c` 用它作为全局转矩绝对上限，防止目标转矩超过额定允许范围

主站写 `0x6087 Torque slope` 后，数据进入 `TestSlave_obj6087`
`torque.c` 用它控制内部转矩需求 `demand_q16` 的变化斜率

主站写 `0x60E0 Positive torque limit` 后，数据进入 `TestSlave_obj60E0`
`torque.c` 用它限制正方向最大转矩

主站写 `0x60E1 Negative torque limit` 后，数据进入 `TestSlave_obj60E1`
`torque.c` 用它限制负方向最大转矩


G4/control 给 slave：

G4 反馈帧里的 mode 字段由 `Torque_OnG4Feedback()` 写入 `0x6061`，主站用它观察 G4/control 实际执行模式

G4 反馈帧里的实际转矩由 `Torque_OnG4Feedback()` 写入 `0x6077`，主站用它观察 actual torque

G4 反馈帧里的主反馈字段在转矩模式下解释为实际速度，并写入 `0x606C`
`Torque_FillDs402Input()` 用 `0x606C` 辅助判断 quick stop 或 fault reaction 是否已经停稳

G4 反馈帧状态字节的 bit0 写入 `g_g4_fault_status`，作为 DS402 `local_fault` 的来源

G4 反馈是否持续到达由 `CAN_App_Check_G4_Watchdog()` 维护成 `g_g4_feedback_online` 和 `g_g4_comm_timeout`
`g_g4_feedback_online` 送给 DS402 的 `drive_online`，`g_g4_comm_timeout` 送给 DS402 的 `local_fault`

### 11.2 给 DS402 的输入

`Torque_FillDs402Input()` 给状态机以下输入：

- `controlword`：来自 `0x6040`
- `nmt_operational`：来自 `TestSlave_Data.nodeState`
- `drive_online`：来自 `g_g4_feedback_online`
- `local_fault`：`g_g4_fault_status || g_g4_comm_timeout`
- `fault_reset_allowed`：G4 故障和通信超时都消失后为 true
- `quick_stop_done/fault_reaction_done`：内部转矩需求 `demand_q16` 已经斜坡到 0，并且实际速度 `0x606C` 接近 0；如果 G4 已离线，则只要求 demand 回零

### 11.3 DS402 给转矩模式的输出

转矩模式读取：

- `Ds402_IsOperationEnabled()`：允许读取 `0x6071` 并输出转矩
- `Ds402_IsQuickStopActive()`：转矩需求向 0 斜坡
- `Ds402_IsFaultReactionActive()`：按 `0x605E` 选择 quick stop 或 disable，转矩需求向 0 斜坡
- `Ds402_IsFault()`：发送 disable

### 11.4 转矩模式给 G4/control 的输出

`Torque_Run()` 先把 `0x6071` 按 `0x6072/0x60E0/0x60E1` 限幅，再按 `0x6087` 做 Q16 斜坡，得到 `demand`

当 DS402 处于 `OPERATION_ENABLED` 时，`torque.c` 发送 `CMD_TYPE_ENABLE`，目标值是经过 `0x6072/0x60E0/0x60E1` 限幅并按 `0x6087` 斜坡后的转矩需求

当 DS402 处于 `QUICK_STOP_ACTIVE` 时，`torque.c` 把内部转矩需求向 0 斜坡
并按 `0x605A Quick stop option code` 选择发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`

当 DS402 处于 `FAULT_REACTION_ACTIVE` 时，`torque.c` 把内部转矩需求向 0 斜坡
并按 `0x605E Fault reaction option code` 选择发送 `CMD_TYPE_QUICKSTOP` 或 `CMD_TYPE_DISABLE`

当 DS402 处于 `FAULT` 时，`torque.c` 发送 `CMD_TYPE_DISABLE`，目标值为 0

当 DS402 尚未使能时，`torque.c` 默认发送 `CMD_TYPE_DISABLE`；如果当前状态和 option code 要求受控停机，则按 option code 选择 `CMD_TYPE_HALT`

### 11.5 转矩模式给主站的输出

- `0x6041 Statusword`：由 DS402 统一生成
- `0x6061 Modes of operation display`：G4 反馈 mode
- `0x6074 Torque demand value`：内部限幅和斜坡后的转矩需求
- `0x6077 Torque actual value`：G4 实际转矩
- `0x606C Velocity actual value`：G4 实际速度
- `0x6041 bit10`：转矩 demand 已经追上限幅后的目标

## 12. Option code 的去向

`0x605A~0x605E` 不由状态机直接执行。状态机只输出当前处于哪种 DS402 状态，运动模块读取状态后再根据 option code 选择 G4 命令

`0x605A Quick stop option code` 由 `generator.c` 和 `torque.c` 读取。
它只在 DS402 处于 `QUICK_STOP_ACTIVE` 时影响下游 G4 命令：值为 0 时发送 `CMD_TYPE_DISABLE`，非 0 时发送 `CMD_TYPE_QUICKSTOP`

`0x605B Shutdown option code` 由 `generator.c` 和 `torque.c` 读取。
它影响 `READY_TO_SWITCH_ON` 这类未运行但已进入 shutdown 后的状态：值为 0 时发送 `CMD_TYPE_DISABLE`，非 0 时发送 `CMD_TYPE_HALT`。

`0x605C Disable operation option code` 由 `generator.c` 和 `torque.c` 读取
它影响 `SWITCHED_ON` 状态，也就是从 operation enabled 去使能后仍保持 switched on 的阶段：值为 0 时发送 `CMD_TYPE_DISABLE`，非 0 时发送 `CMD_TYPE_HALT`

`0x605D Halt option code` 由 `generator.c` 和 `torque.c` 读取
它影响控制字 bit8 halt 请求：值为 0 时发送 `CMD_TYPE_DISABLE`，非 0 时发送 `CMD_TYPE_HALT`

`0x605E Fault reaction option code` 由 `generator.c` 和 `torque.c` 读取
它只在 DS402 处于 `FAULT_REACTION_ACTIVE` 时影响下游 G4 命令：值为 0 时发送 `CMD_TYPE_DISABLE`，非 0 时发送 `CMD_TYPE_QUICKSTOP`

## 13. 三种模式的共同边界

三种模式共享同一个 DS402 状态机实例。模式切换只会清各自运动规划状态，不会重新初始化 DS402 状态机，也不会自动清 fault 锁存。

共同规则如下：

- `0x6040/0x6041` 的标准状态跳转只在 `ds402_state.c` 中维护
- `generator.c` 和 `torque.c` 只问状态机当前能不能运行，不再自行拼完整状态字
- `0x6041` 只由 `Ds402_PublishStatusword()` 写回
- G4 故障和通信超时必须由 G4 反馈或 watchdog 自己恢复，不能被 fault reset 伪造清除
- 跟随误差属于位置模式软件锁存，只有 DS402 接受 fault reset 后才由 `Generator_OnDs402FaultReset()` 清除
- 转矩软状态在 DS402 接受 fault reset 后由 `Torque_OnDs402FaultReset()` 复位到 0 转矩

## 14. 外部联调时按什么观察

外部主站或调试人员可以按下面顺序理解和排查：

1. 看 `0x6041` 的基础状态位，按 DS402 mask 判断当前是 Not ready、Switch on disabled、Ready、Switched on、Operation enabled、Quick stop active、Fault reaction active 还是 Fault
2. 看 `0x6060` 和 `0x6061`，确认主站请求模式和 G4 实际回显模式是否一致
3. 看 `0x6064/0x606C/0x6077`，确认 G4 反馈是否还在刷新
4. 看 `g_g4_feedback_online/g_g4_comm_timeout` 对应表现：如果 G4 掉线，状态机会进入 fault reaction，随后进入 fault
5. 看模式相关输出：
   - 位置模式看 `0x6062`、`0x607A`、bit12、bit13
   - 转速模式看 `0x606B`、`0x60FF`、`0x606C`
   - 转矩模式看 `0x6074`、`0x6071`、`0x6077`
6. 如果进入 `FAULT`，主站必须先消除故障源，再让 `0x6040 bit7` 产生上升沿。保持 bit7 为 1 不会重复复位
