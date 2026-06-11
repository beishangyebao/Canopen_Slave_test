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

## 3. 编译结果

已运行 Keil 命令行编译：

```text
D:\Keil\UV4\UV4.exe -b project\canopen-slave.uvprojx -j0 -o project\codex_ds402_build.log
```

编译日志：

```text
compiling main.c...
compiling can_app.c...
compiling bsp_can.c...
compiling TestSlave.c...
compiling ds402_state.c...
compiling torque.c...
compiling generator.c...
linking...
".\Objects\led.axf" - 0 Error(s), 0 Warning(s).
```

最终结果：`0 Error(s), 0 Warning(s)`。
