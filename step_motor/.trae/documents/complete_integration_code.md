# 补全步进电机云台追踪系统代码

## Context
当前工程各模块（UART解析、PID、云台追踪、步进电机驱动）已写好，但缺少集成代码和若干类型/逻辑问题，导致完整链路无法运行。

## 需要修改的内容

### 1. 添加 UART 中断服务函数
**文件**: `uart.c`
- 添加 `UART0_IRQHandler()`，在其中读取接收字节并调用 `UartParser_RxByte(byte)`
- UART0 中断已在 SysConfig 中使能（`DL_UART_MAIN_INTERRUPT_RX`），只需写 ISR

### 2. 修复 step_set_angle 参数类型
**文件**: `step_motor.h` + `step_motor.c`
- `step_set_angle` 的 `angle` 参数从 `uint8_t` 改为 `float`，匹配 `gimbal_tracker.c` 传入的 float 角度值
- 内部计算 `step_remain = (uint32_t)(angle / 0.05625)` 自然适配

### 3. 修复 step_set_speed 参数类型
**文件**: `step_motor.h` + `step_motor.c`
- `step_set_speed` 的 `speed` 参数从 `uint16_t` 改为 `float`，匹配以度/秒为单位的角速度
- 内部计算 `frequency = speed / 0.05625` 改为 `frequency = (uint32_t)(speed / 0.05625f)`

### 4. step_set_angle 中先设速度再启动
**文件**: `step_motor.c`
- `step_set_angle` 内部在启动前先调用 `step_set_speed` 设定默认速度，避免以 CC=0 启动

### 5. 补全 main 函数
**文件**: `empty.c`
- 初始化 `GimbalTracker_Init(0.05f)`
- 初始化 `UartParser_Init`，注册 `on_center` 回调，回调内调用 `GimbalTracker_Update`
- 使能 UART 中断 `NVIC_EnableIRQ(UART_0_INST_INT_IRQN)`
- 主循环调用 `UartParser_Process()`

### 6. step_motor_Init 中移除 EN 引脚操作
**文件**: `step_motor.c`
- 用户说明 EN/SLP 等引脚直接接 3.3V，不需要代码控制，移除 `DL_GPIO_setPins(EN)` 操作

## 验证
- 编译无错误
- 逻辑链完整：UART中断 → 环形缓冲 → 帧解析 → 回调 → PID → 角度累加 → step_set_angle → PWM脉冲
