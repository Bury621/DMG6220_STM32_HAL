# STM32CubeMX 配置

以下步骤以 `CAN1 + 1 Mbps + 标准帧 + 轮询接收` 为例。F407 上 `CAN1` 属于 APB1，其他 STM32 系列请在 CubeMX 的 Clock Configuration 中确认实际 CAN 时钟。

## 1. 启用 CAN

1. 打开 `Connectivity -> CAN1`。
2. `Mode` 选择 `Normal`。
3. 按开发板原理图分配引脚。常见 F407 映射：
   - `PA11 = CAN1_RX`
   - `PA12 = CAN1_TX`
4. 本驱动使用 11 位标准帧，不需要扩展帧。

如果使用外部 CAN 收发器，请同时连接收发器的 `TXD`、`RXD`、`VCC` 和 `GND`。

## 2. 配置 1 Mbps

在 `Parameter Settings` 中设置：

| 参数 | 建议值 |
| --- | --- |
| Mode | Normal |
| Prescaler | 根据 CAN 时钟计算 |
| Time Quanta in Bit Segment 1 | 根据 CAN 时钟计算 |
| Time Quanta in Bit Segment 2 | 根据 CAN 时钟计算 |
| ReSynchronization Jump Width | 1 TQ |
| Time Triggered Mode | Disable |
| Auto Bus Off | Enable |
| Auto Wake Up | Disable |
| Auto Retransmission | Enable |
| Receive FIFO Locked | Disable |
| Transmit FIFO Priority | Disable |

公式：

```text
bitrate = CAN_clock / (Prescaler * (1 + TimeSeg1 + TimeSeg2))
sample_point = (1 + TimeSeg1) / (1 + TimeSeg1 + TimeSeg2)
```

推荐参数：

| CAN 时钟 | Prescaler | TimeSeg1 | TimeSeg2 | SJW | 结果 | 采样点 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 MHz | 1 | 5 TQ | 2 TQ | 1 TQ | 1 Mbps | 75% |
| 42 MHz | 3 | 11 TQ | 2 TQ | 1 TQ | 1 Mbps | 85.7% |

以 8 MHz CAN 时钟为例，CubeMX 生成的代码应为：

```c
hcan1.Instance = CAN1;
hcan1.Init.Prescaler = 1;
hcan1.Init.Mode = CAN_MODE_NORMAL;
hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
hcan1.Init.TimeTriggeredMode = DISABLE;
hcan1.Init.AutoBusOff = ENABLE;
hcan1.Init.AutoWakeUp = DISABLE;
hcan1.Init.AutoRetransmission = ENABLE;
hcan1.Init.ReceiveFifoLocked = DISABLE;
hcan1.Init.TransmitFifoPriority = DISABLE;
```

如果 CAN 时钟是 42 MHz，则使用 `Prescaler = 3`、`TimeSeg1 = CAN_BS1_11TQ`、`TimeSeg2 = CAN_BS2_2TQ`。

## 3. 中断配置

本驱动默认轮询 `CAN_RX_FIFO0`，因此不要求打开 CAN 中断。若系统需要中断接收：

1. 在 `NVIC Settings` 中使能 `CAN1 RX0 interrupt`。
2. 在中断回调中只设置事件标志。
3. 在主任务或主循环中调用 `DMG6220_Update()` 读取 FIFO。

不要让多个任务同时调用 `DMG6220_Update()`，否则会竞争同一个 CAN RX FIFO。

## 4. 生成工程

1. `Project Manager` 中选择正确的工具链和工程路径。
2. 在 `Code Generator` 中按习惯选择只生成必要文件或重新生成文件。
3. 确认 `stm32f4xx_hal_conf.h` 中启用了：

```c
#define HAL_CAN_MODULE_ENABLED
```

4. 生成代码后，将本仓库的 `Core/Inc/DMG6220.h` 和 `Core/Src/DMG6220.c` 加入工程。

## 5. 初始化顺序

CubeMX 生成的 `main.c` 会先调用 `MX_CAN1_Init()`。用户代码按以下顺序执行：

```c
MX_CAN1_Init();

if (DMG6220_Init(&gimbal, &hcan1, 0x01U) != HAL_OK)
{
    Error_Handler();
}

if (DMG6220_Enable(&gimbal) != HAL_OK)
{
    Error_Handler();
}

HAL_Delay(100U);
```

`DMG6220_Init()` 会：

1. 保存 CAN 句柄和电机 ID。
2. 配置一个接收所有标准帧的滤波器。
3. 在 CAN 尚未启动时调用 `HAL_CAN_Start()`。
4. 不修改 CubeMX 生成的波特率、采样点和 GPIO。

如果应用已经自行配置了滤波器并调用了 `HAL_CAN_Start()`，驱动检测到 CAN 已处于 listening 状态后会跳过启动步骤。

## 6. 主循环

```c
while (1)
{
    (void)DMG6220_Update(&gimbal);
    (void)DMG6220_Service(&gimbal);

    HAL_Delay(1U);
}
```

实际工程中不要用长时间 `HAL_Delay()` 阻塞主循环。可以使用 1 ms 定时器、RTOS 任务或现有调度器调用上述两个函数。

## 7. 验证顺序

1. 只上电 STM32 和 CAN 收发器，不接电机，确认 CAN 时钟和引脚配置。
2. 用逻辑分析仪或 CAN 分析仪确认总线上有反馈帧。
3. 发送使能帧，确认电机进入使能状态。
4. 发送小角度、小速度命令，确认方向和角度范围。
5. 最后再接入云台机械负载并逐级提高限制。

## 常见配置错误

- CAN 时钟不是 8 MHz，却直接照抄 `Prescaler = 1, BS1 = 5TQ, BS2 = 2TQ`。
- CubeMX 默认生成的 CAN 参数没有修改，实际波特率不是 1 Mbps。
- 使用了扩展帧过滤器，但驱动发送和接收的是标准帧。
- `CAN_H` 和 `CAN_L` 接反，或两端都没有 120 ohm 终端电阻。
- STM32、CAN 收发器和电机电源没有共地。
- 多个模块同时抢占同一个 FIFO，导致反馈被其他代码读走。
