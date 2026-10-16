# DMG6220 STM32 HAL Driver

面向 STM32 HAL 的 DMG6220 CAN 云台电机驱动。代码保留了原工程中的位置速度模式、反馈解析、MIT 模式和周期性目标重发功能，同时移除了对 `CAN1`、APB1 时钟和固定 1 Mbps 参数的硬编码。

当前实现基于经典 bxCAN HAL API，适用于 STM32F1、STM32F4 等带 bxCAN 的系列。STM32G4、STM32H7 等使用 FDCAN 的系列需要单独做移植。

## 特性

- 基于 STM32 HAL 的 CAN 标准帧通信。
- 支持位置速度模式、原始位置/速度命令和 MIT 命令。
- 解析位置、速度、力矩、错误码和电机 ID。
- `DMG6220_MoveAngle()` 自动选择最短角度路径。
- `DMG6220_Service()` 每 10 ms 重发一次最近的命令。
- CAN 比特率和 GPIO 由 CubeMX 配置，驱动不会覆盖时钟参数。
- 不依赖 FreeRTOS、动态内存或 C++。

## 目录结构

```text
DMG6220_STM32_HAL/
├── Core/
│   ├── Inc/DMG6220.h          # 对外 API 和状态结构体
│   └── Src/DMG6220.c          # CAN 帧、解析和控制实现
├── docs/
│   ├── CUBEMX.md              # CubeMX 逐步配置说明
│   └── KEY_CODE.md            # 关键代码和协议拆解
├── examples/
│   └── dmg6220_stm32f407_example.c
├── LICENSE
└── README.md
```

## 硬件连接

DMG6220 使用 CAN 总线，STM32 不能直接连接电机线：

```text
STM32 CAN_TX  -> CAN 收发器 TXD
STM32 CAN_RX  <- CAN 收发器 RXD
CAN_H         -> 电机 CAN_H
CAN_L         -> 电机 CAN_L
GND           -> 电机和收发器共地
```

总线两端需要 `120 ohm` 终端电阻。首次测试建议使用带保护的 CAN 收发器，并先确认电源、使能脚和电机供电范围。

## CubeMX 配置摘要

完整步骤见 [docs/CUBEMX.md](docs/CUBEMX.md)。核心参数如下：

1. 启用 `CAN1`，普通模式，使用 11 位标准帧。
2. 将 CAN 引脚映射到实际硬件，F407 常见映射为 `PA11 = CAN1_RX`、`PA12 = CAN1_TX`。
3. CAN 波特率设置为 `1 Mbps`，采样点建议为 75% 到 87.5%。
4. 使能 `HAL_CAN_MODULE_ENABLED`，生成代码。
5. 不需要 CAN 中断也能使用本驱动，驱动通过 `CAN_RX_FIFO0` 轮询接收。

常见 1 Mbps 参数：

| CAN 时钟 | Prescaler | TimeSeg1 | TimeSeg2 | SJW | 波特率 | 采样点 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 MHz | 1 | 5 TQ | 2 TQ | 1 TQ | 1 Mbps | 75% |
| 42 MHz | 3 | 11 TQ | 2 TQ | 1 TQ | 1 Mbps | 85.7% |

计算公式：

```text
bitrate = CAN_clock / (Prescaler * (1 + TimeSeg1 + TimeSeg2))
```

例如 `8 MHz / (1 * (1 + 5 + 2)) = 1 Mbps`。

## 添加驱动

将以下目录复制到 CubeMX 工程：

```text
Core/Inc/DMG6220.h -> YourProject/Core/Inc/
Core/Src/DMG6220.c -> YourProject/Core/Src/
```

确认工程包含 `Core/Inc`，并将 `DMG6220.c` 加入编译。驱动使用：

- `main.h` 中的 HAL 类型。
- STM32 HAL 的 CAN API。
- `HAL_CAN_MODULE_ENABLED`。

## 最小使用示例

完整示例见 [examples/dmg6220_stm32f407_example.c](examples/dmg6220_stm32f407_example.c)。

```c
#include "main.h"
#include "DMG6220.h"

extern CAN_HandleTypeDef hcan1;

static DMG6220 gimbal;

void DMG6220_ExampleInit(void)
{
    if (DMG6220_Init(&gimbal, &hcan1, 0x01U) != HAL_OK)
    {
        Error_Handler();
    }

    if (DMG6220_Enable(&gimbal) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100U);

    if (DMG6220_WaitForPosition(&gimbal, 1000U) != HAL_OK)
    {
        Error_Handler();
    }

    (void)DMG6220_MoveAngle(&gimbal, 90.0f, 120.0f);
}

void DMG6220_ExampleLoop(void)
{
    (void)DMG6220_Update(&gimbal);
    (void)DMG6220_Service(&gimbal);
}
```

`DMG6220_Init()` 负责绑定已经由 CubeMX 初始化的 CAN 句柄、配置滤波器并启动 CAN。它不修改波特率、时钟或 GPIO。

## API

| API | 作用 |
| --- | --- |
| `DMG6220_Init()` | 绑定 `CAN_HandleTypeDef`、电机 ID，并启动 CAN |
| `DMG6220_Enable()` | 发送电机使能帧 |
| `DMG6220_Update()` | 从 RX FIFO 0 读完反馈并更新状态 |
| `DMG6220_GetPosition()` | 更新并返回角度，单位为度 |
| `DMG6220_WaitForPosition()` | 等待第一帧有效反馈 |
| `DMG6220_MoveAngle()` | 按最短路径转到目标角度，速度单位为度/秒 |
| `DMG6220_SendPosition()` | 发送原始位置和速度，单位为 rad 和 rad/s |
| `DMG6220_Service()` | 每 10 ms 重发最近的位置速度命令 |
| `DMG6220_SendMit()` | 发送 MIT 控制帧 |

## CAN 协议摘要

假设电机 CAN ID 为 `0x01`：

| 功能 | 标准帧 ID | 数据 |
| --- | ---: | --- |
| 使能 | `0x001` | `FF FF FF FF FF FF FF FC` |
| 位置速度命令 | `0x101` | 4 字节小端 `float position_rad` + 4 字节小端 `float velocity_rad_s` |
| 电机反馈 | `0x001` | 错误/ID、位置、速度、力矩等压缩数据 |
| MIT 命令 | `0x001` | 位置、速度、Kp、Kd、力矩压缩到 8 字节 |

驱动会读取 FIFO 中所有标准数据帧，并只接收 `data[0] & 0x0F == motor->can_id` 的反馈。详细拆解见 [docs/KEY_CODE.md](docs/KEY_CODE.md)。

## 电机端配置

使用达妙配置工具确认以下内容：

- CAN 波特率为 `1 Mbps`。
- CAN ID 与本工程 `DMG6220_Init()` 传入的 ID 一致。
- 使用 `DMG6220_MoveAngle()` 时，控制模式设置为位置速度模式。
- 使用 `DMG6220_SendMit()` 时，控制模式设置为 MIT 模式。
- 限制最大电流、速度和角度，先做空载测试。

## 使用注意事项

- `DMG6220_MoveAngle()` 的速度参数是最大机械速度，符号会被忽略。
- 目标角度会按最短路径折算到当前角度附近的等效位置。
- 必须在主循环中持续调用 `DMG6220_Update()` 和 `DMG6220_Service()`。
- 电机反馈频率和重发频率应与实际控制周期匹配；不要在其他任务中同时抢占同一个 RX FIFO。
- 多个电机共享同一个 CAN 句柄时，第一个 `DMG6220_Init()` 会启动 CAN，后续调用会复用已启动的总线。如果应用自己管理滤波器和启动流程，请在调用 `DMG6220_Init()` 前完成 CAN 初始化。
- 本驱动不负责自动处理总线关闭后的恢复策略；可根据系统需要读取 HAL CAN 状态并重新初始化。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 没有反馈 | 波特率、CAN_H/CAN_L、共地、终端电阻、收发器供电 |
| `DMG6220_Enable()` 返回错误 | CAN 是否启动、邮箱是否已满、电机 ID 是否正确 |
| `position_valid` 一直为 0 | 是否调用 `Update()`、反馈 ID 是否匹配、是否有其他任务抢读 FIFO |
| 电机不转 | 电机端是否为位置速度模式、是否使能、命令是否周期发送 |
| 位置抖动或丢帧 | 总线负载、线缆、终端电阻、`Service()` 调用周期 |

## 许可证

本项目使用 MIT License，详见 [LICENSE](LICENSE)。

Copyright (c) 2026 Bury
