# 关键代码与协议说明

本文提取驱动中最重要的几段实现，便于维护、移植和审阅。完整代码以 [../Core/Src/DMG6220.c](../Core/Src/DMG6220.c) 为准。

## 1. 电机状态

```c
typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t can_id;
    uint8_t position_valid;
    uint8_t error;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float target_rad;
    float target_velocity_rad_s;
    uint32_t last_target_tick;
    uint8_t target_valid;
} DMG6220;
```

`Update()` 更新反馈字段，`MoveAngle()` 写入目标字段，`Service()` 按时间重发目标。

## 2. CAN 启动和滤波器

驱动只操作传入的 `CAN_HandleTypeDef`，不写死 `CAN1`，也不重设波特率：

```c
static HAL_StatusTypeDef DMG6220_StartCan(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    if (hcan->State == HAL_CAN_STATE_LISTENING)
    {
        return HAL_OK;
    }

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_CAN_Start(hcan);
}
```

这里使用全零 ID 和全零掩码，表示接收所有标准帧；真正的电机 ID 过滤在 `Update()` 中完成。

## 3. 使能帧

```c
memset(data, 0xFF, sizeof(data));
data[7] = 0xFCU;
return DMG6220_TransmitFrame(motor, motor->can_id, data);
```

数据内容为：

```text
FF FF FF FF FF FF FF FC
```

标准帧 ID 使用电机 CAN ID。

## 4. 位置速度命令

命令标准帧 ID 为 `0x100 + can_id`。数据是两个小端 `float`：

```c
DMG6220_WriteFloatLe(&data[0], position_rad);
DMG6220_WriteFloatLe(&data[4], velocity_rad_s);

return DMG6220_TransmitFrame(motor, 0x100U + motor->can_id, data);
```

小端编码等价于：

```text
data[0] = position byte 0 (LSB)
data[1] = position byte 1
data[2] = position byte 2
data[3] = position byte 3 (MSB)
data[4] = velocity byte 0 (LSB)
data[5] = velocity byte 1
data[6] = velocity byte 2
data[7] = velocity byte 3 (MSB)
```

## 5. 反馈解包

驱动先检查标准帧 ID、帧类型、DLC 和数据中的低 4 位电机 ID，再解压三个量：

```c
position_raw = ((uint16_t)data[1] << 8U) | data[2];
velocity_raw = ((uint16_t)data[3] << 4U) | (data[4] >> 4U);
torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8U) | data[5];

motor->position_rad = DMG6220_UintToFloat(position_raw,
                                           -DMG6220_P_MAX,
                                           DMG6220_P_MAX, 16U);
motor->velocity_rad_s = DMG6220_UintToFloat(velocity_raw,
                                             -DMG6220_V_MAX,
                                             DMG6220_V_MAX, 12U);
motor->torque_nm = DMG6220_UintToFloat(torque_raw, -DMG6220_T_MAX,
                                        DMG6220_T_MAX, 12U);
motor->error = data[0] >> 4U;
```

当前量程：

| 数据 | 原始位宽 | 范围 |
| --- | ---: | ---: |
| 位置 | 16 bit | `-12.5` 到 `12.5 rad` |
| 速度 | 12 bit | `-45.0` 到 `45.0 rad/s` |
| 力矩 | 12 bit | `-10.0` 到 `10.0 N*m` |

压缩值还原公式：

```text
value = minimum + raw * (maximum - minimum) / ((1 << bits) - 1)
```

## 6. 最短路径角度控制

角度差先折算到 `[-pi, pi]`，再从当前角度计算等效目标位置：

```c
target_deg = current_deg +
             DMG6220_WrapDelta((angle_deg - current_deg) *
                               DMG6220_DEG_TO_RAD) *
             DMG6220_RAD_TO_DEG;
```

因此传入 `280` 度时，不会无条件让电机跨越多个完整圈后到达反馈坐标中的同一角度，而是选择离当前位置最近的等效角度。

## 7. 周期重发

位置速度控制需要在主循环中持续调用 `Service()`：

```c
now = HAL_GetTick();
if ((now - motor->last_target_tick) >= 10U)
{
    motor->last_target_tick = now;
    return DMG6220_SendPosition(motor, motor->target_rad,
                                motor->target_velocity_rad_s);
}
```

`last_target_tick` 使用无符号差值计算，能正常处理 `HAL_GetTick()` 溢出。

## 8. MIT 命令打包

MIT 帧把一个位置的 16 位数据、一个速度的 12 位数据、Kp/Kd 和力矩拼成 8 字节：

```c
data[0] = (uint8_t)(position_raw >> 8U);
data[1] = (uint8_t)position_raw;
data[2] = (uint8_t)(velocity_raw >> 4U);
data[3] = (uint8_t)(((velocity_raw & 0x0FU) << 4U) | (kp_raw >> 8U));
data[4] = (uint8_t)kp_raw;
data[5] = (uint8_t)(kd_raw >> 4U);
data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) | (torque_raw >> 8U));
data[7] = (uint8_t)torque_raw;
```

MIT 可变范围：

| 量 | 范围 |
| --- | ---: |
| 位置 | `-12.5` 到 `12.5 rad` |
| 速度 | `-45.0` 到 `45.0 rad/s` |
| Kp | `0` 到 `500` |
| Kd | `0` 到 `5` |
| 力矩 | `-10.0` 到 `10.0 N*m` |

## 9. 调用关系

```text
DMG6220_Init
  -> DMG6220_StartCan
       -> HAL_CAN_ConfigFilter
       -> HAL_CAN_Start

DMG6220_Enable
  -> DMG6220_TransmitFrame(ID = can_id)

DMG6220_MoveAngle
  -> DMG6220_Update
  -> DMG6220_SendPosition
       -> DMG6220_TransmitFrame(ID = 0x100 + can_id)

DMG6220_Update
  -> HAL_CAN_GetRxMessage(CAN_RX_FIFO0)

DMG6220_Service
  -> DMG6220_SendPosition 每 10 ms
```
