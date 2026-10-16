/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Bury
 */

#include "DMG6220.h"

#include <string.h>

#define DMG6220_DEG_TO_RAD 0.01745329252f
#define DMG6220_RAD_TO_DEG 57.29577951f
#define DMG6220_TWO_PI 6.283185307f
#define DMG6220_PI 3.141592654f
#define DMG6220_P_MAX 12.5f
#define DMG6220_V_MAX 45.0f
#define DMG6220_T_MAX 10.0f
#define DMG6220_MIN_MOVE_SPEED_RAD_S 0.1f

static float DMG6220_Clamp(float value, float minimum, float maximum)
{
    if (!(value > minimum))
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static uint16_t DMG6220_FloatToUint(float value, float minimum,
                                    float maximum, uint8_t bits)
{
    float span;
    uint32_t max_raw;

    value = DMG6220_Clamp(value, minimum, maximum);
    span = maximum - minimum;
    max_raw = (1UL << bits) - 1UL;

    return (uint16_t)((value - minimum) * (float)max_raw / span);
}

static float DMG6220_UintToFloat(uint16_t value, float minimum,
                                 float maximum, uint8_t bits)
{
    float span = maximum - minimum;
    uint32_t max_raw = (1UL << bits) - 1UL;

    return minimum + (float)value * span / (float)max_raw;
}

static void DMG6220_WriteFloatLe(uint8_t *data, float value)
{
    union
    {
        float value;
        uint32_t raw;
    } convert;

    convert.value = value;
    data[0] = (uint8_t)convert.raw;
    data[1] = (uint8_t)(convert.raw >> 8U);
    data[2] = (uint8_t)(convert.raw >> 16U);
    data[3] = (uint8_t)(convert.raw >> 24U);
}

static float DMG6220_WrapDelta(float delta)
{
    while (delta > DMG6220_PI)
    {
        delta -= DMG6220_TWO_PI;
    }

    while (delta < -DMG6220_PI)
    {
        delta += DMG6220_TWO_PI;
    }

    return delta;
}

static HAL_StatusTypeDef DMG6220_StartCan(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    /* Let applications with their own filters or CAN manager own startup. */
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

static HAL_StatusTypeDef DMG6220_TransmitFrame(DMG6220 *motor,
                                               uint32_t standard_id,
                                               const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;

    if ((motor == NULL) || (motor->hcan == NULL) || (data == NULL) ||
        (standard_id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    tx_header.StdId = standard_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(motor->hcan, &tx_header, (uint8_t *)data,
                               &mailbox);
}

HAL_StatusTypeDef DMG6220_Init(DMG6220 *motor, CAN_HandleTypeDef *hcan,
                              uint8_t can_id)
{
    if ((motor == NULL) || (hcan == NULL) || (can_id > 0x0FU))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->can_id = can_id;

    return DMG6220_StartCan(hcan);
}

HAL_StatusTypeDef DMG6220_Enable(DMG6220 *motor)
{
    uint8_t data[8];

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    memset(data, 0xFF, sizeof(data));
    data[7] = 0xFCU;

    return DMG6220_TransmitFrame(motor, motor->can_id, data);
}

HAL_StatusTypeDef DMG6220_Update(DMG6220 *motor)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    HAL_StatusTypeDef status = HAL_TIMEOUT;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    while (HAL_CAN_GetRxMessage(motor->hcan, CAN_RX_FIFO0, &rx_header,
                                data) == HAL_OK)
    {
        uint16_t position_raw;
        uint16_t velocity_raw;
        uint16_t torque_raw;

        if ((rx_header.StdId != motor->can_id) ||
            (rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA) ||
            (rx_header.DLC != 8U) ||
            ((data[0] & 0x0FU) != motor->can_id))
        {
            continue;
        }

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
        motor->position_valid = 1U;
        status = HAL_OK;
    }

    return status;
}

HAL_StatusTypeDef DMG6220_GetPosition(DMG6220 *motor, float *angle_deg)
{
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (angle_deg == NULL))
    {
        return HAL_ERROR;
    }

    status = DMG6220_Update(motor);
    if ((status == HAL_ERROR) || (motor->position_valid == 0U))
    {
        return HAL_ERROR;
    }

    *angle_deg = motor->position_rad * DMG6220_RAD_TO_DEG;
    return HAL_OK;
}

HAL_StatusTypeDef DMG6220_WaitForPosition(DMG6220 *motor,
                                         uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    do
    {
        if ((DMG6220_Update(motor) == HAL_OK) &&
            (motor->position_valid != 0U))
        {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef DMG6220_MoveAngle(DMG6220 *motor, float angle_deg,
                                    float speed_dps)
{
    float current_deg;
    float target_deg;
    float velocity_rad_s;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    DMG6220_Update(motor);
    if (motor->position_valid == 0U)
    {
        return HAL_ERROR;
    }

    if (speed_dps < 0.0f)
    {
        speed_dps = -speed_dps;
    }

    velocity_rad_s = speed_dps * DMG6220_DEG_TO_RAD;
    if (velocity_rad_s < DMG6220_MIN_MOVE_SPEED_RAD_S)
    {
        velocity_rad_s = DMG6220_MIN_MOVE_SPEED_RAD_S;
    }

    current_deg = motor->position_rad * DMG6220_RAD_TO_DEG;
    target_deg = current_deg +
                 DMG6220_WrapDelta((angle_deg - current_deg) *
                                   DMG6220_DEG_TO_RAD) *
                     DMG6220_RAD_TO_DEG;

    motor->target_rad = target_deg * DMG6220_DEG_TO_RAD;
    motor->target_velocity_rad_s = velocity_rad_s;
    motor->last_target_tick = HAL_GetTick();
    motor->target_valid = 1U;

    return DMG6220_SendPosition(motor, motor->target_rad,
                                motor->target_velocity_rad_s);
}

HAL_StatusTypeDef DMG6220_SendPosition(DMG6220 *motor, float position_rad,
                                      float velocity_rad_s)
{
    uint8_t data[8];

    if ((motor == NULL) || (motor->can_id > 0x0FU))
    {
        return HAL_ERROR;
    }

    DMG6220_WriteFloatLe(&data[0], position_rad);
    DMG6220_WriteFloatLe(&data[4], velocity_rad_s);

    return DMG6220_TransmitFrame(motor, 0x100U + motor->can_id, data);
}

HAL_StatusTypeDef DMG6220_Service(DMG6220 *motor)
{
    uint32_t now;

    if ((motor == NULL) || (motor->target_valid == 0U))
    {
        return HAL_OK;
    }

    now = HAL_GetTick();
    if ((now - motor->last_target_tick) >= 10U)
    {
        motor->last_target_tick = now;
        return DMG6220_SendPosition(motor, motor->target_rad,
                                    motor->target_velocity_rad_s);
    }

    return HAL_OK;
}

HAL_StatusTypeDef DMG6220_SendMit(DMG6220 *motor, float position,
                                 float velocity, float kp, float kd,
                                 float torque)
{
    uint8_t data[8];
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque_raw;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    position_raw = DMG6220_FloatToUint(position, -DMG6220_P_MAX,
                                       DMG6220_P_MAX, 16U);
    velocity_raw = DMG6220_FloatToUint(velocity, -DMG6220_V_MAX,
                                       DMG6220_V_MAX, 12U);
    kp_raw = DMG6220_FloatToUint(kp, 0.0f, 500.0f, 12U);
    kd_raw = DMG6220_FloatToUint(kd, 0.0f, 5.0f, 12U);
    torque_raw = DMG6220_FloatToUint(torque, -DMG6220_T_MAX,
                                     DMG6220_T_MAX, 12U);

    data[0] = (uint8_t)(position_raw >> 8U);
    data[1] = (uint8_t)position_raw;
    data[2] = (uint8_t)(velocity_raw >> 4U);
    data[3] = (uint8_t)(((velocity_raw & 0x0FU) << 4U) | (kp_raw >> 8U));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4U);
    data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) | (torque_raw >> 8U));
    data[7] = (uint8_t)torque_raw;

    return DMG6220_TransmitFrame(motor, motor->can_id, data);
}
