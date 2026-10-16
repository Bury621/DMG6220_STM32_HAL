/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Bury
 */

#include "main.h"
#include "DMG6220.h"

/* hcan1 is declared by CubeMX. Keep this extern only in this example file. */
extern CAN_HandleTypeDef hcan1;

static DMG6220 g_dmg6220;

void DMG6220_ExampleInit(void)
{
    if (DMG6220_Init(&g_dmg6220, &hcan1, 0x01U) != HAL_OK)
    {
        Error_Handler();
    }

    if (DMG6220_Enable(&g_dmg6220) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100U);

    if (DMG6220_WaitForPosition(&g_dmg6220, 1000U) != HAL_OK)
    {
        Error_Handler();
    }

    if (DMG6220_MoveAngle(&g_dmg6220, 90.0f, 120.0f) != HAL_OK)
    {
        Error_Handler();
    }
}

void DMG6220_ExampleLoop(void)
{
    (void)DMG6220_Update(&g_dmg6220);
    (void)DMG6220_Service(&g_dmg6220);
}

HAL_StatusTypeDef DMG6220_ExampleReadAngle(float *angle_deg)
{
    if (angle_deg == NULL)
    {
        return HAL_ERROR;
    }

    return DMG6220_GetPosition(&g_dmg6220, angle_deg);
}
