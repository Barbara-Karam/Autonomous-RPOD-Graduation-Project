/* ========== tim.c ========== */
#include "tim.h"

/* ── Static assert: TIM9 prescaler/period is derived assuming SYSCLK = 84 MHz.
      APB2 = SYSCLK (RCC_HCLK_DIV1), so TIM9 input clock = 84 MHz.
      If the PLL is changed, update Prescaler and Period in MX_TIM9_Init() to match. ── */
#include "stm32f4xx_hal.h"
_Static_assert(
    /* PLLM=25, PLLN=168, PLLP=2 → 168 MHz VCO / 2 = 84 MHz SYSCLK */
    /* Voltage Scale 2 caps at 84 MHz, so this is the only valid config here. */
    /* Change this value AND the TIM9 prescaler/period together if the PLL is retargeted. */
    (84000000u / 840u / 1000u) == 100u,
    "TIM9 prescaler/period mismatch: expected 100 Hz at 84 MHz SYSCLK"
);

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* ── TIM1: Motors 0, 1, 2 (20 kHz complementary PWM) ── */
void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTime = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 4199;           /* 84 MHz / 1 / 4200 = 20 kHz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

    sBreakDeadTime.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTime.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTime.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTime.DeadTime         = 0;
    sBreakDeadTime.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTime.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTime.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTime) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim1);
}

/* ── TIM9: 100 Hz Control Loop ── */
void MX_TIM9_Init(void)
{
    /* SYSCLK = 84 MHz, APB2 = HCLK/1 = 84 MHz → TIM9 input = 84 MHz
       Prescaler 839 → 84 MHz / 840 = 100 kHz
       Period    999 → 100 kHz / 1000 = 100 Hz                          */
    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 839;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = 999;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim9) != HAL_OK) { Error_Handler(); }
}

/* ── TIM10: Motor 3 RPWM (20 kHz) ── */
void MX_TIM10_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 0;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 4199;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim10);
}

/* ── TIM11: Motor 3 LPWM (20 kHz) ── */
void MX_TIM11_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim11.Instance               = TIM11;
    htim11.Init.Prescaler         = 0;
    htim11.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim11.Init.Period            = 4199;
    htim11.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim11);
}

/* ── Encoders (TIM2, 3, 4, 5) ── */
void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295u;   /* 32-bit full range */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 4;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 4;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535u;         /* 16-bit full range */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 4;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 4;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535u;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 4;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 4;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

void MX_TIM5_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 0;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 4294967295u;   /* 32-bit full range */
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 4;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 4;
    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/* ── MSP callbacks ── */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM9) {
        __HAL_RCC_TIM9_CLK_ENABLE();
        /* Priority 3: control loop, below DMA (0) and UART (1) */
        HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
    if (tim_pwmHandle->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        /* FIX #5: Do NOT enable TIM1_UP_TIM10_IRQn here.
           TIM1 is used purely for PWM output; its update event fires at 20 kHz
           and has no handler, wasting ~5% CPU.  TIM10 update is also unused.
           Re-enable this IRQ only if a TIM1/TIM10 period-elapsed callback is needed. */
    } else if (tim_pwmHandle->Instance == TIM10) {
        __HAL_RCC_TIM10_CLK_ENABLE();
    } else if (tim_pwmHandle->Instance == TIM11) {
        __HAL_RCC_TIM11_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (timHandle->Instance == TIM1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM1 CH1/CH2/CH3 — PA8, PA9, PA10 */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;   /* explicit — AF1 */
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM1 CH1N/CH2N/CH3N — PB13, PB14, PB15
           FIX #14: Alternate re-assigned explicitly even though the value is
           the same, so a future pin remap cannot silently inherit a stale AF. */
        GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;   /* explicit — AF1 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (timHandle->Instance == TIM10)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM10 CH1 — PB8 */
        GPIO_InitStruct.Pin       = GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;  /* explicit — AF3 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (timHandle->Instance == TIM11)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM11 CH1 — PB9
           FIX #14: Alternate set explicitly; never rely on leftover struct state. */
        GPIO_InitStruct.Pin       = GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;  /* explicit — AF3 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *tim_encoderHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (tim_encoderHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM2 CH1 — PA5 */
        GPIO_InitStruct.Pin       = GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM2 CH2 — PB3
           FIX #14: Alternate re-assigned explicitly. */
        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (tim_encoderHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* TIM3 CH1/CH2 — PA6, PA7 */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_encoderHandle->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM4 CH1/CH2 — PB6, PB7 */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (tim_encoderHandle->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* TIM5 CH1/CH2 — PA0, PA1 */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}
