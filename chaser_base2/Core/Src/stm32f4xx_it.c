/* ========== stm32f4xx_it.c ========== */
#include "main.h"
#include "stm32f4xx_it.h"
#include "control_loop.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
/* FIX #10: htim11 must be declared extern so the TIM11 update handler can
   dispatch through HAL_TIM_IRQHandler if needed in the future. */
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

/* ── Core / fault handlers ── */
void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void)        {}
void DebugMon_Handler(void)   {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    { HAL_IncTick(); }

/* ── DMA ── */
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_rx); }
void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_tx); }

/* ── TIM1 Break + TIM9 (shared vector) ──
   Priority 3.  TIM9 drives the 100 Hz control loop.
   TIM1 break is checked in the same ISR because they share TIM1_BRK_TIM9_IRQn. */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    /* 100 Hz control loop tick */
    if (__HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim9, TIM_IT_UPDATE) != RESET)
    {
        HAL_TIM_IRQHandler(&htim9);
    }

    /* TIM1 break fault (e.g. over-current trip) */
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_BREAK) != RESET)
    {
        HAL_TIM_IRQHandler(&htim1);
    }
}

/* ── TIM1 Update + TIM10 (shared vector) ──
   FIX #5 (companion to tim.c change): TIM1_UP and TIM10 update interrupts are
   NOT enabled (HAL_NVIC_EnableIRQ is not called for TIM1_UP_TIM10_IRQn in
   HAL_TIM_PWM_MspInit).  This handler is provided as a safety net only — it
   clears any spurious flags and returns immediately without invoking the HAL
   callback, so a mis-configuration cannot spin the CPU at 20 kHz. */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* Clear update flags without invoking callbacks — neither TIM1 update
       nor TIM10 update has a registered handler in this firmware. */
    __HAL_TIM_CLEAR_FLAG(&htim1,  TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);
}

/* ── TIM8 Trigger/Commutation + TIM11 (shared vector on STM32F401) ──
   FIX #10: TIM11 shares TIM8_TRG_COM_TIM11_IRQn.  Without this handler any
   TIM11 update interrupt (or spurious trigger) would reach the default handler
   and hang.  TIM11 is used only for PWM output so the handler just clears the
   update flag and returns.  If a TIM11 period-elapsed callback is ever needed,
   call HAL_TIM_IRQHandler(&htim11) here instead. */
void TIM8_TRG_COM_TIM11_IRQHandler(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);
}

/* ── USART2 ── */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* ── HAL period-elapsed callback ──
   Only TIM9 is registered; all other timers clear their flags in their own
   handlers above and do not reach this callback. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9) {
        ControlLoop_Run();
    }
}
