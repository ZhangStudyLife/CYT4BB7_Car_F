#include "WS2812.h"

#define WS2812_BIT_CYCLES       (313U)
#define WS2812_T0H_CYCLES       (90U)
#define WS2812_T1H_CYCLES       (190U)
#define WS2812_RESET_DELAY_US   (80U)
#define WS2812_DWT_UNLOCK_KEY   (0xC5ACCE55U)

static uint8 s_ws2812_cycle_ready = 0U;

static void WS2812_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = WS2812_DWT_UNLOCK_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    system_delay(16U);
    s_ws2812_cycle_ready = (DWT->CYCCNT != 0U) ? 1U : 0U;
}

__STATIC_FORCEINLINE void WS2812_WaitUntil(uint32 target_cycle)
{
    while ((int32)(DWT->CYCCNT - target_cycle) < 0)
    {
    }
}

__STATIC_FORCEINLINE void WS2812_SendBit(uint8 bit)
{
    uint32 start_cycle;
    uint32 high_cycles = (bit != 0U) ? WS2812_T1H_CYCLES : WS2812_T0H_CYCLES;

    gpio_high(WS2812_DIN_PIN);
    start_cycle = DWT->CYCCNT;
    WS2812_WaitUntil(start_cycle + high_cycles);
    gpio_low(WS2812_DIN_PIN);
    WS2812_WaitUntil(start_cycle + WS2812_BIT_CYCLES);
}

static void WS2812_SendByte(uint8 value)
{
    uint8 mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        WS2812_SendBit((uint8)(value & mask));
    }
}

void WS2812_Init(void)
{
    WS2812_EnableCycleCounter();
    gpio_init(WS2812_DIN_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    system_delay_us(WS2812_RESET_DELAY_US);
}

void WS2812_SetRGB(uint8 red, uint8 green, uint8 blue)
{
    uint32 primask;

    if (s_ws2812_cycle_ready == 0U)
    {
        WS2812_EnableCycleCounter();
        if (s_ws2812_cycle_ready == 0U)
        {
            return;
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();

    WS2812_SendByte(green);
    WS2812_SendByte(red);
    WS2812_SendByte(blue);
    gpio_low(WS2812_DIN_PIN);

    __set_PRIMASK(primask);
    system_delay_us(WS2812_RESET_DELAY_US);
}

void WS2812_SetColor(uint32 rgb)
{
    WS2812_SetRGB((uint8)((rgb >> 16) & 0xFFU),
                  (uint8)((rgb >> 8) & 0xFFU),
                  (uint8)(rgb & 0xFFU));
}

void WS2812_Off(void)
{
    WS2812_SetRGB(0U, 0U, 0U);
}
