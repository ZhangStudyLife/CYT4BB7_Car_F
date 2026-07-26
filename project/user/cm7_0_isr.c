

#include "zf_common_headfile.h"

static uint8_t pit_ch0_100HZ_count = 0;

void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    air_comm_car_tick_1MS();

    if(g_tick_1000HZ < 60000U)
    {
        g_tick_1000HZ++;
    }
    tick_1000us_cnt++;

    pit_ch0_100HZ_count++;
    if(pit_ch0_100HZ_count >= 10)
    {
        pit_ch0_100HZ_count = 0;
        timer_100HZ_flag = 1;
        menu_timer_handler();
    }

}

void pit0_ch1_isr()
{
    pit_isr_flag_clear(PIT_CH1);

}

void pit0_ch2_isr()
{
    pit_isr_flag_clear(PIT_CH2);

}

void pit0_ch10_isr()
{
    pit_isr_flag_clear(PIT_CH10);

}

void pit0_ch11_isr()
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}

void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))
    {

#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif

    }
    else
    {

    }
}

void uart1_isr (void)
{
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))
    {

        gnss_uart_callback();

    }
    else
    {

    }
}

void uart3_isr (void)
{
    uint8 dat;

    if(uart_isr_mask(UART_3))
    {
        while(uart_query_byte(UART_3, &dat))
        {
            air_comm_car_rx_byte(dat);
        }

    }
    else
    {

    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))
    {

        uart_receiver_handler();

    }
    else
    {

    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))
    {

    }
    else
    {

    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))
    {

    }
    else
    {

    }
}

void gpio_0_exti_isr()
{

}

void gpio_1_exti_isr()
{
    if(exti_flag_get(P01_0))
    {

    }
    if(exti_flag_get(P01_1))
    {

    }
}

void gpio_2_exti_isr()
{
    if(exti_flag_get(P02_0))
    {

    }
    if(exti_flag_get(P02_4))
    {

    }

}

void gpio_3_exti_isr()
{

}

void gpio_4_exti_isr()
{

}

void gpio_5_exti_isr()
{

}

void gpio_6_exti_isr()
{

}

void gpio_7_exti_isr()
{

}

void gpio_8_exti_isr()
{

}

void gpio_9_exti_isr()
{

}

void gpio_10_exti_isr()
{

}

void gpio_11_exti_isr()
{

}

void gpio_12_exti_isr()
{

}

void gpio_13_exti_isr()
{

}

void gpio_14_exti_isr()
{

}

void gpio_15_exti_isr()
{

}

void gpio_16_exti_isr()
{

}

void gpio_17_exti_isr()
{

}

void gpio_18_exti_isr()
{

}

void gpio_19_exti_isr()
{
    if(exti_flag_get(P19_1))
    {
    }

}

void gpio_20_exti_isr()
{

}

void gpio_21_exti_isr()
{

}

void gpio_22_exti_isr()
{

}

void gpio_23_exti_isr()
{

}
