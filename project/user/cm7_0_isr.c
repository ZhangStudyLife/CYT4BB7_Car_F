/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */


#include "zf_common_headfile.h"

void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    air_comm_car_tick_1MS();
    tick_1000us_cnt++;
    car_loop_imu_1000HZ_isr();
}

void pit0_ch1_isr()
{
    pit_isr_flag_clear(PIT_CH1);
    car_loop_motion_100HZ_isr();
    menu_timer_handler();
    car_loop_release_background_100HZ_isr();
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
