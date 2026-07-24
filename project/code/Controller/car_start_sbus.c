/* Compatibility wrapper.
 * Mode selection is centralized in car_mode.c: CH4 enables control, CH5/CH6
 * select MODE0~MODE8. Keep these APIs as read-only adapters for old callers.
 */
#include "car_start_sbus.h"
#include "car_loop.h"

car_start_sbus_state_e g_car_start_sbus_state = CAR_START_SBUS_STATE_INIT;

void car_start_sbus_init(void)
{
    car_start_sbus_reset();
}

void car_start_sbus_reset(void)
{
    g_car_start_sbus_state = CAR_START_SBUS_STATE_INIT;
}

void car_start_sbus_update_25HZ(void)
{
    g_car_start_sbus_state = (0U != car_control_enabled) ?
                             CAR_START_SBUS_STATE_RUNNING :
                             CAR_START_SBUS_STATE_STANDBY;
}

car_start_sbus_state_e car_start_sbus_get_state(void)
{
    return g_car_start_sbus_state;
}

uint8 car_start_sbus_get_mode(void)
{
    return (uint8)car_mode_get();
}

uint8 car_start_sbus_is_running(void)
{
    return (CAR_START_SBUS_STATE_RUNNING == g_car_start_sbus_state) ? 1U : 0U;
}

uint8 car_start_sbus_emergency_stop_active(void)
{
    return car_emergency_stop_active;
}
