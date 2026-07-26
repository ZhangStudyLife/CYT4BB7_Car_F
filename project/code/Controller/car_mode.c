#include "car_mode.h"

void car_mode_init(void)
{
}

void car_mode_reset(void)
{
}

car_mode_e car_mode_get(void)
{
    return CAR_MODE_0;
}

void car_mode_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode_update_100HZ(uint32 now_ms)
{
    (void)now_ms;
}
