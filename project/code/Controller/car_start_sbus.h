#ifndef CAR_START_SBUS_H
#define CAR_START_SBUS_H

#include "zf_common_headfile.h"

typedef enum
{
    CAR_START_SBUS_STATE_INIT = 0,
    CAR_START_SBUS_STATE_STANDBY,
    CAR_START_SBUS_STATE_RUNNING
} car_start_sbus_state_e;

extern car_start_sbus_state_e g_car_start_sbus_state;

void car_start_sbus_init(void);
void car_start_sbus_reset(void);
void car_start_sbus_update_25HZ(void);
car_start_sbus_state_e car_start_sbus_get_state(void);
uint8 car_start_sbus_get_mode(void);
uint8 car_start_sbus_is_running(void);
uint8 car_start_sbus_emergency_stop_active(void);

#endif /* CAR_START_SBUS_H */
