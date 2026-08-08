#ifndef CAR_SAFETY_H
#define CAR_SAFETY_H

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CAR_SAFETY_FAULT_NONE = 0,
    CAR_SAFETY_FAULT_REMOTE_LOSS,
    CAR_SAFETY_FAULT_IMU_LOSS,
    CAR_SAFETY_FAULT_MAINTENANCE_ACTIVE,
    CAR_SAFETY_FAULT_STOP_NOT_DECELERATING,
    CAR_SAFETY_FAULT_LEFT_MOTOR_STALL,
    CAR_SAFETY_FAULT_RIGHT_MOTOR_STALL,
    CAR_SAFETY_FAULT_CONTINUOUS_ROTATION,
    CAR_SAFETY_FAULT_EXTREME_ROTATION
} car_safety_fault_e;

typedef struct
{
    uint8 link_up;
    uint8 imu_healthy;
    uint8 maintenance_active;
    uint8 run_switch_on;
    float left_target_speed;
    float right_target_speed;
    float left_feedback_speed;
    float right_feedback_speed;
    float left_motor_output;
    float right_motor_output;
    float gyroz_dps;
} car_safety_input_t;

void car_safety_init(void);
void car_safety_set_motion_scheduler_started(void);
uint8 car_safety_maintenance_acquire(void);
void car_safety_maintenance_release(void);
uint8 car_safety_is_maintenance_requested(void);
void car_safety_update_100HZ(const car_safety_input_t *input);
uint8 car_safety_is_output_allowed(void);
car_safety_fault_e car_safety_get_fault(void);

#ifdef __cplusplus
}
#endif

#endif
