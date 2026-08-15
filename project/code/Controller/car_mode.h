#ifndef CAR_MODE_H
#define CAR_MODE_H

#include "zf_common_headfile.h"

typedef enum
{
    CAR_MODE_0 = 0,
    CAR_MODE_1,
    CAR_MODE_2,
    CAR_MODE_3,
    CAR_MODE_4,
    CAR_MODE_5,
    CAR_MODE_6,
    CAR_MODE_7,
    CAR_MODE_8
} car_mode_e;

typedef struct
{
    float yaw_target_deg;
    float yaw_error_deg;
    float gyroz_target_dps;
    float gyroz_output;
    float yaw_p_term;
    float yaw_d_term;
    float yaw_output;
    float gyroz_p_term;
    float gyroz_i_term;
    float gyroz_ff_term;
    float left_speed_p_term;
    float left_speed_i_term;
    float left_speed_d_term;
    float left_speed_ff_term;
    float left_brake_ff;
    float right_speed_p_term;
    float right_speed_i_term;
    float right_speed_d_term;
    float right_speed_ff_term;
    float right_brake_ff;
    float large_turn_target_yaw_deg;
    float large_turn_target_speed_mps;
    uint16 large_turn_trigger_cycles;
    uint16 large_turn_finish_cycles;
    uint16 large_turn_elapsed_cycles;
    int8 large_turn_direction;
    uint8 large_turn_state;
    uint8 large_turn_rearm_required;
    uint8 speed_brake_active;
} car_drive_diag_t;

void car_mode_init(void);
void car_mode_reset(void);
void car_mode_reset_control(void);
car_mode_e car_mode_get(void);
uint8 car_mode_is_control_enabled(void);
uint8 car_mode_update_100HZ(uint32 now_ms);
void car_mode_get_diag(car_drive_diag_t *diag);

void car_mode0_init(void);
void car_mode0_reset(void);
void car_mode0_update_100HZ(uint32 now_ms);

void car_mode1_init(void);
void car_mode1_reset(void);
void car_mode1_update_100HZ(uint32 now_ms);
void car_mode1_get_diag(car_drive_diag_t *diag);

void car_mode2_init(void);
void car_mode2_reset(void);
void car_mode2_update_100HZ(uint32 now_ms);
void car_mode2_get_diag(car_drive_diag_t *diag);

void car_mode3_init(void);
void car_mode3_reset(void);
void car_mode3_update_100HZ(uint32 now_ms);

void car_mode4_init(void);
void car_mode4_reset(void);
void car_mode4_update_100HZ(uint32 now_ms);
void car_mode4_get_diag(car_drive_diag_t *diag);

void car_mode5_init(void);
void car_mode5_reset(void);
void car_mode5_update_100HZ(uint32 now_ms);
void car_mode5_get_diag(car_drive_diag_t *diag);

void car_mode6_init(void);
void car_mode6_reset(void);
void car_mode6_update_25HZ(uint32 now_ms);
void car_mode6_update_100HZ(uint32 now_ms);

void car_mode7_init(void);
void car_mode7_reset(void);
void car_mode7_update_100HZ(uint32 now_ms);

void car_mode8_init(void);
void car_mode8_reset(void);
void car_mode8_update_25HZ(uint32 now_ms);
void car_mode8_update_100HZ(uint32 now_ms);

#endif
