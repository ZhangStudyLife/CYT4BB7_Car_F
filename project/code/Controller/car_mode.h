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

void car_mode_init(void);
void car_mode_reset(void);
void car_mode_reset_control(void);
car_mode_e car_mode_get(void);
uint8 car_mode_is_control_enabled(void);
uint8 car_mode_update_100HZ(uint32 now_ms);

void car_mode0_init(void);
void car_mode0_reset(void);
void car_mode0_update_100HZ(uint32 now_ms);

void car_mode1_init(void);
void car_mode1_reset(void);
void car_mode1_update_100HZ(uint32 now_ms);

void car_mode2_init(void);
void car_mode2_reset(void);
void car_mode2_update_25HZ(uint32 now_ms);
void car_mode2_update_100HZ(uint32 now_ms);

void car_mode3_init(void);
void car_mode3_reset(void);
void car_mode3_update_25HZ(uint32 now_ms);
void car_mode3_update_100HZ(uint32 now_ms);

void car_mode4_init(void);
void car_mode4_reset(void);
void car_mode4_update_25HZ(uint32 now_ms);
void car_mode4_update_100HZ(uint32 now_ms);

void car_mode5_init(void);
void car_mode5_reset(void);
void car_mode5_update_25HZ(uint32 now_ms);
void car_mode5_update_100HZ(uint32 now_ms);

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
