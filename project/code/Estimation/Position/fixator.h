#ifndef _FIXATOR_H_
#define _FIXATOR_H_

#include "zf_common_headfile.h"

#define FIXATOR_MATCH_RADIUS_M (0.5f)
#define FIXATOR_NO_BEACON_INDEX (0xFFFFU)
#define FIXATOR_SOURCE_NONE (0U)
#define FIXATOR_SOURCE_AIR_BEACON (1U)
#define FIXATOR_SOURCE_BEACON_DETECTED (2U)

typedef struct
{
    uint8 pending_fix;
    uint8 last_match_valid;
    uint8 counts_in_sequence;       /* 1=计入信标序列，0=只做位置校准 */
    uint8 fix_source;               /* 0=无，1=Air 灭信标灯 */
    uint16 beacon_index;
    uint16 previous_beacon_index;
    float before_position[2];
    float fixed_position[2];
    float match_distance2_m2;
    uint32 fix_count;
    uint32 sequence_count;
} fixator_data_t;

extern fixator_data_t g_fixator;

void fixator_init(void);
void fixator_reset(void);
void fixator_update_100HZ(void);
uint8 fixator_get_position_fix(float position[2]);

#endif
