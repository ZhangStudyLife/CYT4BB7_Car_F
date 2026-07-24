#include "fixator.h"
#include "Estimation/Beacon_Detection/beacon_detection.h"

#define FIXATOR_EVENT_STRICT_RADIUS_M     (0.50f)
#define FIXATOR_EVENT_LOOSE_RADIUS_M      (0.90f)
#define FIXATOR_DUPLICATE_MIN_TICK        (1000U)
#define FIXATOR_BEACON_DETECTED_RADIUS_M  (0.20f)
#define FIXATOR_BEACON_DETECTED_DEAD_TICK (1000U)

#define FIXATOR_ALPHA_STRICT              (1.00f)
#define FIXATOR_ALPHA_LOOSE               (0.80f)
#define FIXATOR_ALPHA_DIRECT              (0.60f)

typedef enum
{
    FIXATOR_REASON_NONE = 0,
    FIXATOR_REASON_STRICT,
    FIXATOR_REASON_LOOSE,
    FIXATOR_REASON_DIRECT
} fixator_reason_t;

typedef struct
{
    uint16 beacon_index;
    float distance_m;
    fixator_reason_t reason;
} fixator_candidate_t;

fixator_data_t g_fixator = {0};

static uint32 s_last_fix_tick;
static uint32 s_last_beacon_detected_fix_tick;
static uint16 s_last_accepted_index;
static uint8 s_last_air_beacon_lost_flag;
static uint8 s_beacon_detected_fix_started;

static float fixator_distance2(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;

    return (dx * dx) + (dy * dy);
}

static uint8 fixator_get_beacon(uint16 index, beacon_config_point_t *beacon)
{
    if(index >= beacon_config_get_count())
    {
        return 0U;
    }

    return beacon_config_get_beacon(index, beacon);
}

static float fixator_distance_to_beacon(uint16 index, const float position[2])
{
    beacon_config_point_t beacon;

    if(fixator_get_beacon(index, &beacon) == 0U)
    {
        return 9999.0f;
    }

    return sqrtf(fixator_distance2(position[x], position[y], beacon.x, beacon.y));
}

static float fixator_alpha(fixator_reason_t reason)
{
    switch(reason)
    {
        case FIXATOR_REASON_STRICT:
            return FIXATOR_ALPHA_STRICT;
        case FIXATOR_REASON_LOOSE:
            return FIXATOR_ALPHA_LOOSE;
        case FIXATOR_REASON_DIRECT:
            return FIXATOR_ALPHA_DIRECT;
        default:
            return 0.0f;
    }
}

static uint8 fixator_find_nearest(const float position[2], fixator_candidate_t *candidate)
{
    uint16 count = beacon_config_get_count();
    uint16 i;
    uint8 found = 0U;
    float best_distance_m = 9999.0f;
    uint16 best_index = FIXATOR_NO_BEACON_INDEX;

    if(candidate == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < count; i++)
    {
        float distance_m = fixator_distance_to_beacon(i, position);

        if((found == 0U) || (distance_m < best_distance_m))
        {
            found = 1U;
            best_distance_m = distance_m;
            best_index = i;
        }
    }

    if(found == 0U)
    {
        return 0U;
    }

    candidate->beacon_index = best_index;
    candidate->distance_m = best_distance_m;
    candidate->reason = FIXATOR_REASON_NONE;

    return 1U;
}

static void fixator_clear_match_snapshot(void)
{
    g_fixator.last_match_valid = 0U;
    g_fixator.counts_in_sequence = 0U;
    g_fixator.fix_source = FIXATOR_SOURCE_NONE;
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    g_fixator.before_position[x] = g_odometer.position[x];
    g_fixator.before_position[y] = g_odometer.position[y];
    g_fixator.fixed_position[x] = g_odometer.position[x];
    g_fixator.fixed_position[y] = g_odometer.position[y];
    g_fixator.match_distance2_m2 = 0.0f;
}

static uint8 fixator_is_duplicate(uint16 beacon_index)
{
    uint32 dt = tick_1000us_cnt - s_last_fix_tick;

    if((s_last_accepted_index == beacon_index) && (dt <= FIXATOR_DUPLICATE_MIN_TICK))
    {
        return 1U;
    }

    return 0U;
}

static uint8 fixator_apply_candidate(const fixator_candidate_t *candidate,
                                    uint8 source,
                                    uint8 check_duplicate)
{
    beacon_config_point_t beacon;
    float alpha;

    if(candidate == NULL)
    {
        return 0U;
    }
    if((g_fixator.pending_fix != 0U) ||
       (fixator_get_beacon(candidate->beacon_index, &beacon) == 0U))
    {
        return 0U;
    }
    if((check_duplicate != 0U) &&
       (fixator_is_duplicate(candidate->beacon_index) != 0U))
    {
        return 0U;
    }

    alpha = fixator_alpha(candidate->reason);
    g_fixator.pending_fix = 1U;
    g_fixator.last_match_valid = 1U;
    g_fixator.counts_in_sequence = 1U;
    g_fixator.fix_source = source;
    g_fixator.beacon_index = candidate->beacon_index;
    g_fixator.previous_beacon_index = candidate->beacon_index;
    g_fixator.before_position[x] = g_odometer.position[x];
    g_fixator.before_position[y] = g_odometer.position[y];
    g_fixator.fixed_position[x] =
        g_odometer.position[x] + ((beacon.x - g_odometer.position[x]) * alpha);
    g_fixator.fixed_position[y] =
        g_odometer.position[y] + ((beacon.y - g_odometer.position[y]) * alpha);
    g_fixator.match_distance2_m2 = candidate->distance_m * candidate->distance_m;
    g_fixator.fix_count++;
    g_fixator.sequence_count++;
    s_last_fix_tick = tick_1000us_cnt;
    s_last_accepted_index = candidate->beacon_index;

    return 1U;
}

static uint8 fixator_accept_air_beacon_candidate(fixator_candidate_t *candidate)
{
    float position[2];

    if(candidate == NULL)
    {
        return 0U;
    }

    position[x] = g_odometer.position[x];
    position[y] = g_odometer.position[y];
    if(fixator_find_nearest(position, candidate) == 0U)
    {
        return 0U;
    }

    if(candidate->distance_m <= FIXATOR_EVENT_STRICT_RADIUS_M)
    {
        candidate->reason = FIXATOR_REASON_STRICT;
        return 1U;
    }

    if(candidate->distance_m <= FIXATOR_EVENT_LOOSE_RADIUS_M)
    {
        candidate->reason = FIXATOR_REASON_LOOSE;
        return 1U;
    }

    return 0U;
}

static uint8 fixator_accept_beacon_detected_candidate(fixator_candidate_t *candidate)
{
    float position[2];

    if(candidate == NULL)
    {
        return 0U;
    }

    position[x] = g_odometer.position[x];
    position[y] = g_odometer.position[y];
    if(fixator_find_nearest(position, candidate) == 0U)
    {
        return 0U;
    }

    if(candidate->distance_m <= FIXATOR_BEACON_DETECTED_RADIUS_M)
    {
        candidate->reason = FIXATOR_REASON_DIRECT;
        return 1U;
    }

    return 0U;
}

static void fixator_handle_air_beacon_event(void)
{
    fixator_candidate_t candidate;

    fixator_clear_match_snapshot();

    if(fixator_accept_air_beacon_candidate(&candidate) != 0U)
    {
        fixator_apply_candidate(&candidate, FIXATOR_SOURCE_AIR_BEACON, 1U);
    }
}

static uint8 fixator_beacon_detected_dead_zone_active(void)
{
    uint32 dt;

    if(s_beacon_detected_fix_started == 0U)
    {
        return 0U;
    }

    dt = tick_1000us_cnt - s_last_beacon_detected_fix_tick;
    return (dt < FIXATOR_BEACON_DETECTED_DEAD_TICK) ? 1U : 0U;
}

static void fixator_handle_beacon_detected_event(void)
{
    fixator_candidate_t candidate;

    if(g_fixator.pending_fix != 0U)
    {
        return;
    }

    fixator_clear_match_snapshot();

    if(fixator_beacon_detected_dead_zone_active() != 0U)
    {
        return;
    }

    if(fixator_accept_beacon_detected_candidate(&candidate) != 0U)
    {
        if(fixator_apply_candidate(&candidate,
                                   FIXATOR_SOURCE_BEACON_DETECTED,
                                   0U) != 0U)
        {
            s_last_beacon_detected_fix_tick = tick_1000us_cnt;
            s_beacon_detected_fix_started = 1U;
        }
    }
}

void fixator_init(void)
{
    fixator_reset();
}

void fixator_reset(void)
{
    memset(&g_fixator, 0, sizeof(g_fixator));
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    g_fixator.previous_beacon_index = FIXATOR_NO_BEACON_INDEX;
    s_last_fix_tick = 0U;
    s_last_beacon_detected_fix_tick = 0U;
    s_last_accepted_index = FIXATOR_NO_BEACON_INDEX;
    s_last_air_beacon_lost_flag = (g_air_beacon_lost_flag > 0.5f) ? 1U : 0U;
    s_beacon_detected_fix_started = 0U;
}

void fixator_update_100HZ(void)
{
    uint8 air_beacon_lost_flag = (g_air_beacon_lost_flag > 0.5f) ? 1U : 0U;
    uint8 beacon_detected_flag = (g_beacon_detection.enter_event != 0U) ? 1U : 0U;

    if((air_beacon_lost_flag != 0U) &&
       (s_last_air_beacon_lost_flag == 0U))
    {
        fixator_handle_air_beacon_event();
    }

    if(beacon_detected_flag != 0U)
    {
        fixator_handle_beacon_detected_event();
    }

    s_last_air_beacon_lost_flag = air_beacon_lost_flag;
}

uint8 fixator_get_position_fix(float position[2])
{
    if((position == NULL) || (g_fixator.pending_fix == 0U))
    {
        return 0U;
    }

    position[x] = g_fixator.fixed_position[x];
    position[y] = g_fixator.fixed_position[y];
    g_fixator.pending_fix = 0U;

    return 1U;
}
