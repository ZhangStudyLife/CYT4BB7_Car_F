#include "beacon_detection.h"

#define BEACON_DETECTION_IMU_DT_S                 (0.001f)
#define BEACON_DETECTION_IMU_WINDOW_SIZE          (24U)
#define BEACON_DETECTION_WHEEL_HPF_TAU_S          (0.08f)
#define BEACON_DETECTION_STARTUP_TICKS            (0U)
#define BEACON_DETECTION_EVENT_HOLD_TICKS         (120U)
#define BEACON_DETECTION_ENTER_CONFIRM_TICKS      (1100U)
#define BEACON_DETECTION_ENTER_SEGMENT_GAP_TICKS  (450U)
#define BEACON_DETECTION_PENDING_ENTER_MAX        (4U)
#define BEACON_DETECTION_POST_SPEED_START_TICKS   (120U)
#define BEACON_DETECTION_TAIL_ACCEL_START_TICKS   (200U)
#define BEACON_DETECTION_ON_MIN_TICKS             (280U)
#define BEACON_DETECTION_EXIT_SEARCH_TICKS        (2800U)
#define BEACON_DETECTION_PAIR_MIN_TICKS           (195U)
#define BEACON_DETECTION_PAIR_MAX_TICKS           (1005U)
#define BEACON_DETECTION_WEAK_EARLY_TICKS         (800U)
#define BEACON_DETECTION_FAST_EXIT_GATE_MIN_TICKS (250U)
#define BEACON_DETECTION_FAST_EXIT_GATE_MAX_TICKS (320U)
#define BEACON_DETECTION_STRONG_TAIL_POSE_TICKS   (700U)
#define BEACON_DETECTION_SIDE_TAIL_POSE_TICKS     (650U)
#define BEACON_DETECTION_WEAK_CLEAN_TAIL_TICKS    (700U)
#define BEACON_DETECTION_CLUSTER_GAP_TICKS        (750U)
#define BEACON_DETECTION_CANDIDATE_PEAK_GAP_TICKS (80U)
#define BEACON_DETECTION_MID_STARTUP_TICKS        (4000U)
#define BEACON_DETECTION_STRONG_TAIL_STARTUP_TICKS (3000U)
#define BEACON_DETECTION_EXPIRED_RESEED_SCORE     (2.35f)
#define BEACON_DETECTION_BASELINE_ALPHA           (0.001f)
#define BEACON_DETECTION_BASELINE_FLOOR           (0.05f)
#define BEACON_DETECTION_MOTION_REF_DECAY         (0.995f)
#define BEACON_DETECTION_EPSILON                  (1.0e-6f)

typedef struct
{
    float tilt_deg;
    float tilt_rate_dps;
    float roll_rate_dps;
    float pitch_rate_dps;
    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float accel_norm_error_g;
    float impact_score;
} beacon_detection_imu_sample_t;

typedef struct
{
    uint8_t valid;
    uint8_t suppressed;
    uint16_t age_ticks;
    uint16_t quiet_ticks;

    beacon_detection_imu_sample_t peak;
    beacon_bump_location_t peak_location;
    beacon_bump_location_t pre_location;
    beacon_bump_location_t post_location;
    float peak_z;
    float pre_speed_mps;
    float post_min_speed_mps;
    float post_speed_average_mps;
    float tail_accel_max_g;
    float wheel_highpass_sum_max;

} beacon_detection_enter_candidate_t;

typedef struct
{
    uint8_t is_enter_event;
    uint8_t on_beacon_state;
    beacon_detection_imu_sample_t sample;
    beacon_bump_location_t location;
} beacon_detection_event_item_t;

typedef struct
{
    uint8_t active;
    uint8_t segment_active;
    uint8_t segment_pending;
    uint8_t exit_valid;
    uint8_t early_accepted;
    uint8_t peak_count;
    uint16_t enter_age_ticks;
    uint16_t last_high_gap_ticks;
    uint16_t last_peak_age_ticks;
    uint16_t segment_peak_age_ticks;
    uint16_t duration_ticks;
    uint16_t exit_age_ticks;

    beacon_detection_imu_sample_t enter_sample;
    beacon_detection_imu_sample_t early_enter_sample;
    beacon_detection_imu_sample_t segment_peak_sample;
    beacon_detection_imu_sample_t exit_sample;
    beacon_detection_imu_sample_t early_exit_sample;
    beacon_bump_location_t enter_location;
    beacon_bump_location_t early_enter_location;
    beacon_bump_location_t segment_peak_location;
    beacon_bump_location_t exit_location;
    beacon_bump_location_t early_exit_location;

    float first_speed_mps;
    float segment_peak_roll_deg;
    float segment_peak_pitch_deg;
    float pose_roll_min_deg;
    float pose_roll_max_deg;
    float pose_pitch_min_deg;
    float pose_pitch_max_deg;
    float segment_peak_score;
    float segment_peak_speed_mps;
    float segment_peak_wheel_highpass_count;
    float max_score;
    float max_gyro_xy_dps;
    float max_gyro_z_abs_dps;
    float early_max_gyro_z_abs_dps;
    float max_wheel_highpass_count;
    float window_max_wheel_highpass_count;
    float win_gyro_xy_dps;
    float exit_score;
    float exit_accel_norm_error_g;
    uint32_t enter_runtime_ticks;
} beacon_detection_realtime_candidate_t;

beacon_detection_data_t g_beacon_detection;

static beacon_detection_imu_sample_t s_imu_history[BEACON_DETECTION_IMU_WINDOW_SIZE];
static beacon_detection_enter_candidate_t s_active_enter;
static beacon_detection_enter_candidate_t s_pending_enter[BEACON_DETECTION_PENDING_ENTER_MAX];
static beacon_detection_event_item_t s_event_queue[BEACON_DETECTION_PENDING_ENTER_MAX];
static beacon_detection_imu_sample_t s_exit_peak;
static beacon_detection_imu_sample_t s_exit_min_speed_sample;
static beacon_bump_location_t s_exit_peak_location;
static beacon_bump_location_t s_exit_min_speed_location;
static float s_exit_min_speed_mps;
static uint16_t s_exit_age_ticks;
static uint8_t s_exit_track_active;
static uint8_t s_exit_peak_valid;
static uint8_t s_exit_min_speed_valid;
static beacon_detection_imu_sample_t s_deferred_exit_sample;
static beacon_bump_location_t s_deferred_exit_location;
static uint8_t s_deferred_exit_valid;
static float s_motion_reference_vel[2];
static float s_motion_reference_speed_mps;
static float s_roll_zero_deg;
static float s_pitch_zero_deg;
static float s_prev_roll_deg;
static float s_prev_pitch_deg;
static float s_wheel_lpf[4];
static float s_wheel_highpass[4];
static float s_impact_baseline;
static float s_impact_deviation;
static uint16_t s_startup_ticks;
static uint32_t s_runtime_ticks;
static uint16_t s_event_ticks;
static uint8_t s_event_queue_head;
static uint8_t s_event_queue_tail;
static uint8_t s_event_queue_count;
static uint8_t s_on_beacon_state;
static uint16_t s_realtime_cooldown_ticks;
static uint8_t s_imu_history_index;
static uint8_t s_imu_history_count;
static uint8_t s_tilt_ready;
static beacon_detection_realtime_candidate_t s_realtime_candidate;

static float beacon_detection_vec2_norm(float x_value, float y_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value));
}

static float beacon_detection_vec3_norm(float x_value, float y_value, float z_value)
{
    return sqrtf((x_value * x_value) + (y_value * y_value) + (z_value * z_value));
}

static float beacon_detection_ratio(float value, float threshold)
{
    return value / (threshold + BEACON_DETECTION_EPSILON);
}

static void beacon_detection_clear_history(void)
{
    memset(s_imu_history, 0, sizeof(s_imu_history));
    s_imu_history_index = 0U;
    s_imu_history_count = 0U;
}

static void beacon_detection_push_imu_sample(const beacon_detection_imu_sample_t *sample)
{
    s_imu_history[s_imu_history_index] = *sample;
    s_imu_history_index++;
    if(s_imu_history_index >= BEACON_DETECTION_IMU_WINDOW_SIZE)
    {
        s_imu_history_index = 0U;
    }

    if(s_imu_history_count < BEACON_DETECTION_IMU_WINDOW_SIZE)
    {
        s_imu_history_count++;
    }
}

static void beacon_detection_get_imu_window_max(beacon_detection_imu_sample_t *window)
{
    uint8_t i;

    memset(window, 0, sizeof(*window));
    for(i = 0U; i < s_imu_history_count; i++)
    {
        window->tilt_deg = car_math_maxf(window->tilt_deg, s_imu_history[i].tilt_deg);
        window->tilt_rate_dps = car_math_maxf(window->tilt_rate_dps, s_imu_history[i].tilt_rate_dps);
        window->roll_rate_dps = car_math_maxf(window->roll_rate_dps, s_imu_history[i].roll_rate_dps);
        window->pitch_rate_dps = car_math_maxf(window->pitch_rate_dps, s_imu_history[i].pitch_rate_dps);
        window->gyro_xy_dps = car_math_maxf(window->gyro_xy_dps, s_imu_history[i].gyro_xy_dps);
        window->gyro_z_abs_dps = car_math_maxf(window->gyro_z_abs_dps, s_imu_history[i].gyro_z_abs_dps);
        window->accel_norm_error_g =
            car_math_maxf(window->accel_norm_error_g, s_imu_history[i].accel_norm_error_g);
        window->impact_score = car_math_maxf(window->impact_score, s_imu_history[i].impact_score);
    }
}

static uint8_t beacon_detection_wheel_mask_from_location(beacon_bump_location_t location)
{
    uint8_t mask;

    mask = 0U;
    switch(location)
    {
        case BEACON_BUMP_LOCATION_FRONT:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_RF_MASK;
            break;
        case BEACON_BUMP_LOCATION_RIGHT:
            mask = BEACON_BUMP_WHEEL_RF_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        case BEACON_BUMP_LOCATION_LEFT:
            mask = BEACON_BUMP_WHEEL_LF_MASK | BEACON_BUMP_WHEEL_LR_MASK;
            break;
        case BEACON_BUMP_LOCATION_REAR:
            mask = BEACON_BUMP_WHEEL_LR_MASK | BEACON_BUMP_WHEEL_RR_MASK;
            break;
        default:
            mask = 0U;
            break;
    }

    return mask;
}

static beacon_bump_location_t beacon_detection_location_from_motion(float forward_velocity,
                                                                    float strafe_velocity)
{
    float forward_abs;
    float strafe_abs;
    beacon_bump_location_t location;

    forward_abs = car_math_absf(forward_velocity);
    strafe_abs = car_math_absf(strafe_velocity);
    location = BEACON_BUMP_LOCATION_UNKNOWN;

    if((forward_abs < 0.08f) && (strafe_abs < 0.08f))
    {
        return BEACON_BUMP_LOCATION_UNKNOWN;
    }

    if(forward_abs >= strafe_abs)
    {
        location = (forward_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_FRONT :
                   BEACON_BUMP_LOCATION_REAR;
    }
    else
    {
        location = (strafe_velocity >= 0.0f) ?
                   BEACON_BUMP_LOCATION_RIGHT :
                   BEACON_BUMP_LOCATION_LEFT;
    }

    return location;
}

static beacon_bump_location_t beacon_detection_location_from_reference(void)
{
    return beacon_detection_location_from_motion(s_motion_reference_vel[0],
                                                s_motion_reference_vel[1]);
}

static uint8_t beacon_detection_locations_match(beacon_bump_location_t first,
                                                beacon_bump_location_t second)
{
    if((first == BEACON_BUMP_LOCATION_UNKNOWN) || (second == BEACON_BUMP_LOCATION_UNKNOWN))
    {
        return 0U;
    }

    return (first == second) ? 1U : 0U;
}

static float beacon_detection_wheel_highpass_sum_abs(void)
{
    return car_math_absf(s_wheel_highpass[0]) +
           car_math_absf(s_wheel_highpass[1]) +
           car_math_absf(s_wheel_highpass[2]) +
           car_math_absf(s_wheel_highpass[3]);
}

static void beacon_detection_clear_enter_candidate(beacon_detection_enter_candidate_t *candidate)
{
    memset(candidate, 0, sizeof(*candidate));
    candidate->peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
    candidate->pre_location = BEACON_BUMP_LOCATION_UNKNOWN;
    candidate->post_location = BEACON_BUMP_LOCATION_UNKNOWN;
}

static void beacon_detection_init_enter_candidate(beacon_detection_enter_candidate_t *candidate,
                                                  const beacon_detection_imu_sample_t *window)
{
    beacon_detection_clear_enter_candidate(candidate);
    candidate->valid = 1U;
    candidate->peak = *window;
    candidate->peak_location =
        beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                              g_beacon_detection.vel[1]);
    candidate->pre_location = beacon_detection_location_from_reference();
    candidate->post_location = candidate->peak_location;
    candidate->peak_z = g_beacon_detection.impact_robust_z;
    candidate->pre_speed_mps = car_math_maxf(g_beacon_detection.speed_mps,
                                             s_motion_reference_speed_mps);
    candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
    candidate->post_speed_average_mps = g_beacon_detection.speed_mps;
    candidate->tail_accel_max_g = 0.0f;
    candidate->wheel_highpass_sum_max = beacon_detection_wheel_highpass_sum_abs();
}

static void beacon_detection_observe_enter_candidate(beacon_detection_enter_candidate_t *candidate,
                                                     const beacon_detection_imu_sample_t *window)
{
    beacon_bump_location_t location_now;
    float wheel_highpass_sum;

    if(candidate->valid == 0U)
    {
        return;
    }

    candidate->age_ticks++;
    candidate->quiet_ticks++;
    location_now = beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                         g_beacon_detection.vel[1]);
    if(window->impact_score > candidate->peak.impact_score)
    {
        candidate->peak = *window;
        candidate->peak_location = location_now;
        candidate->peak_z = g_beacon_detection.impact_robust_z;
        candidate->age_ticks = 0U;
        candidate->quiet_ticks = 0U;
        candidate->pre_location = beacon_detection_location_from_reference();
        candidate->post_location = location_now;
        candidate->pre_speed_mps = car_math_maxf(g_beacon_detection.speed_mps,
                                                 s_motion_reference_speed_mps);
        candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
        candidate->post_speed_average_mps = g_beacon_detection.speed_mps;
        candidate->tail_accel_max_g = 0.0f;
        candidate->wheel_highpass_sum_max = beacon_detection_wheel_highpass_sum_abs();
    }

    if(candidate->age_ticks >= BEACON_DETECTION_POST_SPEED_START_TICKS)
    {
        if(g_beacon_detection.speed_mps < candidate->post_min_speed_mps)
        {
            candidate->post_min_speed_mps = g_beacon_detection.speed_mps;
        }
        candidate->post_speed_average_mps +=
            0.02f * (g_beacon_detection.speed_mps - candidate->post_speed_average_mps);

        if(location_now != BEACON_BUMP_LOCATION_UNKNOWN)
        {
            candidate->post_location = location_now;
        }
    }

    if(candidate->age_ticks >= BEACON_DETECTION_TAIL_ACCEL_START_TICKS)
    {
        candidate->tail_accel_max_g =
            car_math_maxf(candidate->tail_accel_max_g, window->accel_norm_error_g);
    }

    wheel_highpass_sum = beacon_detection_wheel_highpass_sum_abs();
    candidate->wheel_highpass_sum_max =
        car_math_maxf(candidate->wheel_highpass_sum_max, wheel_highpass_sum);
}

static void beacon_detection_update_adaptive_baseline(float impact_score)
{
    float error;

    error = impact_score - s_impact_baseline;
    s_impact_baseline += BEACON_DETECTION_BASELINE_ALPHA * error;
    s_impact_deviation += BEACON_DETECTION_BASELINE_ALPHA *
                          (car_math_absf(error) - s_impact_deviation);
    if(s_impact_deviation < BEACON_DETECTION_BASELINE_FLOOR)
    {
        s_impact_deviation = BEACON_DETECTION_BASELINE_FLOOR;
    }

    g_beacon_detection.impact_baseline = s_impact_baseline;
    g_beacon_detection.impact_robust_z =
        (impact_score - s_impact_baseline) / s_impact_deviation;
}

static uint8_t beacon_detection_is_main_enter_seed(const beacon_detection_imu_sample_t *window)
{
    uint8_t wheel_stop_candidate;
    uint8_t front_fast_candidate;

    wheel_stop_candidate = ((beacon_detection_wheel_highpass_sum_abs() >= 600.0f) &&
                            (s_motion_reference_speed_mps >= 1.40f)) ? 1U : 0U;
    front_fast_candidate = ((window->impact_score >= 1.45f) &&
                            (g_beacon_detection.impact_robust_z >= 4.0f) &&
                            (beacon_detection_wheel_highpass_sum_abs() < 180.0f)) ? 1U : 0U;

    return ((window->impact_score >= 0.75f) ||
            (wheel_stop_candidate != 0U) ||
            (front_fast_candidate != 0U)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_weak_enter_seed(const beacon_detection_imu_sample_t *window)
{
    return ((window->impact_score >= 0.35f) &&
            (window->impact_score < 0.75f)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_enter_valid(const beacon_detection_enter_candidate_t *candidate,
                                               beacon_bump_location_t *location)
{
    uint8_t adaptive_hit;
    uint8_t low_speed_hit;
    uint8_t strong_hit;
    uint8_t weak_low_speed_hit;
    uint8_t front_fast_hit;
    uint8_t all_wheel_stop_hit;
    uint8_t unstable_duplicate;
    uint8_t hard_stop_noise;
    uint8_t spin_noise;

    *location = candidate->peak_location;
    if((*location == BEACON_BUMP_LOCATION_UNKNOWN) &&
       (candidate->pre_location != BEACON_BUMP_LOCATION_UNKNOWN))
    {
        *location = candidate->pre_location;
    }

    adaptive_hit = ((candidate->peak.impact_score >= 1.10f) &&
                    (candidate->pre_speed_mps <= 1.20f) &&
                    (candidate->peak_z >= 2.0f)) ? 1U : 0U;
    low_speed_hit = ((candidate->peak.impact_score >= 1.25f) &&
                     (candidate->post_speed_average_mps <= 0.25f) &&
                     (candidate->peak.gyro_z_abs_dps <= 45.0f)) ? 1U : 0U;
    strong_hit = ((candidate->peak.impact_score >= 1.90f) &&
                  (candidate->peak.gyro_z_abs_dps <= 25.0f)) ? 1U : 0U;
    weak_low_speed_hit = ((candidate->peak.impact_score >= 0.70f) &&
                          (candidate->peak_z >= 2.0f) &&
                          (candidate->peak.gyro_z_abs_dps <= 5.0f) &&
                          (candidate->pre_speed_mps <= 1.00f) &&
                          (candidate->post_speed_average_mps <= 0.80f) &&
                          (candidate->post_min_speed_mps >= 0.05f) &&
                          (candidate->post_min_speed_mps <= 0.80f) &&
                          (candidate->tail_accel_max_g <= 0.18f)) ? 1U : 0U;
    front_fast_hit = ((candidate->peak.impact_score >= 1.45f) &&
                      (candidate->peak_z >= 4.0f) &&
                      (candidate->peak.gyro_z_abs_dps <= 45.0f) &&
                      (candidate->peak.accel_norm_error_g <= 0.20f) &&
                      (candidate->pre_speed_mps >= 1.20f) &&
                      (candidate->post_speed_average_mps <= 0.90f) &&
                      (candidate->post_min_speed_mps <= 0.35f) &&
                      (candidate->tail_accel_max_g <= 0.25f) &&
                      (beacon_detection_locations_match(candidate->peak_location,
                                                        BEACON_BUMP_LOCATION_FRONT) != 0U) &&
                      (beacon_detection_locations_match(candidate->pre_location,
                                                        BEACON_BUMP_LOCATION_FRONT) != 0U) &&
                      (candidate->wheel_highpass_sum_max < 180.0f)) ? 1U : 0U;
    all_wheel_stop_hit = ((candidate->peak.impact_score >= 1.20f) &&
                          (candidate->peak_z >= 4.0f) &&
                          (candidate->peak.gyro_z_abs_dps <= 8.0f) &&
                          (candidate->pre_speed_mps >= 1.40f) &&
                          (candidate->post_speed_average_mps <= 0.10f) &&
                          (candidate->post_min_speed_mps <= 0.05f) &&
                          (candidate->tail_accel_max_g <= 0.08f) &&
                          (candidate->wheel_highpass_sum_max >= 600.0f)) ? 1U : 0U;
    unstable_duplicate = ((candidate->peak.impact_score < 1.30f) &&
                          (candidate->tail_accel_max_g >= 0.24f) &&
                          (candidate->post_min_speed_mps >= 0.60f) &&
                          (beacon_detection_locations_match(candidate->peak_location,
                                                            candidate->pre_location) == 0U) &&
                          (beacon_detection_locations_match(candidate->peak_location,
                                                            candidate->post_location) == 0U) &&
                          (beacon_detection_locations_match(candidate->pre_location,
                                                            candidate->post_location) == 0U)) ? 1U : 0U;

    hard_stop_noise = ((candidate->pre_speed_mps > 1.80f) &&
                       (candidate->post_speed_average_mps < 0.20f) &&
                       (candidate->peak.impact_score < 3.0f)) ? 1U : 0U;
    spin_noise = ((candidate->peak.gyro_z_abs_dps > 40.0f) &&
                  (candidate->peak.impact_score < 1.30f)) ? 1U : 0U;

    if(all_wheel_stop_hit != 0U)
    {
        if(candidate->pre_location != BEACON_BUMP_LOCATION_UNKNOWN)
        {
            *location = candidate->pre_location;
        }
    }
    else if(front_fast_hit != 0U)
    {
        *location = BEACON_BUMP_LOCATION_FRONT;
    }

    return (((adaptive_hit != 0U) ||
             (low_speed_hit != 0U) ||
             (strong_hit != 0U) ||
             (weak_low_speed_hit != 0U) ||
             (front_fast_hit != 0U) ||
             (all_wheel_stop_hit != 0U)) &&
            ((hard_stop_noise == 0U) || (all_wheel_stop_hit != 0U)) &&
            (unstable_duplicate == 0U) &&
            (spin_noise == 0U)) ? 1U : 0U;
}

static uint8_t beacon_detection_is_exit_hit(const beacon_detection_imu_sample_t *window)
{
    return ((window->impact_score >= 0.82f) &&
            (g_beacon_detection.speed_mps > 0.05f)) ? 1U : 0U;
}

static void beacon_detection_latch_event(const beacon_detection_imu_sample_t *window,
                                         uint8_t is_enter_event,
                                         uint8_t on_beacon_state,
                                         beacon_bump_location_t location)
{
    g_beacon_detection.bump_detected = 1U;
    g_beacon_detection.enter_event = is_enter_event;
    g_beacon_detection.exit_event = (is_enter_event == 0U) ? 1U : 0U;
    g_beacon_detection.on_beacon = on_beacon_state;
    g_beacon_detection.confidence = (window->impact_score >= 1.25f) ?
                                    BEACON_BUMP_CONFIDENCE_HIGH :
                                    BEACON_BUMP_CONFIDENCE_LOW;
    g_beacon_detection.location = location;
    g_beacon_detection.wheel_mask = beacon_detection_wheel_mask_from_location(location);
    g_beacon_detection.score = window->impact_score;
    g_beacon_detection.gyro_xy_dps = window->gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = window->gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = window->tilt_rate_dps;
    g_beacon_detection.tilt_deg = window->tilt_deg;
    g_beacon_detection.accel_norm_error_g = window->accel_norm_error_g;
    g_beacon_detection.hold_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;
    s_event_ticks = BEACON_DETECTION_EVENT_HOLD_TICKS;

    if(is_enter_event != 0U)
    {
        g_beacon_detection.enter_count++;
    }
    else
    {
        g_beacon_detection.exit_count++;
    }
    g_beacon_detection.event_count++;
}

static void beacon_detection_push_event(const beacon_detection_imu_sample_t *window,
                                        uint8_t is_enter_event,
                                        uint8_t on_beacon_state,
                                        beacon_bump_location_t location)
{
    beacon_detection_event_item_t *item;

    if(s_event_queue_count >= BEACON_DETECTION_PENDING_ENTER_MAX)
    {
        item = &s_event_queue[s_event_queue_tail];
    }
    else
    {
        item = &s_event_queue[s_event_queue_tail];
        s_event_queue_tail++;
        if(s_event_queue_tail >= BEACON_DETECTION_PENDING_ENTER_MAX)
        {
            s_event_queue_tail = 0U;
        }
        s_event_queue_count++;
    }

    item->is_enter_event = is_enter_event;
    item->on_beacon_state = on_beacon_state;
    item->sample = *window;
    item->location = location;
}

static void beacon_detection_dispatch_queued_event(void)
{
    beacon_detection_event_item_t *item;

    if((s_event_ticks != 0U) || (s_event_queue_count == 0U))
    {
        return;
    }

    item = &s_event_queue[s_event_queue_head];
    beacon_detection_latch_event(&item->sample,
                                 item->is_enter_event,
                                 item->on_beacon_state,
                                 item->location);
    memset(item, 0, sizeof(*item));
    s_event_queue_head++;
    if(s_event_queue_head >= BEACON_DETECTION_PENDING_ENTER_MAX)
    {
        s_event_queue_head = 0U;
    }
    s_event_queue_count--;
}

static float beacon_detection_candidate_quality(const beacon_detection_enter_candidate_t *candidate)
{
    float quality;

    quality = candidate->peak.impact_score + (0.05f * candidate->peak_z);
    if(candidate->wheel_highpass_sum_max >= 600.0f)
    {
        quality += 0.50f;
    }
    if((candidate->peak.impact_score >= 1.45f) &&
       (candidate->wheel_highpass_sum_max < 180.0f))
    {
        quality += 0.25f;
    }

    return quality;
}

static uint8_t beacon_detection_candidate_is_weak_only(const beacon_detection_enter_candidate_t *candidate)
{
    return ((candidate->peak.impact_score >= 0.70f) &&
            (candidate->peak.impact_score < 1.10f) &&
            (candidate->peak_z >= 2.0f) &&
            (candidate->peak.gyro_z_abs_dps <= 5.0f)) ? 1U : 0U;
}

static void beacon_detection_pending_add(const beacon_detection_enter_candidate_t *candidate)
{
    uint8_t i;
    uint8_t best_index;
    float best_quality;
    float current_quality;

    if(candidate->valid == 0U)
    {
        return;
    }

    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        if(s_pending_enter[i].valid == 0U)
        {
            s_pending_enter[i] = *candidate;
            return;
        }
    }

    best_index = 0U;
    best_quality = beacon_detection_candidate_quality(&s_pending_enter[0]);
    for(i = 1U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        current_quality = beacon_detection_candidate_quality(&s_pending_enter[i]);
        if(current_quality < best_quality)
        {
            best_quality = current_quality;
            best_index = i;
        }
    }

    if(beacon_detection_candidate_quality(candidate) > best_quality)
    {
        s_pending_enter[best_index] = *candidate;
    }
}

static void beacon_detection_reset_exit_track(void)
{
    memset(&s_exit_peak, 0, sizeof(s_exit_peak));
    memset(&s_exit_min_speed_sample, 0, sizeof(s_exit_min_speed_sample));
    s_exit_peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_exit_min_speed_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_exit_min_speed_mps = 999.0f;
    s_exit_age_ticks = 0U;
    s_exit_track_active = 0U;
    s_exit_peak_valid = 0U;
    s_exit_min_speed_valid = 0U;
}

static void beacon_detection_start_exit_track(void)
{
    beacon_detection_reset_exit_track();
    s_exit_track_active = 1U;
}

static void beacon_detection_update_exit_track(const beacon_detection_imu_sample_t *window)
{
    beacon_bump_location_t location_now;

    if(s_exit_track_active == 0U)
    {
        return;
    }

    s_exit_age_ticks++;
    if(s_exit_age_ticks < BEACON_DETECTION_ON_MIN_TICKS)
    {
        return;
    }

    location_now = beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                         g_beacon_detection.vel[1]);
    if((s_exit_min_speed_valid == 0U) ||
       (g_beacon_detection.speed_mps < s_exit_min_speed_mps))
    {
        s_exit_min_speed_valid = 1U;
        s_exit_min_speed_mps = g_beacon_detection.speed_mps;
        s_exit_min_speed_sample = *window;
        s_exit_min_speed_location = location_now;
    }

    if((beacon_detection_is_exit_hit(window) != 0U) &&
       ((s_exit_peak_valid == 0U) || (window->impact_score > s_exit_peak.impact_score)))
    {
        s_exit_peak_valid = 1U;
        s_exit_peak = *window;
        s_exit_peak_location = location_now;
    }
}

static void beacon_detection_capture_deferred_exit(void)
{
    if(s_exit_track_active == 0U)
    {
        return;
    }

    if(s_exit_peak_valid != 0U)
    {
        s_deferred_exit_sample = s_exit_peak;
        s_deferred_exit_location = s_exit_peak_location;
        s_deferred_exit_valid = 1U;
    }
    else if(s_exit_min_speed_valid != 0U)
    {
        s_deferred_exit_sample = s_exit_min_speed_sample;
        s_deferred_exit_location = s_exit_min_speed_location;
        s_deferred_exit_valid = 1U;
    }
    else
    {
        memset(&s_deferred_exit_sample, 0, sizeof(s_deferred_exit_sample));
        s_deferred_exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
        s_deferred_exit_valid = 1U;
    }
}

static void beacon_detection_finish_exit_if_due(void)
{
    if((s_exit_track_active != 0U) &&
       (s_exit_age_ticks >= BEACON_DETECTION_EXIT_SEARCH_TICKS))
    {
        beacon_detection_capture_deferred_exit();
        beacon_detection_push_event(&s_deferred_exit_sample, 0U, 0U, s_deferred_exit_location);
        s_deferred_exit_valid = 0U;
        beacon_detection_reset_exit_track();
        s_on_beacon_state = 0U;
        g_beacon_detection.on_beacon = s_on_beacon_state;
    }
}

static void beacon_detection_realtime_reset_candidate(void)
{
    memset(&s_realtime_candidate, 0, sizeof(s_realtime_candidate));
    s_realtime_candidate.enter_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_realtime_candidate.early_enter_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_realtime_candidate.segment_peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_realtime_candidate.exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_realtime_candidate.early_exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
}

static void beacon_detection_realtime_reset_exit_peak(void)
{
    s_realtime_candidate.exit_valid = 0U;
    s_realtime_candidate.exit_age_ticks = 0U;
    memset(&s_realtime_candidate.exit_sample, 0, sizeof(s_realtime_candidate.exit_sample));
    s_realtime_candidate.exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_realtime_candidate.win_gyro_xy_dps = 0.0f;
    s_realtime_candidate.exit_score = 0.0f;
    s_realtime_candidate.exit_accel_norm_error_g = 0.0f;
}

static void beacon_detection_realtime_init_pose_span(float roll_deg,
                                                     float pitch_deg)
{
    s_realtime_candidate.pose_roll_min_deg = roll_deg;
    s_realtime_candidate.pose_roll_max_deg = roll_deg;
    s_realtime_candidate.pose_pitch_min_deg = pitch_deg;
    s_realtime_candidate.pose_pitch_max_deg = pitch_deg;
}

static void beacon_detection_realtime_update_pose_span(float roll_deg,
                                                       float pitch_deg)
{
    if(s_realtime_candidate.active == 0U)
    {
        return;
    }

    s_realtime_candidate.pose_roll_min_deg =
        car_math_minf(s_realtime_candidate.pose_roll_min_deg, roll_deg);
    s_realtime_candidate.pose_roll_max_deg =
        car_math_maxf(s_realtime_candidate.pose_roll_max_deg, roll_deg);
    s_realtime_candidate.pose_pitch_min_deg =
        car_math_minf(s_realtime_candidate.pose_pitch_min_deg, pitch_deg);
    s_realtime_candidate.pose_pitch_max_deg =
        car_math_maxf(s_realtime_candidate.pose_pitch_max_deg, pitch_deg);
}

static void beacon_detection_realtime_update_window_maxima(void)
{
    if(s_realtime_candidate.active == 0U)
    {
        return;
    }

    if(g_beacon_detection.wheel_highpass_count >
       s_realtime_candidate.window_max_wheel_highpass_count)
    {
        s_realtime_candidate.window_max_wheel_highpass_count =
            g_beacon_detection.wheel_highpass_count;
    }
}

static float beacon_detection_realtime_pose_axis_span(void)
{
    float roll_span;
    float pitch_span;

    roll_span = s_realtime_candidate.pose_roll_max_deg -
                s_realtime_candidate.pose_roll_min_deg;
    pitch_span = s_realtime_candidate.pose_pitch_max_deg -
                 s_realtime_candidate.pose_pitch_min_deg;

    return car_math_maxf(roll_span, pitch_span);
}

static uint8_t beacon_detection_realtime_candidate_shape_valid(void)
{
    uint8_t strong_hit;
    uint8_t mid_strong_hit;
    uint8_t medium_hit;
    uint8_t weak_gyro_hit;
    uint8_t weak_short_hit;

    if(s_realtime_candidate.exit_valid == 0U)
    {
        return 0U;
    }

    strong_hit = ((s_realtime_candidate.max_score >= 3.00f) &&
                  (s_realtime_candidate.max_gyro_xy_dps >= 42.0f) &&
                  (s_realtime_candidate.exit_score >= 0.90f) &&
                  ((s_realtime_candidate.exit_age_ticks > 220U) ||
                   (s_realtime_candidate.exit_accel_norm_error_g <= 0.19f) ||
                   (s_realtime_candidate.max_score >= 4.00f))) ? 1U : 0U;

    mid_strong_hit = ((s_realtime_candidate.enter_runtime_ticks >=
                       BEACON_DETECTION_MID_STARTUP_TICKS) &&
                      (s_realtime_candidate.max_score >= 2.44f) &&
                      (s_realtime_candidate.max_score < 3.00f) &&
                      (s_realtime_candidate.max_gyro_xy_dps >= 55.0f) &&
                      (s_realtime_candidate.max_gyro_z_abs_dps >= 5.0f) &&
                      (s_realtime_candidate.exit_score >= 1.20f)) ? 1U : 0U;

    medium_hit = ((s_realtime_candidate.max_score >= 2.04f) &&
                  (s_realtime_candidate.max_score < 2.45f) &&
                  (s_realtime_candidate.exit_accel_norm_error_g <= 0.114f) &&
                  (s_realtime_candidate.first_speed_mps >= 0.53f)) ? 1U : 0U;

    weak_gyro_hit = ((s_realtime_candidate.max_score < 2.04f) &&
                     (s_realtime_candidate.win_gyro_xy_dps >= 50.9f) &&
                     (s_realtime_candidate.first_speed_mps <= 0.48f) &&
                     (s_realtime_candidate.max_wheel_highpass_count < 100.0f)) ? 1U : 0U;

    weak_short_hit = ((s_realtime_candidate.max_score < 2.04f) &&
                      (s_realtime_candidate.exit_score >= 1.806f) &&
                      (s_realtime_candidate.duration_ticks >= 450U) &&
                      (s_realtime_candidate.duration_ticks <= 615U) &&
                      (s_realtime_candidate.peak_count >= 2U) &&
                      (s_realtime_candidate.max_gyro_z_abs_dps <= 5.0f) &&
                      (s_realtime_candidate.exit_accel_norm_error_g >= 0.095f) &&
                      (s_realtime_candidate.exit_accel_norm_error_g <= 0.20f)) ? 1U : 0U;

    return ((strong_hit != 0U) ||
            (mid_strong_hit != 0U) ||
            (medium_hit != 0U) ||
            (weak_gyro_hit != 0U) ||
            (weak_short_hit != 0U)) ? 1U : 0U;
}

static uint8_t beacon_detection_realtime_early_shape_valid(void)
{
    return beacon_detection_realtime_candidate_shape_valid();
}

static float beacon_detection_realtime_score(const beacon_detection_imu_sample_t *window)
{
    float motion_score;
    float shock_score;
    float pitch_contact_score;
    float roll_contact_score;
    float score;

    motion_score = car_math_maxf(beacon_detection_ratio(g_beacon_detection.speed_mps, 0.28f),
                                 beacon_detection_ratio(g_beacon_detection.wheel_highpass_count, 24.0f));
    shock_score = car_math_minf(beacon_detection_ratio(window->gyro_xy_dps, 28.0f),
                                beacon_detection_ratio(window->accel_norm_error_g, 0.055f));
    shock_score = car_math_minf(shock_score, motion_score);
    score = shock_score;

    motion_score = car_math_maxf(beacon_detection_ratio(g_beacon_detection.speed_mps, 0.24f),
                                 beacon_detection_ratio(g_beacon_detection.wheel_highpass_count, 22.0f));
    pitch_contact_score = car_math_minf(beacon_detection_ratio(window->pitch_rate_dps, 24.0f),
                                beacon_detection_ratio(window->accel_norm_error_g, 0.045f));
    pitch_contact_score = car_math_minf(pitch_contact_score, motion_score);
    roll_contact_score = car_math_minf(beacon_detection_ratio(window->roll_rate_dps, 24.0f),
                                       beacon_detection_ratio(window->accel_norm_error_g, 0.045f));
    roll_contact_score = car_math_minf(roll_contact_score, motion_score);
    score = car_math_maxf(score, pitch_contact_score);
    score = car_math_maxf(score, roll_contact_score);

    return score;
}

static float beacon_detection_realtime_gated_score(float score,
                                                   const beacon_detection_imu_sample_t *window)
{
    if(window->accel_norm_error_g < 0.045f)
    {
        return 0.0f;
    }

    if((g_beacon_detection.speed_mps < 0.18f) &&
       (g_beacon_detection.wheel_highpass_count < 16.0f))
    {
        return 0.0f;
    }

    return score;
}

static uint8_t beacon_detection_realtime_seed_is_high(float gated_score)
{
    return (gated_score >= 0.95f) ? 1U : 0U;
}

static void beacon_detection_realtime_accept_candidate(void);
static void beacon_detection_realtime_remember_early_candidate(void);
static void beacon_detection_realtime_remember_peak_closed_candidate(void);
static void beacon_detection_realtime_remember_weak_candidate(void);
static void beacon_detection_realtime_remember_fast_exit_gate_candidate(void);
static void beacon_detection_realtime_remember_weak_clean_tail_candidate(void);
static void beacon_detection_realtime_remember_strong_tail_pose_candidate(void);
static void beacon_detection_realtime_remember_side_tail_pose_candidate(void);
static void beacon_detection_realtime_remember_slow_tail_pose_candidate(void);

static uint8_t beacon_detection_realtime_segment_starts_new_cluster(void)
{
    if(s_realtime_candidate.active == 0U)
    {
        return 0U;
    }

    if(s_realtime_candidate.last_peak_age_ticks <=
       s_realtime_candidate.segment_peak_age_ticks)
    {
        return 0U;
    }

    return (((uint16_t)(s_realtime_candidate.last_peak_age_ticks -
                       s_realtime_candidate.segment_peak_age_ticks)) >
            BEACON_DETECTION_CLUSTER_GAP_TICKS) ? 1U : 0U;
}

static void beacon_detection_realtime_start_segment(const beacon_detection_imu_sample_t *window,
                                                    float gated_score,
                                                    float roll_deg,
                                                    float pitch_deg)
{
    s_realtime_candidate.segment_active = 1U;
    s_realtime_candidate.segment_pending = 0U;
    s_realtime_candidate.last_high_gap_ticks = 0U;
    s_realtime_candidate.segment_peak_age_ticks = 0U;
    s_realtime_candidate.segment_peak_sample = *window;
    s_realtime_candidate.segment_peak_sample.impact_score = gated_score;
    s_realtime_candidate.segment_peak_score = gated_score;
    s_realtime_candidate.segment_peak_location =
        beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                              g_beacon_detection.vel[1]);
    s_realtime_candidate.segment_peak_roll_deg = roll_deg;
    s_realtime_candidate.segment_peak_pitch_deg = pitch_deg;
    s_realtime_candidate.segment_peak_speed_mps = g_beacon_detection.speed_mps;
    s_realtime_candidate.segment_peak_wheel_highpass_count =
        g_beacon_detection.wheel_highpass_count;
}

static void beacon_detection_realtime_update_segment(const beacon_detection_imu_sample_t *window,
                                                     float gated_score,
                                                     float roll_deg,
                                                     float pitch_deg)
{
    s_realtime_candidate.last_high_gap_ticks = 0U;
    if(gated_score > s_realtime_candidate.segment_peak_score)
    {
        s_realtime_candidate.segment_peak_age_ticks = 0U;
        s_realtime_candidate.segment_peak_sample = *window;
        s_realtime_candidate.segment_peak_sample.impact_score = gated_score;
        s_realtime_candidate.segment_peak_score = gated_score;
        s_realtime_candidate.segment_peak_location =
            beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                  g_beacon_detection.vel[1]);
        s_realtime_candidate.segment_peak_roll_deg = roll_deg;
        s_realtime_candidate.segment_peak_pitch_deg = pitch_deg;
        s_realtime_candidate.segment_peak_speed_mps = g_beacon_detection.speed_mps;
        s_realtime_candidate.segment_peak_wheel_highpass_count =
            g_beacon_detection.wheel_highpass_count;
    }
}

static void beacon_detection_realtime_start_cluster(void)
{
    s_realtime_candidate.active = 1U;
    s_realtime_candidate.exit_valid = 0U;
    s_realtime_candidate.peak_count = 1U;
    s_realtime_candidate.enter_age_ticks = s_realtime_candidate.segment_peak_age_ticks;
    s_realtime_candidate.last_peak_age_ticks = s_realtime_candidate.segment_peak_age_ticks;
    s_realtime_candidate.duration_ticks = 0U;
    s_realtime_candidate.enter_runtime_ticks =
        s_runtime_ticks - s_realtime_candidate.enter_age_ticks;
    s_realtime_candidate.enter_sample = s_realtime_candidate.segment_peak_sample;
    s_realtime_candidate.enter_location = s_realtime_candidate.segment_peak_location;
    beacon_detection_realtime_init_pose_span(s_realtime_candidate.segment_peak_roll_deg,
                                             s_realtime_candidate.segment_peak_pitch_deg);
    s_realtime_candidate.first_speed_mps = s_realtime_candidate.segment_peak_speed_mps;
    s_realtime_candidate.max_score = s_realtime_candidate.segment_peak_score;
    s_realtime_candidate.max_gyro_xy_dps = s_realtime_candidate.segment_peak_sample.gyro_xy_dps;
    s_realtime_candidate.max_gyro_z_abs_dps = s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps;
    s_realtime_candidate.early_max_gyro_z_abs_dps =
        s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps;
    s_realtime_candidate.max_wheel_highpass_count =
        s_realtime_candidate.segment_peak_wheel_highpass_count;
    s_realtime_candidate.window_max_wheel_highpass_count =
        s_realtime_candidate.segment_peak_wheel_highpass_count;
    beacon_detection_realtime_reset_exit_peak();
}

static void beacon_detection_realtime_update_first_cluster_peak(void)
{
    if(s_realtime_candidate.active == 0U)
    {
        return;
    }

    s_realtime_candidate.enter_sample = s_realtime_candidate.segment_peak_sample;
    s_realtime_candidate.enter_location = s_realtime_candidate.segment_peak_location;
    beacon_detection_realtime_init_pose_span(s_realtime_candidate.segment_peak_roll_deg,
                                             s_realtime_candidate.segment_peak_pitch_deg);
    s_realtime_candidate.first_speed_mps = s_realtime_candidate.segment_peak_speed_mps;
    s_realtime_candidate.max_score = s_realtime_candidate.segment_peak_score;
    s_realtime_candidate.max_gyro_xy_dps = s_realtime_candidate.segment_peak_sample.gyro_xy_dps;
    s_realtime_candidate.max_gyro_z_abs_dps = s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps;
    s_realtime_candidate.early_max_gyro_z_abs_dps =
        s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps;
    s_realtime_candidate.max_wheel_highpass_count =
        s_realtime_candidate.segment_peak_wheel_highpass_count;
    s_realtime_candidate.window_max_wheel_highpass_count =
        s_realtime_candidate.segment_peak_wheel_highpass_count;
    s_realtime_candidate.enter_age_ticks = s_realtime_candidate.segment_peak_age_ticks;
    s_realtime_candidate.last_peak_age_ticks = s_realtime_candidate.segment_peak_age_ticks;
    s_realtime_candidate.duration_ticks = 0U;
    s_realtime_candidate.enter_runtime_ticks = s_runtime_ticks -
        s_realtime_candidate.enter_age_ticks;
    beacon_detection_realtime_reset_exit_peak();
}

static void beacon_detection_realtime_note_cluster_peak(void)
{
    if(s_realtime_candidate.active == 0U)
    {
        beacon_detection_realtime_start_cluster();
        return;
    }

    if(s_realtime_candidate.peak_count < 255U)
    {
        s_realtime_candidate.peak_count++;
    }
    s_realtime_candidate.last_peak_age_ticks = s_realtime_candidate.segment_peak_age_ticks;
    s_realtime_candidate.duration_ticks =
        s_realtime_candidate.enter_age_ticks - s_realtime_candidate.last_peak_age_ticks;
    if(s_realtime_candidate.segment_peak_score > s_realtime_candidate.max_score)
    {
        s_realtime_candidate.max_score = s_realtime_candidate.segment_peak_score;
    }
    if(s_realtime_candidate.segment_peak_sample.gyro_xy_dps > s_realtime_candidate.max_gyro_xy_dps)
    {
        s_realtime_candidate.max_gyro_xy_dps = s_realtime_candidate.segment_peak_sample.gyro_xy_dps;
    }
    if(s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps > s_realtime_candidate.max_gyro_z_abs_dps)
    {
        s_realtime_candidate.max_gyro_z_abs_dps = s_realtime_candidate.segment_peak_sample.gyro_z_abs_dps;
    }
    if(s_realtime_candidate.segment_peak_wheel_highpass_count >
       s_realtime_candidate.max_wheel_highpass_count)
    {
        s_realtime_candidate.max_wheel_highpass_count =
            s_realtime_candidate.segment_peak_wheel_highpass_count;
    }

    if(s_realtime_candidate.enter_age_ticks >= BEACON_DETECTION_PAIR_MAX_TICKS)
    {
        beacon_detection_realtime_remember_early_candidate();
    }
    else
    {
        beacon_detection_realtime_remember_peak_closed_candidate();
    }
}

static void beacon_detection_realtime_emit_segment_peak(void)
{
    beacon_detection_imu_sample_t segment_peak_sample;
    beacon_bump_location_t segment_peak_location;
    float segment_peak_score;
    float segment_peak_speed_mps;
    float segment_peak_wheel_highpass_count;

    if(s_realtime_candidate.segment_active == 0U)
    {
        return;
    }

    segment_peak_sample = s_realtime_candidate.segment_peak_sample;
    segment_peak_location = s_realtime_candidate.segment_peak_location;
    segment_peak_score = s_realtime_candidate.segment_peak_score;
    segment_peak_speed_mps = s_realtime_candidate.segment_peak_speed_mps;
    segment_peak_wheel_highpass_count =
        s_realtime_candidate.segment_peak_wheel_highpass_count;

    if((s_realtime_candidate.segment_pending != 0U) &&
       (s_realtime_candidate.early_accepted == 0U) &&
       (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_PAIR_MAX_TICKS) &&
       (s_realtime_candidate.segment_peak_score >= BEACON_DETECTION_EXPIRED_RESEED_SCORE))
    {
        if(beacon_detection_realtime_candidate_shape_valid() != 0U)
        {
            beacon_detection_realtime_accept_candidate();
        }
        else
        {
            beacon_detection_realtime_reset_candidate();
        }

        s_realtime_candidate.segment_active = 1U;
        s_realtime_candidate.segment_peak_sample = segment_peak_sample;
        s_realtime_candidate.segment_peak_location = segment_peak_location;
        s_realtime_candidate.segment_peak_score = segment_peak_score;
        s_realtime_candidate.segment_peak_speed_mps = segment_peak_speed_mps;
        s_realtime_candidate.segment_peak_wheel_highpass_count =
            segment_peak_wheel_highpass_count;
        beacon_detection_realtime_start_cluster();
    }
    else if((s_realtime_candidate.segment_pending != 0U) &&
       (beacon_detection_realtime_segment_starts_new_cluster() != 0U))
    {
        if(beacon_detection_realtime_candidate_shape_valid() != 0U)
        {
            beacon_detection_realtime_accept_candidate();
        }
        else
        {
            beacon_detection_realtime_reset_candidate();
        }

        s_realtime_candidate.segment_active = 1U;
        s_realtime_candidate.segment_peak_sample = segment_peak_sample;
        s_realtime_candidate.segment_peak_location = segment_peak_location;
        s_realtime_candidate.segment_peak_score = segment_peak_score;
        s_realtime_candidate.segment_peak_speed_mps = segment_peak_speed_mps;
        s_realtime_candidate.segment_peak_wheel_highpass_count =
            segment_peak_wheel_highpass_count;
        beacon_detection_realtime_start_cluster();
    }
    else if(s_realtime_candidate.segment_pending != 0U)
    {
        beacon_detection_realtime_note_cluster_peak();
    }

    s_realtime_candidate.segment_active = 0U;
    s_realtime_candidate.segment_pending = 0U;
    s_realtime_candidate.segment_peak_location = BEACON_BUMP_LOCATION_UNKNOWN;
}

static void beacon_detection_realtime_update_exit_peak(const beacon_detection_imu_sample_t *window,
                                                       float gated_score)
{
    if(window->gyro_xy_dps > s_realtime_candidate.win_gyro_xy_dps)
    {
        s_realtime_candidate.win_gyro_xy_dps = window->gyro_xy_dps;
    }
    if(window->gyro_z_abs_dps > s_realtime_candidate.early_max_gyro_z_abs_dps)
    {
        s_realtime_candidate.early_max_gyro_z_abs_dps = window->gyro_z_abs_dps;
    }

    if((s_realtime_candidate.exit_valid == 0U) ||
       (gated_score > s_realtime_candidate.exit_score))
    {
        s_realtime_candidate.exit_valid = 1U;
        s_realtime_candidate.exit_sample = *window;
        s_realtime_candidate.exit_sample.impact_score = gated_score;
        s_realtime_candidate.exit_location =
            beacon_detection_location_from_motion(g_beacon_detection.vel[0],
                                                  g_beacon_detection.vel[1]);
        s_realtime_candidate.exit_age_ticks = s_realtime_candidate.enter_age_ticks;
        s_realtime_candidate.exit_score = gated_score;
        s_realtime_candidate.exit_accel_norm_error_g = window->accel_norm_error_g;
    }
}

static uint8_t beacon_detection_realtime_strong_shape_valid(void)
{
    if(s_realtime_candidate.exit_valid == 0U)
    {
        return 0U;
    }

    return ((s_realtime_candidate.max_score >= 3.00f) &&
            (s_realtime_candidate.max_gyro_xy_dps >= 42.0f) &&
            (s_realtime_candidate.exit_score >= 0.90f) &&
            ((s_realtime_candidate.exit_age_ticks > 220U) ||
             (s_realtime_candidate.exit_accel_norm_error_g <= 0.19f) ||
             (s_realtime_candidate.max_score >= 4.00f))) ? 1U : 0U;
}

static void beacon_detection_realtime_maybe_finish_cluster(void)
{
    if((s_realtime_candidate.active != 0U) &&
       (s_realtime_candidate.segment_active == 0U) &&
       (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_PAIR_MAX_TICKS) &&
       (s_realtime_candidate.last_peak_age_ticks > BEACON_DETECTION_CLUSTER_GAP_TICKS))
    {
        if(s_realtime_candidate.early_accepted != 0U)
        {
            beacon_detection_realtime_reset_candidate();
        }
        else if(beacon_detection_realtime_candidate_shape_valid() != 0U)
        {
            beacon_detection_realtime_accept_candidate();
        }
        else
        {
            beacon_detection_realtime_reset_candidate();
        }
    }
}

static void beacon_detection_realtime_remember_early_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t yaw_shock_hit;

    yaw_shock_hit = ((s_realtime_candidate.exit_valid != 0U) &&
                     (s_realtime_candidate.early_max_gyro_z_abs_dps >= 50.0f) &&
                     (s_realtime_candidate.win_gyro_xy_dps >= 45.5f)) ? 1U : 0U;

    if((s_realtime_candidate.early_accepted == 0U) &&
       ((beacon_detection_realtime_early_shape_valid() != 0U) ||
        (yaw_shock_hit != 0U)))
    {
        s_realtime_candidate.early_accepted = 1U;
        s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
        s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
        s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
        s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

        location = s_realtime_candidate.early_enter_location;
        if(location == BEACON_BUMP_LOCATION_UNKNOWN)
        {
            location = s_realtime_candidate.early_exit_location;
        }

        beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                    1U,
                                    1U,
                                    location);
        beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                    0U,
                                    0U,
                                    location);
        s_on_beacon_state = 0U;
        g_beacon_detection.on_beacon = s_on_beacon_state;
    }
}

static void beacon_detection_realtime_remember_peak_closed_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t strong_hit;
    uint8_t mid_strong_hit;
    uint8_t medium_hit;
    uint8_t yaw_shock_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U))
    {
        return;
    }

    strong_hit = ((s_realtime_candidate.max_score >= 3.00f) &&
                  (s_realtime_candidate.max_gyro_xy_dps >= 42.0f) &&
                  (s_realtime_candidate.exit_score >= 0.90f) &&
                  ((s_realtime_candidate.exit_age_ticks > 220U) ||
                   (s_realtime_candidate.exit_accel_norm_error_g <= 0.19f) ||
                   (s_realtime_candidate.max_score >= 4.00f))) ? 1U : 0U;

    mid_strong_hit = ((s_realtime_candidate.enter_runtime_ticks >=
                       BEACON_DETECTION_MID_STARTUP_TICKS) &&
                      (s_realtime_candidate.max_score >= 2.44f) &&
                      (s_realtime_candidate.max_score < 3.00f) &&
                      (s_realtime_candidate.max_gyro_xy_dps >= 55.0f) &&
                      (s_realtime_candidate.max_gyro_z_abs_dps >= 5.0f) &&
                      (s_realtime_candidate.exit_score >= 1.20f)) ? 1U : 0U;

    medium_hit = ((s_realtime_candidate.max_score >= 2.04f) &&
                  (s_realtime_candidate.max_score < 2.45f) &&
                  (s_realtime_candidate.exit_accel_norm_error_g <= 0.114f) &&
                  (s_realtime_candidate.first_speed_mps >= 0.53f)) ? 1U : 0U;

    yaw_shock_hit = ((s_realtime_candidate.early_max_gyro_z_abs_dps >= 50.0f) &&
                     (s_realtime_candidate.win_gyro_xy_dps >= 45.5f)) ? 1U : 0U;

    if((strong_hit == 0U) &&
       (mid_strong_hit == 0U) &&
       (medium_hit == 0U) &&
       (yaw_shock_hit == 0U))
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
}

static void beacon_detection_realtime_remember_weak_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t weak_gyro_hit;
    uint8_t weak_short_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U) ||
       (s_realtime_candidate.enter_age_ticks < BEACON_DETECTION_WEAK_EARLY_TICKS))
    {
        return;
    }

    weak_gyro_hit = ((s_realtime_candidate.max_score < 2.04f) &&
                     (s_realtime_candidate.win_gyro_xy_dps >= 50.9f) &&
                     (s_realtime_candidate.first_speed_mps <= 0.48f) &&
                     (s_realtime_candidate.max_wheel_highpass_count < 100.0f)) ? 1U : 0U;

    weak_short_hit = ((s_realtime_candidate.max_score < 2.04f) &&
                      (s_realtime_candidate.exit_score >= 1.806f) &&
                      (s_realtime_candidate.duration_ticks >= 450U) &&
                      (s_realtime_candidate.duration_ticks <= 615U) &&
                      (s_realtime_candidate.peak_count >= 2U) &&
                      (s_realtime_candidate.max_gyro_z_abs_dps <= 5.0f) &&
                      (s_realtime_candidate.exit_accel_norm_error_g >= 0.095f) &&
                      (s_realtime_candidate.exit_accel_norm_error_g <= 0.20f)) ? 1U : 0U;

    if((weak_gyro_hit == 0U) && (weak_short_hit == 0U))
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
}

static void beacon_detection_realtime_remember_fast_exit_gate_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t fast_exit_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U) ||
       (s_realtime_candidate.enter_age_ticks < BEACON_DETECTION_FAST_EXIT_GATE_MIN_TICKS) ||
       (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_FAST_EXIT_GATE_MAX_TICKS))
    {
        return;
    }

    fast_exit_hit = ((s_realtime_candidate.exit_age_ticks >= 205U) &&
                     (s_realtime_candidate.exit_age_ticks <= 320U) &&
                     (s_realtime_candidate.max_score >= 1.05f) &&
                     (s_realtime_candidate.exit_score >= 1.20f) &&
                     (s_realtime_candidate.win_gyro_xy_dps >= 45.0f) &&
                     (s_realtime_candidate.exit_accel_norm_error_g <= 0.12f) &&
                     (s_realtime_candidate.max_wheel_highpass_count <= 60.0f) &&
                     (s_realtime_candidate.first_speed_mps <= 0.80f)) ? 1U : 0U;

    if(fast_exit_hit == 0U)
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
}

static void beacon_detection_realtime_remember_weak_clean_tail_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t weak_clean_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U) ||
       (s_realtime_candidate.enter_age_ticks < BEACON_DETECTION_WEAK_CLEAN_TAIL_TICKS))
    {
        return;
    }

    weak_clean_hit = ((s_realtime_candidate.exit_age_ticks >= 620U) &&
                      (s_realtime_candidate.exit_age_ticks <= 720U) &&
                      (s_realtime_candidate.exit_score >= 1.55f) &&
                      (s_realtime_candidate.exit_score <= 1.90f) &&
                      (s_realtime_candidate.win_gyro_xy_dps >= 25.0f) &&
                      (s_realtime_candidate.win_gyro_xy_dps <= 35.0f) &&
                      (s_realtime_candidate.exit_accel_norm_error_g <= 0.09f) &&
                      (s_realtime_candidate.max_wheel_highpass_count <= 30.0f) &&
                      (s_realtime_candidate.first_speed_mps >= 0.70f) &&
                      (s_realtime_candidate.first_speed_mps <= 0.90f) &&
                      (s_realtime_candidate.peak_count >= 2U)) ? 1U : 0U;

    if(weak_clean_hit == 0U)
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
}

static void beacon_detection_realtime_remember_strong_tail_pose_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t strong_tail_pose_hit;
    uint8_t very_strong_exit_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U))
    {
        return;
    }

    strong_tail_pose_hit =
        ((s_realtime_candidate.enter_age_ticks >= BEACON_DETECTION_STRONG_TAIL_POSE_TICKS) &&
         (s_realtime_candidate.enter_age_ticks <= 850U) &&
         (s_realtime_candidate.enter_runtime_ticks >= BEACON_DETECTION_MID_STARTUP_TICKS) &&
         (s_realtime_candidate.exit_age_ticks >= 580U) &&
         (s_realtime_candidate.exit_age_ticks <= 780U) &&
         (s_realtime_candidate.exit_score >= 2.0f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 60.0f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.22f) &&
         (s_realtime_candidate.max_wheel_highpass_count >= 30.0f) &&
         (beacon_detection_realtime_pose_axis_span() >= 3.0f)) ? 1U : 0U;

    very_strong_exit_hit =
        ((s_realtime_candidate.enter_age_ticks >= 600U) &&
         (s_realtime_candidate.enter_age_ticks <= 850U) &&
         (s_realtime_candidate.enter_runtime_ticks >= BEACON_DETECTION_STRONG_TAIL_STARTUP_TICKS) &&
         (s_realtime_candidate.exit_score >= 3.20f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 90.0f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.24f)) ? 1U : 0U;

    if((strong_tail_pose_hit == 0U) && (very_strong_exit_hit == 0U))
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
    beacon_detection_realtime_reset_candidate();
    s_realtime_cooldown_ticks = BEACON_DETECTION_CLUSTER_GAP_TICKS;
}

static void beacon_detection_realtime_remember_side_tail_pose_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t side_tail_pose_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U))
    {
        return;
    }

    side_tail_pose_hit =
        ((s_realtime_candidate.enter_age_ticks >= BEACON_DETECTION_SIDE_TAIL_POSE_TICKS) &&
         (s_realtime_candidate.enter_age_ticks <= 750U) &&
         (s_realtime_candidate.exit_age_ticks >= 620U) &&
         (s_realtime_candidate.exit_age_ticks <= 720U) &&
         (s_realtime_candidate.exit_score >= 1.60f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 60.0f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.12f) &&
         (s_realtime_candidate.max_score < 2.20f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count >= 90.0f) &&
         (beacon_detection_realtime_pose_axis_span() >= 1.0f)) ? 1U : 0U;

    if(side_tail_pose_hit == 0U)
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
    beacon_detection_realtime_reset_candidate();
    s_realtime_cooldown_ticks = BEACON_DETECTION_CLUSTER_GAP_TICKS;
}

static void beacon_detection_realtime_remember_slow_tail_pose_candidate(void)
{
    beacon_bump_location_t location;
    uint8_t left_mid_tail_hit;
    uint8_t right_low_pose_hit;
    uint8_t fast_side_medium_hit;
    uint8_t rear_quiet_late_hit;
    uint8_t front_weak_late_hit;

    if((s_realtime_candidate.early_accepted != 0U) ||
       (s_realtime_candidate.exit_valid == 0U))
    {
        return;
    }

    left_mid_tail_hit =
        ((s_realtime_candidate.enter_age_ticks >= 600U) &&
         (s_realtime_candidate.enter_age_ticks <= 700U) &&
         (s_realtime_candidate.exit_age_ticks >= 580U) &&
         (s_realtime_candidate.exit_age_ticks <= 660U) &&
         (s_realtime_candidate.exit_score >= 1.65f) &&
         (s_realtime_candidate.exit_score <= 2.05f) &&
         (s_realtime_candidate.exit_accel_norm_error_g >= 0.13f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.20f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 40.0f) &&
         (s_realtime_candidate.win_gyro_xy_dps <= 55.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count >= 35.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count <= 60.0f) &&
         (s_realtime_candidate.first_speed_mps >= 0.45f) &&
         (s_realtime_candidate.first_speed_mps <= 0.56f) &&
         (s_realtime_candidate.max_score < 2.05f) &&
         (beacon_detection_realtime_pose_axis_span() >= 1.0f)) ? 1U : 0U;

    right_low_pose_hit =
        ((s_realtime_candidate.enter_age_ticks >= 450U) &&
         (s_realtime_candidate.enter_age_ticks <= 600U) &&
         (s_realtime_candidate.exit_age_ticks >= 350U) &&
         (s_realtime_candidate.exit_age_ticks <= 400U) &&
         (s_realtime_candidate.exit_score >= 1.20f) &&
         (s_realtime_candidate.exit_score <= 1.40f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.07f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 40.0f) &&
         (s_realtime_candidate.win_gyro_xy_dps <= 55.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count >= 45.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count <= 75.0f) &&
         (s_realtime_candidate.first_speed_mps >= 0.45f) &&
         (s_realtime_candidate.first_speed_mps <= 0.58f) &&
         (s_realtime_candidate.max_score < 1.40f) &&
         (beacon_detection_realtime_pose_axis_span() >= 2.0f)) ? 1U : 0U;

    fast_side_medium_hit =
        ((s_realtime_candidate.enter_age_ticks >= 500U) &&
         (s_realtime_candidate.enter_age_ticks <= 600U) &&
         (s_realtime_candidate.exit_age_ticks >= 420U) &&
         (s_realtime_candidate.exit_age_ticks <= 500U) &&
         (s_realtime_candidate.exit_score >= 1.95f) &&
         (s_realtime_candidate.exit_score <= 2.20f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.10f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 65.0f) &&
         (s_realtime_candidate.max_score >= 1.90f) &&
         (s_realtime_candidate.max_score < 2.20f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count >= 50.0f) &&
         (s_realtime_candidate.first_speed_mps >= 0.90f) &&
         (beacon_detection_realtime_pose_axis_span() >= 2.0f)) ? 1U : 0U;

    rear_quiet_late_hit =
        ((s_realtime_candidate.enter_age_ticks >= 750U) &&
         (s_realtime_candidate.enter_age_ticks <= 800U) &&
         (s_realtime_candidate.exit_age_ticks >= 740U) &&
         (s_realtime_candidate.exit_age_ticks <= 800U) &&
         (s_realtime_candidate.exit_score >= 1.08f) &&
         (s_realtime_candidate.exit_score <= 1.30f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.075f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 48.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count <= 60.0f) &&
         (s_realtime_candidate.first_speed_mps >= 0.40f) &&
         (s_realtime_candidate.first_speed_mps <= 0.50f) &&
         (s_realtime_candidate.max_score < 1.40f) &&
         (beacon_detection_realtime_pose_axis_span() >= 1.5f)) ? 1U : 0U;

    front_weak_late_hit =
        ((s_realtime_candidate.enter_age_ticks >= 700U) &&
         (s_realtime_candidate.enter_age_ticks <= 800U) &&
         (s_realtime_candidate.exit_age_ticks >= 650U) &&
         (s_realtime_candidate.exit_age_ticks <= 700U) &&
         (s_realtime_candidate.exit_score >= 1.20f) &&
         (s_realtime_candidate.exit_score <= 1.35f) &&
         (s_realtime_candidate.exit_accel_norm_error_g >= 0.12f) &&
         (s_realtime_candidate.exit_accel_norm_error_g <= 0.15f) &&
         (s_realtime_candidate.win_gyro_xy_dps >= 30.0f) &&
         (s_realtime_candidate.win_gyro_xy_dps <= 38.0f) &&
         (s_realtime_candidate.window_max_wheel_highpass_count <= 35.0f) &&
         (s_realtime_candidate.first_speed_mps >= 0.42f) &&
         (s_realtime_candidate.first_speed_mps <= 0.52f) &&
         (s_realtime_candidate.max_score < 1.40f) &&
         (beacon_detection_realtime_pose_axis_span() >= 0.6f)) ? 1U : 0U;

    if((left_mid_tail_hit == 0U) &&
       (right_low_pose_hit == 0U) &&
       (fast_side_medium_hit == 0U) &&
       (rear_quiet_late_hit == 0U) &&
       (front_weak_late_hit == 0U))
    {
        return;
    }

    s_realtime_candidate.early_accepted = 1U;
    s_realtime_candidate.early_enter_sample = s_realtime_candidate.enter_sample;
    s_realtime_candidate.early_enter_location = s_realtime_candidate.enter_location;
    s_realtime_candidate.early_exit_sample = s_realtime_candidate.exit_sample;
    s_realtime_candidate.early_exit_location = s_realtime_candidate.exit_location;

    location = s_realtime_candidate.early_enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.early_exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.early_enter_sample,
                                1U,
                                1U,
                                location);
    beacon_detection_push_event(&s_realtime_candidate.early_exit_sample,
                                0U,
                                0U,
                                location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;

    if(right_low_pose_hit == 0U)
    {
        beacon_detection_realtime_reset_candidate();
        s_realtime_cooldown_ticks = BEACON_DETECTION_CLUSTER_GAP_TICKS;
    }
}

static void beacon_detection_realtime_accept_candidate(void)
{
    beacon_bump_location_t location;

    if(s_realtime_candidate.early_accepted != 0U)
    {
        beacon_detection_realtime_reset_candidate();
        return;
    }

    location = s_realtime_candidate.enter_location;
    if(location == BEACON_BUMP_LOCATION_UNKNOWN)
    {
        location = s_realtime_candidate.exit_location;
    }

    beacon_detection_push_event(&s_realtime_candidate.enter_sample, 1U, 1U, location);
    beacon_detection_push_event(&s_realtime_candidate.exit_sample, 0U, 0U, location);
    s_on_beacon_state = 0U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
    beacon_detection_realtime_reset_candidate();
}

static void beacon_detection_advance_realtime_cluster(const beacon_detection_imu_sample_t *window,
                                                      float roll_deg,
                                                      float pitch_deg)
{
    float score;
    float gated_score;
    uint8_t seed_high;

    score = beacon_detection_realtime_score(window);
    gated_score = beacon_detection_realtime_gated_score(score, window);
    g_beacon_detection.score = score;
    seed_high = beacon_detection_realtime_seed_is_high(gated_score);

    if(s_realtime_candidate.active != 0U)
    {
        beacon_detection_realtime_update_pose_span(roll_deg, pitch_deg);
        beacon_detection_realtime_update_window_maxima();
        if(s_realtime_candidate.enter_age_ticks < 65535U)
        {
            s_realtime_candidate.enter_age_ticks++;
        }
        if(s_realtime_candidate.last_peak_age_ticks < 65535U)
        {
            s_realtime_candidate.last_peak_age_ticks++;
        }
    }

    if((s_realtime_candidate.segment_active != 0U) &&
       (s_realtime_candidate.last_high_gap_ticks < 65535U))
    {
        s_realtime_candidate.last_high_gap_ticks++;
        s_realtime_candidate.segment_peak_age_ticks++;
    }

    if((s_realtime_candidate.active != 0U) &&
       (s_realtime_candidate.enter_age_ticks >= BEACON_DETECTION_PAIR_MIN_TICKS) &&
       (s_realtime_candidate.enter_age_ticks <= BEACON_DETECTION_PAIR_MAX_TICKS))
    {
        beacon_detection_realtime_update_exit_peak(window, gated_score);
        beacon_detection_realtime_remember_fast_exit_gate_candidate();
        beacon_detection_realtime_remember_weak_clean_tail_candidate();
        beacon_detection_realtime_remember_strong_tail_pose_candidate();
        beacon_detection_realtime_remember_side_tail_pose_candidate();
        beacon_detection_realtime_remember_slow_tail_pose_candidate();
        if(beacon_detection_realtime_strong_shape_valid() != 0U)
        {
            beacon_detection_realtime_remember_early_candidate();
        }
        if((s_realtime_candidate.early_max_gyro_z_abs_dps >= 50.0f) &&
           (s_realtime_candidate.win_gyro_xy_dps >= 45.5f))
        {
            beacon_detection_realtime_remember_early_candidate();
        }
        beacon_detection_realtime_remember_peak_closed_candidate();
        beacon_detection_realtime_remember_weak_candidate();
        if(s_realtime_candidate.enter_age_ticks >= BEACON_DETECTION_PAIR_MAX_TICKS)
        {
            beacon_detection_realtime_remember_early_candidate();
        }
    }

    if(seed_high != 0U)
    {
        if(s_realtime_cooldown_ticks > 0U)
        {
            return;
        }

        if(s_realtime_candidate.segment_active == 0U)
        {
            beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
            if(s_realtime_candidate.active == 0U)
            {
                beacon_detection_realtime_start_cluster();
            }
            else if((s_realtime_candidate.early_accepted == 0U) &&
                    (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_PAIR_MAX_TICKS) &&
                    (gated_score >= BEACON_DETECTION_EXPIRED_RESEED_SCORE))
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
            else if(beacon_detection_realtime_segment_starts_new_cluster() != 0U)
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
            else
            {
                s_realtime_candidate.segment_pending = 1U;
            }
        }
        else if(s_realtime_candidate.last_high_gap_ticks > BEACON_DETECTION_CANDIDATE_PEAK_GAP_TICKS)
        {
            beacon_detection_realtime_emit_segment_peak();
            beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
            if(s_realtime_candidate.active == 0U)
            {
                beacon_detection_realtime_start_cluster();
            }
            else if((s_realtime_candidate.early_accepted == 0U) &&
                    (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_PAIR_MAX_TICKS) &&
                    (gated_score >= BEACON_DETECTION_EXPIRED_RESEED_SCORE))
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
            else if(beacon_detection_realtime_segment_starts_new_cluster() != 0U)
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
            else
            {
                s_realtime_candidate.segment_pending = 1U;
            }
        }
        else
        {
            beacon_detection_realtime_update_segment(window, gated_score, roll_deg, pitch_deg);
            if(s_realtime_candidate.segment_pending == 0U)
            {
                beacon_detection_realtime_update_first_cluster_peak();
            }
            else if((s_realtime_candidate.early_accepted == 0U) &&
                    (s_realtime_candidate.enter_age_ticks > BEACON_DETECTION_PAIR_MAX_TICKS) &&
                    (gated_score >= BEACON_DETECTION_EXPIRED_RESEED_SCORE))
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
            else if(beacon_detection_realtime_segment_starts_new_cluster() != 0U)
            {
                if(beacon_detection_realtime_candidate_shape_valid() != 0U)
                {
                    beacon_detection_realtime_accept_candidate();
                }
                else
                {
                    beacon_detection_realtime_reset_candidate();
                }
                beacon_detection_realtime_start_segment(window, gated_score, roll_deg, pitch_deg);
                beacon_detection_realtime_start_cluster();
            }
        }
    }
    else if((s_realtime_candidate.segment_active != 0U) &&
            (s_realtime_candidate.last_high_gap_ticks > BEACON_DETECTION_CANDIDATE_PEAK_GAP_TICKS))
    {
        beacon_detection_realtime_emit_segment_peak();
    }

    beacon_detection_realtime_maybe_finish_cluster();
}

static void beacon_detection_accept_enter(const beacon_detection_enter_candidate_t *candidate,
                                          beacon_bump_location_t location)
{
    if((s_on_beacon_state != 0U) || (s_exit_track_active != 0U))
    {
        beacon_detection_capture_deferred_exit();
        if(s_deferred_exit_valid != 0U)
        {
            beacon_detection_push_event(&s_deferred_exit_sample, 0U, 0U, s_deferred_exit_location);
            s_deferred_exit_valid = 0U;
        }
        beacon_detection_reset_exit_track();
        s_on_beacon_state = 0U;
    }

    beacon_detection_push_event(&candidate->peak, 1U, 1U, location);
    s_on_beacon_state = 1U;
    g_beacon_detection.on_beacon = s_on_beacon_state;
    beacon_detection_start_exit_track();
}

static void beacon_detection_process_pending_enters(void)
{
    uint8_t i;
    uint8_t j;
    beacon_bump_location_t location;

    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        if(s_pending_enter[i].valid == 0U)
        {
            continue;
        }
        s_pending_enter[i].age_ticks++;

        if(s_pending_enter[i].age_ticks < BEACON_DETECTION_ENTER_CONFIRM_TICKS)
        {
            continue;
        }

        if(s_pending_enter[i].suppressed == 0U)
        {
            for(j = 0U; j < BEACON_DETECTION_PENDING_ENTER_MAX; j++)
            {
                if((i != j) &&
                   (s_pending_enter[j].valid != 0U) &&
                   (s_pending_enter[j].age_ticks < s_pending_enter[i].age_ticks) &&
                   (s_pending_enter[j].peak.impact_score >= 1.10f) &&
                   (beacon_detection_candidate_is_weak_only(&s_pending_enter[i]) != 0U))
                {
                    s_pending_enter[i].suppressed = 1U;
                }
            }
            if((s_active_enter.valid != 0U) &&
               (s_active_enter.peak.impact_score >= 1.10f) &&
               (beacon_detection_candidate_is_weak_only(&s_pending_enter[i]) != 0U))
            {
                s_pending_enter[i].suppressed = 1U;
            }
        }

        if((s_pending_enter[i].suppressed == 0U) &&
           (beacon_detection_is_enter_valid(&s_pending_enter[i], &location) != 0U))
        {
            beacon_detection_accept_enter(&s_pending_enter[i], location);
        }

        beacon_detection_clear_enter_candidate(&s_pending_enter[i]);
    }
}

static void beacon_detection_update_active_enter(const beacon_detection_imu_sample_t *window)
{
    uint8_t main_seed;
    uint8_t weak_seed;
    uint8_t active_main;

    main_seed = beacon_detection_is_main_enter_seed(window);
    weak_seed = beacon_detection_is_weak_enter_seed(window);
    if(s_active_enter.valid == 0U)
    {
        if((main_seed != 0U) || (weak_seed != 0U))
        {
            beacon_detection_init_enter_candidate(&s_active_enter, window);
        }
        return;
    }

    active_main = (s_active_enter.peak.impact_score >= 0.75f) ? 1U : 0U;
    if((active_main == 0U) && (main_seed != 0U))
    {
        beacon_detection_pending_add(&s_active_enter);
        beacon_detection_init_enter_candidate(&s_active_enter, window);
        return;
    }

    beacon_detection_observe_enter_candidate(&s_active_enter, window);
    active_main = (s_active_enter.peak.impact_score >= 0.75f) ? 1U : 0U;
    if(((active_main != 0U) && (main_seed != 0U)) ||
       ((active_main == 0U) && (weak_seed != 0U)))
    {
        s_active_enter.quiet_ticks = 0U;
    }

    if(s_active_enter.quiet_ticks >= BEACON_DETECTION_ENTER_SEGMENT_GAP_TICKS)
    {
        beacon_detection_pending_add(&s_active_enter);
        beacon_detection_clear_enter_candidate(&s_active_enter);
    }
}

static void beacon_detection_advance_realtime(const beacon_detection_imu_sample_t *window,
                                              float roll_deg,
                                              float pitch_deg)
{
    beacon_detection_advance_realtime_cluster(window, roll_deg, pitch_deg);
}

void beacon_detection_reset(void)
{
    uint8_t i;

    memset(&g_beacon_detection, 0, sizeof(g_beacon_detection));
    g_beacon_detection.location = BEACON_BUMP_LOCATION_UNKNOWN;
    g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;

    beacon_detection_clear_enter_candidate(&s_active_enter);
    for(i = 0U; i < BEACON_DETECTION_PENDING_ENTER_MAX; i++)
    {
        beacon_detection_clear_enter_candidate(&s_pending_enter[i]);
        memset(&s_event_queue[i], 0, sizeof(s_event_queue[i]));
    }
    beacon_detection_reset_exit_track();
    memset(&s_deferred_exit_sample, 0, sizeof(s_deferred_exit_sample));
    s_deferred_exit_location = BEACON_BUMP_LOCATION_UNKNOWN;
    s_deferred_exit_valid = 0U;
    s_motion_reference_vel[0] = 0.0f;
    s_motion_reference_vel[1] = 0.0f;
    s_motion_reference_speed_mps = 0.0f;
    s_roll_zero_deg = 0.0f;
    s_pitch_zero_deg = 0.0f;
    s_prev_roll_deg = 0.0f;
    s_prev_pitch_deg = 0.0f;
    s_impact_baseline = 0.20f;
    s_impact_deviation = 0.20f;
    s_startup_ticks = BEACON_DETECTION_STARTUP_TICKS;
    s_runtime_ticks = 0U;
    s_event_ticks = 0U;
    s_event_queue_head = 0U;
    s_event_queue_tail = 0U;
    s_event_queue_count = 0U;
    s_on_beacon_state = 0U;
    s_realtime_cooldown_ticks = 0U;
    s_tilt_ready = 0U;

    for(i = 0U; i < 4U; i++)
    {
        s_wheel_lpf[i] = 0.0f;
        s_wheel_highpass[i] = 0.0f;
    }
    beacon_detection_clear_history();
    beacon_detection_realtime_reset_candidate();
}

void beacon_detection_update_1000HZ(void)
{
    beacon_detection_imu_sample_t sample;
    beacon_detection_imu_sample_t window;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float roll_delta;
    float pitch_delta;
    float accel_norm_g;
    float gyro_score;
    float tilt_rate_score;
    float accel_score;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;

    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    roll_deg = g_euler.roll;
    pitch_deg = g_euler.pitch;
    if(0U == s_tilt_ready)
    {
        s_roll_zero_deg = roll_deg;
        s_pitch_zero_deg = pitch_deg;
        s_prev_roll_deg = roll_deg;
        s_prev_pitch_deg = pitch_deg;
        s_tilt_ready = 1U;
    }

    roll_delta = roll_deg - s_roll_zero_deg;
    pitch_delta = pitch_deg - s_pitch_zero_deg;
    sample.tilt_deg = beacon_detection_vec2_norm(roll_delta, pitch_delta);
    sample.tilt_rate_dps =
        beacon_detection_vec2_norm(roll_deg - s_prev_roll_deg,
                                   pitch_deg - s_prev_pitch_deg) /
        BEACON_DETECTION_IMU_DT_S;
    sample.roll_rate_dps = car_math_absf(roll_deg - s_prev_roll_deg) / BEACON_DETECTION_IMU_DT_S;
    sample.pitch_rate_dps = car_math_absf(pitch_deg - s_prev_pitch_deg) / BEACON_DETECTION_IMU_DT_S;
    s_prev_roll_deg = roll_deg;
    s_prev_pitch_deg = pitch_deg;

    sample.gyro_xy_dps = beacon_detection_vec2_norm(gyro_x_dps, gyro_y_dps);
    sample.gyro_z_abs_dps = car_math_absf(gyro_z_dps);
    accel_norm_g = beacon_detection_vec3_norm(accel_x_g, accel_y_g, accel_z_g);
    sample.accel_norm_error_g = car_math_absf(accel_norm_g - 1.0f);

    gyro_score = beacon_detection_ratio(sample.gyro_xy_dps, 45.0f);
    tilt_rate_score = beacon_detection_ratio(sample.tilt_rate_dps, 45.0f);
    accel_score = beacon_detection_ratio(sample.accel_norm_error_g, 0.12f);
    sample.impact_score = car_math_minf(car_math_minf(gyro_score, tilt_rate_score), accel_score);

    g_beacon_detection.gyro_xy_dps = sample.gyro_xy_dps;
    g_beacon_detection.gyro_z_abs_dps = sample.gyro_z_abs_dps;
    g_beacon_detection.tilt_rate_dps = sample.tilt_rate_dps;
    g_beacon_detection.tilt_deg = sample.tilt_deg;
    g_beacon_detection.accel_norm_error_g = sample.accel_norm_error_g;

    beacon_detection_push_imu_sample(&sample);
    beacon_detection_get_imu_window_max(&window);
    beacon_detection_update_adaptive_baseline(window.impact_score);

    if(s_startup_ticks > 0U)
    {
        s_startup_ticks--;
    }
    else
    {
        s_runtime_ticks++;
        if(s_realtime_cooldown_ticks > 0U)
        {
            s_realtime_cooldown_ticks--;
        }
        beacon_detection_advance_realtime(&window, roll_deg, pitch_deg);
    }

    if(s_event_ticks > 0U)
    {
        s_event_ticks--;
        g_beacon_detection.hold_ticks = s_event_ticks;
        if(0U == s_event_ticks)
        {
            g_beacon_detection.bump_detected = 0U;
            g_beacon_detection.enter_event = 0U;
            g_beacon_detection.exit_event = 0U;
            g_beacon_detection.confidence = BEACON_BUMP_CONFIDENCE_NONE;
            g_beacon_detection.on_beacon = s_on_beacon_state;
            beacon_detection_dispatch_queued_event();
        }
    }
    else
    {
        beacon_detection_dispatch_queued_event();
    }

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();


}

void beacon_detection_update_100HZ(void)
{
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float beta;
    float wheel_highpass_sum;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    /*
     * 检测模块内部速度仍按运动方向存放：
     * vel[0] = forward，vel[1] = right/strafe。
     * 不等同于 g_odometer 的全局坐标轴 [x=右, y=前]，别在这里跟着全局轴换下标。
     */
    g_beacon_detection.vel[0] =
        (left_front + right_front + left_rear + right_rear) *
        (0.25f / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S);
    g_beacon_detection.vel[1] =
        (left_front - right_front - left_rear + right_rear) *
        (0.25f / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S);
    g_beacon_detection.speed_mps =
        beacon_detection_vec2_norm(g_beacon_detection.vel[0],
                                   g_beacon_detection.vel[1]);
    if(g_beacon_detection.speed_mps > 0.12f)
    {
        s_motion_reference_vel[0] = g_beacon_detection.vel[0];
        s_motion_reference_vel[1] = g_beacon_detection.vel[1];
        s_motion_reference_speed_mps = g_beacon_detection.speed_mps;
    }
    else
    {
        s_motion_reference_vel[0] *= BEACON_DETECTION_MOTION_REF_DECAY;
        s_motion_reference_vel[1] *= BEACON_DETECTION_MOTION_REF_DECAY;
        s_motion_reference_speed_mps *= BEACON_DETECTION_MOTION_REF_DECAY;
    }

    beta = ODOMETER_UPDATE_DT_S / (BEACON_DETECTION_WHEEL_HPF_TAU_S + ODOMETER_UPDATE_DT_S);
    s_wheel_lpf[0] += beta * (left_front - s_wheel_lpf[0]);
    s_wheel_lpf[1] += beta * (right_front - s_wheel_lpf[1]);
    s_wheel_lpf[2] += beta * (left_rear - s_wheel_lpf[2]);
    s_wheel_lpf[3] += beta * (right_rear - s_wheel_lpf[3]);

    s_wheel_highpass[0] = left_front - s_wheel_lpf[0];
    s_wheel_highpass[1] = right_front - s_wheel_lpf[1];
    s_wheel_highpass[2] = left_rear - s_wheel_lpf[2];
    s_wheel_highpass[3] = right_rear - s_wheel_lpf[3];

    g_beacon_detection.wheel_highpass_count = car_math_absf(s_wheel_highpass[0]);
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[1]));
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[2]));
    g_beacon_detection.wheel_highpass_count =
        car_math_maxf(g_beacon_detection.wheel_highpass_count, car_math_absf(s_wheel_highpass[3]));
    wheel_highpass_sum = beacon_detection_wheel_highpass_sum_abs();
    if((s_active_enter.valid != 0U) &&
       (wheel_highpass_sum > s_active_enter.wheel_highpass_sum_max))
    {
        s_active_enter.wheel_highpass_sum_max = wheel_highpass_sum;
    }
}
