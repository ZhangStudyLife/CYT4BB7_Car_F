#ifndef _BEACON_DETECTION_H_
#define _BEACON_DETECTION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BEACON_BUMP_LOCATION_UNKNOWN = 0,
    BEACON_BUMP_LOCATION_FRONT,
    BEACON_BUMP_LOCATION_RIGHT,
    BEACON_BUMP_LOCATION_LEFT,
    BEACON_BUMP_LOCATION_REAR
} beacon_bump_location_t;

typedef enum
{
    BEACON_BUMP_CONFIDENCE_NONE = 0,
    BEACON_BUMP_CONFIDENCE_LOW,
    BEACON_BUMP_CONFIDENCE_HIGH
} beacon_bump_confidence_t;

#define BEACON_BUMP_WHEEL_LF_MASK (0x01U)
#define BEACON_BUMP_WHEEL_RF_MASK (0x02U)
#define BEACON_BUMP_WHEEL_LR_MASK (0x04U)
#define BEACON_BUMP_WHEEL_RR_MASK (0x08U)

typedef struct
{
    uint8_t bump_detected;
    uint8_t on_beacon;
    uint8_t enter_event;
    uint8_t exit_event;

    beacon_bump_confidence_t confidence;
    beacon_bump_location_t location;
    uint8_t wheel_mask;

    uint16_t hold_ticks;
    uint32_t event_count;
    uint32_t enter_count;
    uint32_t exit_count;

    float score;
    float impact_baseline;
    float impact_robust_z;
    float speed_mps;
    float vel[2];

    float gyro_xy_dps;
    float gyro_z_abs_dps;
    float tilt_rate_dps;
    float tilt_deg;
    float accel_norm_error_g;
    float wheel_highpass_count;
} beacon_detection_data_t;

extern beacon_detection_data_t g_beacon_detection;

void beacon_detection_reset(void);
void beacon_detection_update_1000HZ(void);
void beacon_detection_update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
