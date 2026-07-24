#include "light_sequence.h"

#include <string.h>

#include "light_sequence_config.h"

#define LIGHT_SEQUENCE_MAX_LIGHTS_PER_ROUND (3U)    /* 每轮允许同时亮起的最大灯数 */

typedef struct
{
    uint8 valid;
    uint8 round_index;
    uint8 remaining_mask;
} light_sequence_candidate_t;

/* 各套序列的当前筛选进度。 */
static light_sequence_candidate_t s_candidates[LIGHT_SEQUENCE_PRESET_COUNT];

/* 当前灯号与序列识别结果。 */
static light_sequence_result_t s_result;

/* 上一次100Hz更新时的Air熄灯标志，用于检测0到1上升沿。 */
static uint8 s_last_beacon_lost_flag;

/* 各盏灯最近一次被正式接受的时间戳，单位ms。 */
static uint32 s_last_accepted_time_ms[LIGHT_SEQUENCE_BEACON_COUNT];

/* 最近接受时间有效位，bit0至bit5分别对应1号至6号灯。 */
static uint8 s_accepted_time_valid_mask;

/**
 * @brief 将十进制灯号组合转换为内部位掩码。
 * @param round_lights 十进制灯号组合，例如17表示1号和7号灯。
 * @return 转换成功返回灯位掩码，配置非法返回0。
 */
static uint8 LightSequence_DecodeRound(uint16 round_lights)
{
    uint8 round_mask = 0U;
    uint8 light_count = 0U;

    while(round_lights != 0U)
    {
        uint8 beacon_id = (uint8)(round_lights % 10U);
        uint8 beacon_bit;

        if((beacon_id == 0U) || (beacon_id > LIGHT_SEQUENCE_BEACON_COUNT))
        {
            return 0U;
        }

        beacon_bit = (uint8)(1U << (beacon_id - 1U));
        if((round_mask & beacon_bit) != 0U)
        {
            return 0U;
        }

        round_mask = (uint8)(round_mask | beacon_bit);
        light_count++;
        if(light_count > LIGHT_SEQUENCE_MAX_LIGHTS_PER_ROUND)
        {
            return 0U;
        }

        round_lights /= 10U;
    }

    return round_mask;
}

/**
 * @brief 检查预设序列的轮数、灯号和相邻轮规则是否合法。
 * @param preset 待检查的预设序列。
 * @return 配置有效返回1，否则返回0。
 */
static uint8 LightSequence_PresetValid(const light_sequence_preset_t *preset)
{
    uint8 previous_round_mask = 0U;
    uint8 round_index;

    if((preset == 0) ||
       (preset->round_count == 0U) ||
       (preset->round_count > LIGHT_SEQUENCE_MAX_ROUNDS))
    {
        return 0U;
    }

    for(round_index = 0U; round_index < preset->round_count; round_index++)
    {
        uint8 round_mask = LightSequence_DecodeRound(preset->round_lights[round_index]);

        if(round_mask == 0U)
        {
            return 0U;
        }

        if((round_index > 0U) &&
           ((round_mask & previous_round_mask) != 0U))
        {
            return 0U;
        }

        previous_round_mask = round_mask;
    }

    return 1U;
}

/**
 * @brief 根据修正后的车辆坐标在Predata地图中查找匹配半径内最近的信标灯。
 * @param car_position_x 车辆全局X坐标，单位m。
 * @param car_position_y 车辆全局Y坐标，单位m。
 * @return 匹配成功返回1至6号灯，匹配失败返回0。
 */
static uint8 LightSequence_FindNearestBeacon(float car_position_x,
                                             float car_position_y)
{
    const float match_radius_sq =
        LIGHT_SEQUENCE_MATCH_RADIUS_M * LIGHT_SEQUENCE_MATCH_RADIUS_M;
    beacon_config_data_t map_data;
    float nearest_distance_sq = 0.0f;
    uint8 nearest_beacon_id = LIGHT_SEQUENCE_BEACON_ID_NONE;
    uint8 beacon_index;

    beacon_config_get_predata(&map_data);
    for(beacon_index = 0U; beacon_index < LIGHT_SEQUENCE_BEACON_COUNT; beacon_index++)
    {
        const beacon_config_point_t *beacon = &map_data.beacons[beacon_index];
        float dx;
        float dy;
        float distance_sq;

        dx = car_position_x - beacon->x;
        dy = car_position_y - beacon->y;
        distance_sq = (dx * dx) + (dy * dy);

        if((nearest_beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE) ||
           (distance_sq < nearest_distance_sq))
        {
            nearest_distance_sq = distance_sq;
            nearest_beacon_id = (uint8)(beacon_index + 1U);
        }
    }

    if((nearest_beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE) ||
       (nearest_distance_sq > match_radius_sq))
    {
        return LIGHT_SEQUENCE_BEACON_ID_NONE;
    }

    return nearest_beacon_id;
}

/**
 * @brief 根据有效候选数量刷新序列编号和识别状态。
 * @param 无。
 * @return 无。
 */
static void LightSequence_RefreshResult(void)
{
    uint16 candidate_mask = 0U;
    uint8 candidate_count = 0U;
    uint8 sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    uint8 sequence_index;

    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        if(s_candidates[sequence_index].valid != 0U)
        {
            candidate_mask =
                (uint16)(candidate_mask | (uint16)(1U << sequence_index));
            candidate_count++;
            sequence_id = (uint8)(sequence_index + 1U);
        }
    }

    s_result.candidate_mask = candidate_mask;
    s_result.candidate_count = candidate_count;
    if(candidate_count == 0U)
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_FAILED;
        s_result.sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    }
    else if(candidate_count == 1U)
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_IDENTIFIED;
        s_result.sequence_id = sequence_id;
    }
    else
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_IDENTIFYING;
        s_result.sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    }
}

/**
 * @brief 复位灯号与序列识别状态，重新加载全部候选。
 * @param 无。
 * @return 无。
 */
void LightSequence_Reset(void)
{
    uint8 valid_count = 0U;
    uint8 sequence_index;

    memset(s_candidates, 0, sizeof(s_candidates));
    memset(&s_result, 0, sizeof(s_result));
    memset(s_last_accepted_time_ms, 0, sizeof(s_last_accepted_time_ms));
    s_last_beacon_lost_flag = 0U;
    s_accepted_time_valid_mask = 0U;

    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        if(LightSequence_PresetValid(&g_light_sequence_presets[sequence_index]) != 0U)
        {
            s_candidates[sequence_index].valid = 1U;
            s_candidates[sequence_index].round_index = 0U;
            s_candidates[sequence_index].remaining_mask =
                LightSequence_DecodeRound(
                    g_light_sequence_presets[sequence_index].round_lights[0]);
            valid_count++;
        }
    }

    if(valid_count != LIGHT_SEQUENCE_PRESET_COUNT)
    {
        memset(s_candidates, 0, sizeof(s_candidates));
        s_result.status = LIGHT_SEQUENCE_STATUS_CONFIG_ERROR;
        return;
    }

    LightSequence_RefreshResult();
}

/**
 * @brief 在Air熄灯标志上升沿匹配灯号，并筛选预设亮灯序列。
 * @param beacon_lost_flag Air当前熄灯标志，0表示未熄灯，非0表示检测到熄灯。
 * @param car_position_x 修正后的车辆全局X坐标，单位m。
 * @param car_position_y 修正后的车辆全局Y坐标，单位m。
 * @param current_time_ms Car端当前时间戳，单位ms。
 * @return 无。
 */
void LightSequence_Update(uint8 beacon_lost_flag,
                          float car_position_x,
                          float car_position_y,
                          uint32 current_time_ms)
{
    uint8 event_accepted = 0U;
    uint8 beacon_id;
    uint8 beacon_bit;
    uint8 beacon_index;
    uint8 sequence_index;

    beacon_lost_flag = (beacon_lost_flag != 0U) ? 1U : 0U;
    if((beacon_lost_flag == 0U) || (s_last_beacon_lost_flag != 0U))
    {
        s_last_beacon_lost_flag = beacon_lost_flag;
        return;
    }
    s_last_beacon_lost_flag = beacon_lost_flag;

    s_result.last_beacon_id = LIGHT_SEQUENCE_BEACON_ID_NONE;
    beacon_id = LightSequence_FindNearestBeacon(car_position_x, car_position_y);
    if(beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE)
    {
        return;
    }
    s_result.last_beacon_id = beacon_id;
    s_result.matched_event_count++;

    if(s_result.status == LIGHT_SEQUENCE_STATUS_CONFIG_ERROR)
    {
        return;
    }

    beacon_bit = (uint8)(1U << (beacon_id - 1U));
    beacon_index = (uint8)(beacon_id - 1U);
    if(((s_accepted_time_valid_mask & beacon_bit) != 0U) &&
       ((uint32)(current_time_ms - s_last_accepted_time_ms[beacon_index]) <
        LIGHT_SEQUENCE_DUPLICATE_WINDOW_MS))
    {
        return;
    }

    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        light_sequence_candidate_t *candidate = &s_candidates[sequence_index];
        const light_sequence_preset_t *preset = &g_light_sequence_presets[sequence_index];
        uint8 current_round_mask;
        uint8 previous_round_mask;
        uint8 consumed_mask;

        if(candidate->valid == 0U)
        {
            continue;
        }

        if(candidate->round_index >= preset->round_count)
        {
            previous_round_mask = LightSequence_DecodeRound(
                preset->round_lights[preset->round_count - 1U]);
            if((previous_round_mask & beacon_bit) == 0U)
            {
                candidate->valid = 0U;
            }
            continue;
        }

        current_round_mask = LightSequence_DecodeRound(
            preset->round_lights[candidate->round_index]);
        consumed_mask =
            (uint8)(current_round_mask & (uint8)(~candidate->remaining_mask));
        previous_round_mask = (candidate->round_index > 0U)
                                  ? LightSequence_DecodeRound(
                                        preset->round_lights[candidate->round_index - 1U])
                                  : 0U;

        if((candidate->remaining_mask & beacon_bit) != 0U)
        {
            event_accepted = 1U;
            candidate->remaining_mask =
                (uint8)(candidate->remaining_mask & (uint8)(~beacon_bit));
            if(candidate->remaining_mask == 0U)
            {
                candidate->round_index++;
                if(candidate->round_index < preset->round_count)
                {
                    candidate->remaining_mask = LightSequence_DecodeRound(
                        preset->round_lights[candidate->round_index]);
                }
            }
        }
        else if(((consumed_mask | previous_round_mask) & beacon_bit) == 0U)
        {
            candidate->valid = 0U;
        }
    }

    if(event_accepted != 0U)
    {
        s_last_accepted_time_ms[beacon_index] = current_time_ms;
        s_accepted_time_valid_mask =
            (uint8)(s_accepted_time_valid_mask | beacon_bit);
        s_result.accepted_event_count++;
    }

    LightSequence_RefreshResult();
}

/**
 * @brief 获取当前灯号与序列识别结果。
 * @param result 输出结果指针，传入空指针时不执行复制。
 * @return 无。
 */
void LightSequence_GetResult(light_sequence_result_t *result)
{
    if(result == 0)
    {
        return;
    }

    *result = s_result;
}
