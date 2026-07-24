#include "beacon_config.h"

#define BEACON_CONFIG_FLASH_MAGIC   (0x42434647UL)
#define BEACON_CONFIG_FLASH_VERSION (2U)
#define BEACON_CONFIG_COORD_LIMIT_M (1000.0f)

#if (BEACON_CONFIG_FLASH_PAGE >= FLASH_PAGE_NUM)
#error "Beacon config Flash page exceeds Work Flash range"
#endif

typedef struct
{
    uint32 magic;
    uint16 version;
    uint8 legacy_source;
    uint8 reserved;
    beacon_config_data_t predata;
    uint32 checksum;
} beacon_config_flash_data_t;

typedef char beacon_config_flash_data_must_be_word_aligned[
    ((sizeof(beacon_config_flash_data_t) % sizeof(uint32)) == 0U) ? 1 : -1];
typedef char beacon_config_flash_data_must_fit_page[
    (sizeof(beacon_config_flash_data_t) <= FLASH_PAGE_SIZE) ? 1 : -1];

/*
 * 比赛前在这里登记信标坐标。
 * 坐标含义：车体中心位于信标中心时的全局坐标，单位 m。
 * X 正方向为右侧。
 * Y 正方向为前方。
 * 数组下标0至5依次对应灯序识别中的1号至6号灯。
 * 发车区坐标作为 odometer 初始全局坐标。
 */
static const beacon_config_data_t s_default_predata =
{
    {3.25f, 0.25f},
    {
        {2.2f, 2.0f},
        {1.6f, 3.7f},
        {2.9f, 4.8f},
        {3.1f, 3.2f},
        {4.1f, 2.1f},
        {4.5f, 3.9f},
    }
};

static beacon_config_data_t s_beacon_config;

static uint32 beacon_config_checksum_mix(uint32 checksum, uint32 value)
{
    checksum ^= value;
    checksum = (checksum << 7) | (checksum >> 25);
    checksum += 0x9E3779B9UL;
    return checksum;
}

static uint32 beacon_config_calc_checksum(const beacon_config_flash_data_t *data)
{
    uint32 checksum = 0x2468ACE0UL;
    uint32 value_bits;
    uint16 index;

    if(data == NULL)
    {
        return 0U;
    }

    checksum = beacon_config_checksum_mix(checksum, data->magic);
    checksum = beacon_config_checksum_mix(checksum, data->version);
    checksum = beacon_config_checksum_mix(checksum, data->legacy_source);

    for(index = 0U; index < 2U; index++)
    {
        memcpy(&value_bits, &data->predata.initial_position[index], sizeof(value_bits));
        checksum = beacon_config_checksum_mix(checksum, value_bits);
    }

    for(index = 0U; index < BEACON_CONFIG_BEACON_COUNT; index++)
    {
        memcpy(&value_bits, &data->predata.beacons[index].x, sizeof(value_bits));
        checksum = beacon_config_checksum_mix(checksum, value_bits);
        memcpy(&value_bits, &data->predata.beacons[index].y, sizeof(value_bits));
        checksum = beacon_config_checksum_mix(checksum, value_bits);
    }

    return checksum;
}

static uint8 beacon_config_value_valid(float value)
{
    return ((value == value) &&
            (value >= -BEACON_CONFIG_COORD_LIMIT_M) &&
            (value <= BEACON_CONFIG_COORD_LIMIT_M)) ? 1U : 0U;
}

static uint8 beacon_config_predata_valid(const beacon_config_data_t *data)
{
    uint16 index;

    if((data == NULL) ||
       (beacon_config_value_valid(data->initial_position[x]) == 0U) ||
       (beacon_config_value_valid(data->initial_position[y]) == 0U))
    {
        return 0U;
    }

    for(index = 0U; index < BEACON_CONFIG_BEACON_COUNT; index++)
    {
        if((beacon_config_value_valid(data->beacons[index].x) == 0U) ||
           (beacon_config_value_valid(data->beacons[index].y) == 0U))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8 beacon_config_flash_data_valid(const beacon_config_flash_data_t *data)
{
    if((data == NULL) ||
       (data->magic != BEACON_CONFIG_FLASH_MAGIC) ||
       (data->version != BEACON_CONFIG_FLASH_VERSION) ||
       (data->legacy_source > 1U) ||
       (beacon_config_predata_valid(&data->predata) == 0U))
    {
        return 0U;
    }

    return (data->checksum == beacon_config_calc_checksum(data)) ? 1U : 0U;
}

static uint8 beacon_config_read_flash(beacon_config_flash_data_t *data)
{
    uint32 words;

    if((data == NULL) || (flash_check(0U, BEACON_CONFIG_FLASH_PAGE) == 0U))
    {
        return 0U;
    }

    words = (uint32)(sizeof(*data) / sizeof(uint32));
    memset(data, 0, sizeof(*data));
    flash_read_page(0U, BEACON_CONFIG_FLASH_PAGE, (uint32 *)data, words);
    return beacon_config_flash_data_valid(data);
}

static uint8 beacon_config_write(const beacon_config_data_t *config)
{
    beacon_config_flash_data_t data;
    beacon_config_flash_data_t verify;
    uint32 words;

    if(beacon_config_predata_valid(config) == 0U)
    {
        return 0U;
    }

    memset(&data, 0, sizeof(data));
    data.magic = BEACON_CONFIG_FLASH_MAGIC;
    data.version = BEACON_CONFIG_FLASH_VERSION;
    data.legacy_source = 1U;
    data.predata = *config;
    data.checksum = beacon_config_calc_checksum(&data);
    words = (uint32)(sizeof(data) / sizeof(uint32));

    flash_write_page(0U,
                     BEACON_CONFIG_FLASH_PAGE,
                     (const uint32 *)&data,
                     words);
    memset(&verify, 0, sizeof(verify));
    flash_read_page(0U,
                    BEACON_CONFIG_FLASH_PAGE,
                    (uint32 *)&verify,
                    words);
    if(beacon_config_flash_data_valid(&verify) == 0U)
    {
        return 0U;
    }

    s_beacon_config = *config;
    return 1U;
}

void beacon_config_init(void)
{
    beacon_config_flash_data_t data;

    beacon_config_reset();
    if(beacon_config_read_flash(&data) != 0U)
    {
        s_beacon_config = data.predata;
    }
}

void beacon_config_reset(void)
{
    s_beacon_config = s_default_predata;
}

uint16 beacon_config_get_count(void)
{
    return BEACON_CONFIG_BEACON_COUNT;
}

uint8 beacon_config_get_beacon(uint16 index, beacon_config_point_t *beacon)
{
    if((beacon == NULL) || (index >= beacon_config_get_count()))
    {
        return 0U;
    }

    *beacon = s_beacon_config.beacons[index];
    return 1U;
}

void beacon_config_get_initial_position(float position[2])
{
    if(position == NULL)
    {
        return;
    }

    position[x] = s_beacon_config.initial_position[x];
    position[y] = s_beacon_config.initial_position[y];
}

void beacon_config_get_predata(beacon_config_data_t *data)
{
    if(data != NULL)
    {
        *data = s_beacon_config;
    }
}

uint8 beacon_config_save_predata(const beacon_config_data_t *data)
{
    return beacon_config_write(data);
}
