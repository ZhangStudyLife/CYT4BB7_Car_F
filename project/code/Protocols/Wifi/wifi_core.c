#include "wifi_core.h"

static uint8_t s_wifi_core_inited = 0U;

void wifi_core_Init(void)
{
    if (0U != s_wifi_core_inited)
    {
        return;
    }

    wifi_cmd_Init();
    wifi_params_Init();
    wifi_cal_imu_Init();
    wifi_justfloat_Init();
    wifi_justfloat_SetStandbyContext(1U);
    wifi_justfloat_SetStandbyUserEnable(1U);
    s_wifi_core_inited = 1U;
}

void wifi_core_Poll(void)
{
    if (0U == s_wifi_core_inited)
    {
        return;
    }

    wifi_cmd_Poll();
}

void wifi_core_UpdateStandbyContext(void)
{
    wifi_justfloat_SetStandbyContext(1U);
}

uint8_t wifi_core_IsReady(void)
{
    return wifi_cmd_IsReady();
}
