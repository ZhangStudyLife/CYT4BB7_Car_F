/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
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
