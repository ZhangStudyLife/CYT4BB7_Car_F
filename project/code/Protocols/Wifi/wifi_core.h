#ifndef WIFI_CORE_H
#define WIFI_CORE_H

#include "zf_common_headfile.h"

#include "wifi_cmd/wifi_cmd.h"
#include "wifi_cal_imu/wifi_cal_imu.h"
#include "wifi_justfloat/wifi_justfloat.h"
#include "wifi_params/wifi_params.h"

void wifi_core_Init(void);
void wifi_core_Poll(void);
void wifi_core_UpdateStandbyContext(void);
uint8_t wifi_core_IsReady(void);

#endif /* WIFI_CORE_H */
