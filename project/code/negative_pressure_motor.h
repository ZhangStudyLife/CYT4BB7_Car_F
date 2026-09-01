#ifndef NEGATIVE_PRESSURE_MOTOR_H
#define NEGATIVE_PRESSURE_MOTOR_H

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 电调使用 400 Hz PWM，1000~2000 us 脉宽对应 duty 4000~8000。 */
#define NEGATIVE_PRESSURE_PWM_FREQ              (400U)
#define NEGATIVE_PRESSURE_INPUT_MAX             (10000U)
#define NEGATIVE_PRESSURE_THROTTLE_LIMIT_MAX    (6000U)
#define NEGATIVE_PRESSURE_ESC_ARM_DELAY_MS      (3000U)

typedef enum
{
    NEGATIVE_PRESSURE_MOTOR_LEFT = 0,
    NEGATIVE_PRESSURE_MOTOR_RIGHT,
    NEGATIVE_PRESSURE_MOTOR_COUNT
} negative_pressure_motor_e;

/**
 * @brief 初始化左右负压电调 PWM，并保持最低油门完成上电识别。
 * @note  左电调信号口为 P05_0（P50），右电调信号口为 P05_2（P52）。
 */
void negative_pressure_init(void);

/** @brief 解锁负压电机输出；解锁后仍保持最低油门。 */
void negative_pressure_enable(void);

/** @brief 锁定并立即将左右电调恢复到最低油门。 */
void negative_pressure_disable(void);

/** @return 1 表示已解锁，0 表示已锁定。 */
uint8 negative_pressure_is_enabled(void);

/**
 * @brief 设置单路负压电机油门。
 * @param motor 左/右负压电机。
 * @param throttle 输入范围 0~10000，当前安全上限为 6000。
 */
void negative_pressure_set_motor(negative_pressure_motor_e motor, uint16 throttle);

/**
 * @brief 同时设置左右负压电机油门。
 * @param left_throttle 左电调油门，输入范围 0~10000。
 * @param right_throttle 右电调油门，输入范围 0~10000。
 */
void negative_pressure_set_throttle(uint16 left_throttle, uint16 right_throttle);

#ifdef __cplusplus
}
#endif

#endif
