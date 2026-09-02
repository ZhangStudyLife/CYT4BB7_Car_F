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
#ifndef BEEP_H
#define BEEP_H

#include "zf_common_headfile.h"

/* 主板蜂鸣器引脚（有源蜂鸣器） */
#ifndef BUZZER_PIN
#define BUZZER_PIN (P19_4)
#endif

/* 初始化蜂鸣器（默认静音） */
void Beep_Init(void);

/* 100Hz周期调用：非阻塞刷新蜂鸣器输出 */
void Beep_Update_100HZ(void);

/* 立即停止鸣叫并清空播放状态 */
void Beep_Stop(void);

/* 使能蜂鸣器（持续响） */
void Beep_Enable(void);

/* 失能蜂鸣器（立即停） */
void Beep_Disable(void);

/*
 * 触发鸣叫
 * duty_percent : 占空比(0~100)
 * cycle_time_s : 单周期时长(秒)
 * cycle_count  : 循环次数
 */
void Beep_Play(uint8 duty_percent, float cycle_time_s, uint16 cycle_count);

#endif /* BEEP_H */
