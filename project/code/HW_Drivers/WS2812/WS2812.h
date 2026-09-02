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
#ifndef WS2812_H
#define WS2812_H

#include "zf_common_headfile.h"

#ifndef WS2812_DIN_PIN
#define WS2812_DIN_PIN (P23_7)
#endif

void WS2812_Init(void);
void WS2812_SetRGB(uint8 red, uint8 green, uint8 blue);
void WS2812_SetColor(uint32 rgb);
void WS2812_Off(void);

#endif
