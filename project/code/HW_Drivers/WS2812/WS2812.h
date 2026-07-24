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
