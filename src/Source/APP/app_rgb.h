#ifndef __APP_RGB_H__
#define __APP_RGB_H__

#include "stdint.h"



typedef enum _rgb_effect_t
{
    RGB_NO_EFFECT = 0,

    RGB_WATERFALL,  // Waterfall light
    RGB_MARQUEE,    // Marquee light
    RGB_BREATHING,  // Breathing light
    RGB_GRADIENT,   // Gradient light
    RGB_STARBRIGHT, // Twinkling starlight
    RGB_BATTERY,    // Battery level display

    RGB_MAX_EFFECT
} rgb_effect_t;


void app_rgb_init(void);
void app_rgb_effects_handle(void);

void app_rgb_set_breathing_color(uint8_t color);
void app_rgb_set_effect(uint8_t effect, uint8_t speed);


uint8_t app_rgb_get_speed(void);
rgb_effect_t app_rgb_get_effect(void);


#endif /* __APP_RGB_H__ */
