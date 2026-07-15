#include "app_rgb.h"
#include "bsp_rgb.h"
#include "stdlib.h"
#include "app_bat.h"
#include "bsp_common.h"
#include "app_motion.h"
#include "stdio.h"
#include "config.h"   /* BATTERY_CELLS: the fuel gauge follows the pack, not the chassis */

// Number of waterfall LEDs
#define WATERFALL_MAX     4
#define RED_COUNT         3

rgb_effect_t g_now_effect = RGB_NO_EFFECT;   // RGB_NO_EFFECT
uint8_t g_speed_threshold = 5;

uint8_t gt_red = 0;
uint8_t gt_green = 0;
uint8_t gt_blue = 0;

uint8_t g_battery_count = 0; // Battery read count
int g_V_Z10 = 0; // Accumulated voltage buffer value

uint8_t g_rgb_speed = 0;         // Temporary increment for speed adjustment
uint8_t g_breath_color = 1;  // Default breathing light color value

// RGB light effect buffer values
uint32_t waterfall[MAX_RGB] = {0};
uint32_t marquee[MAX_RGB] = {0};
uint32_t starbright[MAX_RGB] = {0};

#define STAR_MAX_SHOW     5
#define STAR_INFO         3
#define STAR_ID           0      // ID number of the RGB LED
#define STAR_BL           1      // Max brightness of the RGB LED
#define STAR_SP           2      // RGB fade-out speed

uint8_t star[STAR_MAX_SHOW][STAR_INFO] = {0};

// Initialization, load some data
void app_rgb_init(void)
{
    for (int i = 0; i < MAX_RGB; i++)
    {
        waterfall[i] = 0x050000;
    }
    // Turn off the RGB strip at power-on
    RGB_Clear();
    RGB_Update();
}

// Regenerate a random number
static uint8_t rgb_remix(uint8_t val)
{
    static uint8_t last = 0;
    if (ABS(val - last) < val % 30)
    {
        val = (val + last) % 150;
    }
    last = val % 150;
    return last;
}


static void rgb_remix_u8(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (*r > 50 && *g > 50 && *b > 50)
    {
        uint8_t index = rand() % 3;
        if (index == 0) *r = 0;
        else if (index == 1) *g = 0;
        else if (index == 2) *b = 0;
    }
}


// Waterfall (running light) effect
static void app_rgb_waterfall(void)
{
    static uint8_t water_index = 0;
    static uint8_t water_color = 0;
    uint8_t r, g, b;
    if (water_index < MAX_RGB)
    {
        switch (water_color)
        {
        case 0:
            r = 150; g = 0; b = 0;
            break;
        case 1:
            r = 150; g = 65; b = 0;
            break;
        case 2:
            r = 150; g = 150; b = 0;
            break;
        case 3:
            r = 0; g = 150; b = 0;  
            break;
        case 4:
            r = 0; g = 150; b = 150; 
            break;
        case 5:
            r = 0; g = 0; b = 150;   
            break;
        case 6:
            r = 150; g = 0; b = 150;
            break;
        default:
            r = 0; g = 0; b = 0;
            break;
        }

        if (water_index < WATERFALL_MAX)
        {
            r = (uint8_t)(r - r * water_index / WATERFALL_MAX);
            g = (uint8_t)(g - g * water_index / WATERFALL_MAX);
            b = (uint8_t)(b - b * water_index / WATERFALL_MAX);
            waterfall[0] = g << 16 | r << 8 | b;
        }
        else
        {
            waterfall[0] = 0x050000;
        }
        for (int i = 0; i < MAX_RGB; i++)
        {
            RGB_Set_Color_U32(i, waterfall[i]);
            if (i < MAX_RGB-1)
                waterfall[MAX_RGB - i - 1] = waterfall[MAX_RGB - i - 2];
        }
        water_index++;
    }
    else
    {
        water_index = 0;
        water_color = (water_color + 1) % 7;
    }
    RGB_Update();
}

// Gradient light effect.
static void app_rbg_gradient(void)
{
    static uint8_t grad_color = 0;
    static uint8_t grad_index = 0;

    if (grad_color % 2 == 0)
    {
        gt_red = rand() % 256;
        gt_green = rand() % 256;
        gt_blue = rand() % 256;

        gt_green = rgb_remix(gt_green);
        rgb_remix_u8(&gt_red, &gt_green, &gt_blue);
        grad_color++;
    }
    if (grad_color == 1)
    {
        if (grad_index < MAX_RGB)
        {
            RGB_Set_Color(grad_index, gt_red, gt_green, gt_blue);
            grad_index++;
        }
        if (grad_index >= MAX_RGB)
        {
            grad_color = 2;
            grad_index = 0;
        }
    }
    else if (grad_color == 3)
    {
        if (grad_index < MAX_RGB)
        {
            RGB_Set_Color(MAX_RGB - 1 - grad_index, gt_red, gt_green, gt_blue);
            grad_index++;
        }
        if (grad_index >= MAX_RGB)
        {
            grad_color = 0;
            grad_index = 0;
        }
    }
    RGB_Update();
}

// Breathing light effect
static void app_rgb_breathing(uint8_t breath_color)
{
    static uint8_t breath_direction = 0;
    static int breath_count = 0;

    if (breath_direction == 0)
    {
        switch (breath_color)
        {
        case 0: // Red
            RGB_Set_Color(255, breath_count, 0, 0);
            break;
        case 1: // Green
            RGB_Set_Color(255, 0, breath_count, 0);
            break;
        case 2: // Blue
            RGB_Set_Color(255, 0, 0, breath_count);
            break;
        case 3: // Yellow
            RGB_Set_Color(255, breath_count, breath_count, 0);
            break;
        case 4: // Purple
            RGB_Set_Color(255, breath_count, 0, breath_count);
            break;
        case 5: // Cyan
            RGB_Set_Color(255, 0, breath_count, breath_count);
            break;
        case 6: // White
            RGB_Set_Color(255, breath_count, breath_count, breath_count);
            break;
        default:
            break;
        }
        breath_count += 2;
        if (breath_count >= 150)
        {
            breath_direction = 1;
        }
    }
    else
    {
        switch (breath_color)
        {
        case 0:
            RGB_Set_Color(255, breath_count, 0, 0);
            break;
        case 1:
            RGB_Set_Color(255, 0, breath_count, 0);
            break;
        case 2:
            RGB_Set_Color(255, 0, 0, breath_count);
            break;
        case 3:
            RGB_Set_Color(255, breath_count, breath_count, 0);
            break;
        case 4:
            RGB_Set_Color(255, breath_count, 0, breath_count);
            break;
        case 5:
            RGB_Set_Color(255, 0, breath_count, breath_count);
            break;
        case 6:
            RGB_Set_Color(255, breath_count, breath_count, breath_count);
            break;
        default:
            break;
        }
        breath_count -= 2;
        if (breath_count < 0)
        {
            breath_direction = 0;
            breath_count = 0;
        }
    }
    RGB_Update();
}

// Marquee light effect
static void app_rgb_marquee(void)
{
    uint8_t r = rgb_remix(rand() % 150);
    uint8_t g = rgb_remix(rand() % 150);
    uint8_t b = rgb_remix(rand() % 150);

    marquee[0] = g << 16 | r << 8 | b;
    for (int i = 0; i < MAX_RGB; i++)
    {
        RGB_Set_Color_U32(i, marquee[i]);
        if (i < MAX_RGB-1)
            marquee[MAX_RGB - i - 1] = marquee[MAX_RGB - i - 2];
    }
    RGB_Update();
}

// Twinkling starlight effect
static void app_rgb_starbright(uint8_t speed)
{
    static uint8_t led_count = 0;
    u16 t_speed = speed * 10;
    if (rand() % t_speed <= 2)
    {
        if (star[led_count][STAR_BL] != 0)
        {
            led_count = (led_count + 1) % STAR_MAX_SHOW;
            if (star[led_count][STAR_BL] != 0)
                return;
        }
        star[led_count][STAR_ID] = rand() % MAX_RGB;
        star[led_count][STAR_BL] = rand() % 100 + 50;
        star[led_count][STAR_SP] = star[led_count][STAR_BL] / 50 + 3;
        // star[led_count][STAR_SP] = (rand() % 3) + 1;
        led_count = (led_count + 1) % STAR_MAX_SHOW;
    }
    s16 temp = 0;
    for (uint8_t i = 0; i < STAR_MAX_SHOW; i++)
    {
        temp = star[i][STAR_BL] - star[i][STAR_SP];
        if (temp <= 0) temp = 0;
        star[i][STAR_BL] = temp;
        RGB_Set_Color(star[i][STAR_ID], star[i][STAR_BL], star[i][STAR_BL], star[i][STAR_BL]);
    }
    RGB_Update();
}

static void rgb_battery_126V(int V_Z10)
{
    if (V_Z10 > 130) // Above the safe voltage: show a red warning
    {
        RGB_Set_Color(RGB_CTRL_ALL, 255, 0, 0);
    }
    else if (V_Z10 >= 100) // Battery level normal
    {
        // res = (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
        uint8_t count = (V_Z10-100)*(MAX_RGB-(RED_COUNT+1))/(24)+(RED_COUNT+1);
        if (count > MAX_RGB) count = MAX_RGB;
        for (uint8_t i = 0; i < count; i++)
        {
            RGB_Set_Color(i, 0, (17-MAX_RGB+count)*10, 0);
        }
    }
    else if (V_Z10 > 95) // Battery level low or fairly low
    {
        uint8_t count = 1;
        if (V_Z10 > 98) count = RED_COUNT;
        else if (V_Z10 > 97) count = RED_COUNT-1;
        for (uint8_t i = 0; i < count; i++)
        {
            RGB_Set_Color(i, 150, 0, 0);
        }
    }
    else // Battery level too low
    {
        RGB_Set_Color(0, 10, 0, 0);
    }
}

static void rgb_battery_84V(int V_Z10)
{
    // printf("battery:%d\n", V_Z10);
    if (V_Z10 > 85) // Above the safe voltage: show a red warning
    {
        RGB_Set_Color(RGB_CTRL_ALL, 255, 0, 0);
    }
    else if (V_Z10 >= 70) // Battery level normal
    {
        // res = (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
        uint8_t count = (V_Z10-70)*(MAX_RGB-(RED_COUNT+1))/(14)+(RED_COUNT+1);
        if (count > MAX_RGB) count = MAX_RGB;
        for (uint8_t i = 0; i < count; i++)
        {
            RGB_Set_Color(i, 0, (17-MAX_RGB+count)*10, 0);
        }
    }
    else if (V_Z10 > 65) // Battery level low or fairly low
    {
        uint8_t count = 1;
        if (V_Z10 > 68) count = RED_COUNT;
        else if (V_Z10 > 67) count = RED_COUNT-1;
        for (uint8_t i = 0; i < count; i++)
        {
            RGB_Set_Color(i, 150, 0, 0);
        }
    }
    else // Battery level too low
    {
        RGB_Set_Color(0, 10, 0, 0);
    }
}

// Display the battery level
static void app_rgb_battery(void)
{
    int V_Z10 = Bat_Voltage_Z10(); // Depends on the system voltage detection
    g_battery_count++;
    g_V_Z10 += V_Z10;
    if (g_battery_count < 8) return; // Not enough counts yet, return automatically
    g_battery_count = 0;
    V_Z10 = g_V_Z10 >> 3;
    g_V_Z10 = 0;
    RGB_Clear();
    // The gauge scale is a property of the PACK, not of the chassis. Keyed on the chassis
    // type, a 2S pack on a mecanum chassis was measured against the 12.6 V scale and so
    // showed a permanent "empty" (one dim red LED) at full charge. The CAR_SUNRISE arm is
    // kept because that chassis also carries an 8.4 V pack -- both conditions mean the same
    // thing, "this is an 8.4 V pack".
    if (BATTERY_CELLS == 2 || Motion_Get_Car_Type() == CAR_SUNRISE)
    {
        rgb_battery_84V(V_Z10);
    }
    else
    {
        rgb_battery_126V(V_Z10);
    }
    RGB_Update();
}

// RGB light effect driver function, called every 10 milliseconds.
void app_rgb_effects_handle(void)
{
    switch (g_now_effect)
    {
    case RGB_NO_EFFECT:
        break;

    case RGB_WATERFALL:
        if (g_rgb_speed >= g_speed_threshold)
        {
            app_rgb_waterfall();
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    case RGB_MARQUEE:
        if (g_rgb_speed >= g_speed_threshold)
        {
            app_rgb_marquee();
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    case RGB_BREATHING:
        if (g_rgb_speed >= g_speed_threshold)
        {
            app_rgb_breathing(g_breath_color);
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    case RGB_STARBRIGHT:
        if (g_rgb_speed >= g_speed_threshold)
        {
            app_rgb_starbright(g_speed_threshold);
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    case RGB_GRADIENT:
        if (g_rgb_speed >= g_speed_threshold)
        {
            app_rbg_gradient();
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    case RGB_BATTERY:
        if (g_rgb_speed >= 5)
        {
            app_rgb_battery();
            g_rgb_speed = 0;
        }
        g_rgb_speed++;
        break;
    
    case RGB_MAX_EFFECT:
        break;
    default:
        break;
    }
}


// Set the color of the breathing light effect, color=[0, 6]
void app_rgb_set_breathing_color(uint8_t color)
{
    if (color < 7)
        g_breath_color = color;
}


// Set the RGB strip effect, pass in the effect
// effect=[0, RGB_MAX_EFFECT-1]
// speed=[1, 10], the smaller the speed value, the faster
void app_rgb_set_effect(uint8_t effect, uint8_t speed)
{
    if (effect < RGB_MAX_EFFECT)
    {
        g_now_effect = (rgb_effect_t)effect;
    }
    if (speed >= 1 && speed <= 10) g_speed_threshold = speed;
}

// Get the currently running effect
rgb_effect_t app_rgb_get_effect(void)
{
    return g_now_effect;
}

// Get the current speed threshold
uint8_t app_rgb_get_speed(void)
{
    return g_speed_threshold;
}

