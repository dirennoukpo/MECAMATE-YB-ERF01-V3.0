#include "app_math.h"


const float RtA = 57.2957795f;   //constant for converting radian to angle (degree)
const float AtR = 0.0174532925f; //constant for converting angle (degree) to radian



// Value input limit, int type
int Math_Limit_int(int input, int val_min, int val_max)
{
    if (input < val_min) return val_min;
    if (input > val_max) return val_max;
    return input;
}


// Value input limit, float type
float Math_Limit_float(float input, float val_min, float val_max)
{
    if (input < val_min) return val_min;
    if (input > val_max) return val_max;
    return input;
}

// Map a value, integer type.
int Math_Map(int x, int in_min, int in_max, int out_min, int out_max)
{
    int res = (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
    if (res < out_min) res = out_min;
    else if (res > out_max) res = out_max;
    return res;
}
