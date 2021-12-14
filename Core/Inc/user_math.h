// user_math.h
// Abraham Jordan
// Header file for basic data processing

#ifndef USER_MATH_H
#define USER_MATH_H
#include "user_global.h"

// normalize sample data
int normalize_sample(float *sample, float *calibrate, float *normal);
int decToBin_oneDigit(int dec, int binary[8]);

#endif
