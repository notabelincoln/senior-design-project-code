// user_math.h
// Abraham Jordan
// Header file for basic data processing
#include "user_global.h"

// normalize sample data
inline float *normalize_sample_array(float *sample, float *calibrate, uint32_t length);

// return the greater between two floats
inline float float_max(float x1, float x2);

// compute the lesser of two floats
inline float float_min(float x1, float x2);
