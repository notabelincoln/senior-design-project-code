// user_math.c
// Abraham Jordan
// C file for basic data processing
#include "user_math.h"
#include <stdint.h>

// normalize sample data
inline float *normalize_sample_array(float *sample, float *calibrate, float *normal_array)
{
	// counter, min value, max value
	uint32_t i;
	float min;
	float max;

	// gather the minimum and maximum values, store the value delta into ret_array
	for (i = 0; i < SENSOR_DATA_LENGTH - 1; i++) {
		min = float_min(calibrate[i] - sample[i], calibrate[i+1] - sample[i+1]);
		max = float_max(calibrate[i] - sample[i], calibrate[i+1] - sample[i+1]);

	}

	// normalize the values in delta_array
	for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
		normal_array[i] = (calibrate[i] - sample[i]) / (max - min);
	}

	return *normal_array;
}

// return the greater between two floats
inline float float_max(float x1, float x2)
{
	return (x1 >= x2) ? x1 : x2;
}

// compute the lesser of two floats
inline float float_min(float x1, float x2)
{
	return (x1 < x2) ? x1 : x2;
}
