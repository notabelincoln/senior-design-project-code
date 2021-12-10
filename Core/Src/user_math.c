// user_math.c
// Abraham Jordan
// C file for basic data processing
#include "user_math.h"

// normalize sample data
inline float *normalize_sample_array(float *sample, float *calibrate)
{
	// counter, min value, max value
	uint32_t i;
	float min;
	float max;
	float ret_array[SENSOR_DATA_LENGTH];

	// gather the minimum and maximum values, store the value delta into ret_array
	for (i = 0; i < SENSOR_DATA_LENGTH - 1; i++) {
		min = float_min(calibrate[i] - sample[i], calibrate[i+1] - sample[i+1]);
		max = float_max(calibrate[i] - sample[i], calibrate[i+1] - sample[i+1]);
		ret_array[i] = calibrate[i] - sample[i];
	}

	// normalize the values in delta_array
	for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
		ret_array[i] = ret_array[i] / (max - min);
	}

	return ret_array;
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
