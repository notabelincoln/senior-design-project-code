// user_math.c
// Abraham Jordan
// C file for basic data processing
#include "user_math.h"

// normalize sample data
int normalize_sample(float *sample, float *calibrate, float *normal)
{
	// counter, min value, max value
	unsigned int i;
	float min;
	float max;

	// first assign the deltas into normal
	for (i = 0; i < SENSOR_DATA_LENGTH; i++)
		normal[i] = calibrate[i] - sample[i];

	// set min and max value to first element of normalized array
	min = max = normal[0];

	// gather the minimum and maximum values, store the value delta into ret_array
	for (i = 0; i < SENSOR_DATA_LENGTH - 1; i++) {
		max = (max >= normal[i+1]) ? max : normal[i+1];
		min = (min < normal[i+1]) ? min : normal[i+1];
	}

	// normalize the values in delta_array
	for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
		normal[i] = (normal[i] - min) / (max - min);
	}

	return 0;
}
