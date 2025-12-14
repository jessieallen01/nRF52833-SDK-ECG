#ifndef ECG_CYCLES_H
#define ECG_CYCLES_H

#define BUFFER_ARRAY_LEN 800

#include <zephyr/kernel.h>


int cycle_frequency(int16_t buff_diff[],int16_t *num_of_cycles);

#endif
