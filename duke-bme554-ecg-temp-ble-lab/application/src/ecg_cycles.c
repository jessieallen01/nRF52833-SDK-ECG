#include "ecg_cycles.h"

#include <zephyr/kernel.h>


#define THRESHOLD 400 // define threshold for crossing detection
int cycle_frequency(int16_t buff_diff[],int16_t *num_of_cycles)
{
    //when signal crosses from negative to positive, count as a cycle
    //*num_of_cycles = 0;

    for (int i = 1; i < BUFFER_ARRAY_LEN; i++) {
       if (buff_diff[i-1] < THRESHOLD && buff_diff[i] >= THRESHOLD) {
            (*num_of_cycles)++;
   
     }
// you need to add heart rate calculation based on num_of_cycles
//uint32_t bpm = (uint32_t)(ecg_frequency_hz * 60.0f);

    }
    return 0;       
}


