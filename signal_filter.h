#ifndef SIGNAL_FILTER_H
#define SIGNAL_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>

#define FILTER_TAPS 141 // Liczba współczynników filtra
#define BLOCK_SIZE 32  // Liczba próbek na blok

// Deklaracja funkcji
void fir_filter_init(void);
void fir_filter_process(float32_t *input, float32_t *output, uint32_t block_size);

#endif // SIGNAL_FILTER_H
