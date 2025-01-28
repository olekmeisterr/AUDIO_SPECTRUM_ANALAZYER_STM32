#include "signal_filter.h"
#include <arm_math.h>

#define FILTER_TAPS 32 // Liczba wspolczynnikow filtra


// Wspolczynniki filtru (Beda zaprpojektowane w Matlabie
static const float32_t filter_coeffs[FILTER_TAPS] = {

};

// Bufor filtru
static float32_t filter_state[FILTER_TAPS];


