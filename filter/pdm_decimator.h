#ifndef PDM_DECIMATOR_H
#define PDM_DECIMATOR_H

#include <stdint.h>
#include <stddef.h>
#include <arm_math.h>
#include "audio_pipeline.h"

#define FIR_TAP_NUM 15U
#define PCM_BLOCK_FRAMES 256U

typedef struct {
    /* Integrator states for the CIC filter (unsigned for well-defined modulo 2^32 wrap-around) */
    uint32_t integ1;
    uint32_t integ2;
    uint32_t integ3;
    /* Comb states for the CIC filter */
    uint32_t comb1_z1;
    uint32_t comb2_z1;
    uint32_t comb3_z1;
    /* DC blocker states */
    float32_t dc_blocker_x1;
    float32_t dc_blocker_y1;
} pdm_decimator_t;

void pdm_decimator_init(pdm_decimator_t *decimator);
void pdm_decimate_to_float(pdm_decimator_t *decimator, const uint16_t *pdm_input, float32_t *mono_f32_output, size_t pcm_frames);

#endif /* PDM_DECIMATOR_H */
