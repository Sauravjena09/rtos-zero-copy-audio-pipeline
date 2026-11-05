#include "pdm_decimator.h"
#include <string.h>

/* Inverse-sinc^3 coefficients to flatten the CIC droop */
static const float32_t cic_comp_coeffs[FIR_TAP_NUM] = {
    -0.0018f,  0.0049f, -0.0116f,  0.0243f, -0.0479f,
     0.0926f, -0.2014f,  1.2818f, -0.2014f,  0.0926f,
    -0.0479f,  0.0243f, -0.0116f,  0.0049f, -0.0018f
};

__ccm_bss_section static float32_t fir_state[FIR_TAP_NUM + PCM_BLOCK_FRAMES - 1] __aligned(4);
static arm_fir_instance_f32 fir_inst;

void pdm_decimator_init(pdm_decimator_t *decimator)
{
    memset(decimator, 0, sizeof(pdm_decimator_t));
    arm_fir_init_f32(&fir_inst, FIR_TAP_NUM, (float32_t *)cic_comp_coeffs, fir_state, PCM_BLOCK_FRAMES);
}

void pdm_decimate_to_float(pdm_decimator_t *decimator, const uint16_t *pdm_input, float32_t *mono_f32_output, size_t pcm_frames)
{
    /* 1. Cache integrator states in ARM CPU core registers (unsigned modulo 2^32) */
    uint32_t i1 = decimator->integ1;
    uint32_t i2 = decimator->integ2;
    uint32_t i3 = decimator->integ3;

    /* 2. Cache comb delay line registers */
    uint32_t c1_z1 = decimator->comb1_z1;
    uint32_t c2_z1 = decimator->comb2_z1;
    uint32_t c3_z1 = decimator->comb3_z1;

    /* 3. Cache DC blocker states in CPU registers */
    float32_t dc_x1 = decimator->dc_blocker_x1;
    float32_t dc_y1 = decimator->dc_blocker_y1;

    for (size_t f = 0; f < pcm_frames; f++) {
        
        /* Integrator stages (High rate: 3.072 MHz, 64 bits per PCM sample) */
        for (int w = 0; w < PDM_WORDS_PER_FRAME; w++) {
            uint16_t pdm_word = pdm_input[f * PDM_WORDS_PER_FRAME + w];
            for (int b = 15; b >= 0; b--) {
                uint32_t bit_val = ((pdm_word >> b) & 1U) ? 1U : (uint32_t)-1;
                i1 += bit_val;
                i2 += i1;
                i3 += i2;
            }
        }

        /* Comb stages (Low rate: 48 kHz, shift delay registers each frame) */
        uint32_t c1 = i3 - c1_z1;
        c1_z1 = i3;

        uint32_t c2 = c1 - c2_z1;
        c2_z1 = c1;

        uint32_t c3 = c2 - c3_z1;
        c3_z1 = c2;

        /* Cast modulo 2^32 comb difference back to signed int32 */
        int32_t signed_out = (int32_t)c3;

        /* DC Blocker with Gain Normalization (R^3 = 64^3 = 262,144) */
        float32_t x0 = (float32_t)signed_out * (1.0f / 262144.0f);
        float32_t y0 = 0.995f * (x0 - dc_x1) + 0.995f * dc_y1;
        dc_x1 = x0;
        dc_y1 = y0;

        /* Digital Mic Preamp Boost: 2.5x (+8 dB) with soft saturation */
        float32_t boosted = y0 * 2.5f;
        if (boosted > 1.0f) {
            boosted = 1.0f;
        } else if (boosted < -1.0f) {
            boosted = -1.0f;
        }
        mono_f32_output[f] = boosted;
    }

    /* Save persistent states back to struct for next DMA block */
    decimator->integ1 = i1;
    decimator->integ2 = i2;
    decimator->integ3 = i3;

    decimator->comb1_z1 = c1_z1;
    decimator->comb2_z1 = c2_z1;
    decimator->comb3_z1 = c3_z1;

    decimator->dc_blocker_x1 = dc_x1;
    decimator->dc_blocker_y1 = dc_y1;

    /* 15-tap FIR compensation filter to flatten CIC sinc^3 droop */
    arm_fir_f32(&fir_inst, mono_f32_output, mono_f32_output, pcm_frames);
}
