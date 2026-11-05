#include "audio_pipeline.h"
#include <arm_math.h>
#include "pdm_decimator.h"
#include <string.h>
#include <math.h>

/* =========================================================================
 * PIPELINE PROCESSING MODE
 * Set ENABLE_STFT_DSP to 1 for full advanced real-time audio pipeline:
 *   CIC Decimator -> 512-pt WOLA STFT Wiener Gate -> Biquad Filters -> Studio Echo
 * Set to 0 for pristine low-latency direct mic loopback (verification mode):
 *   CIC Decimator -> DC Blocker -> Droop Compensator -> Direct DAC Loopback
 * ========================================================================= */
#define ENABLE_STFT_DSP 1

static pdm_decimator_t pdm_decimator;
static arm_rfft_fast_instance_f32 rfft_instance;

static arm_biquad_casd_df1_inst_f32 biquad_left;
static arm_biquad_casd_df1_inst_f32 biquad_right;

/* WOLA State Buffers */
__ccm_bss_section static float32_t stft_history[STFT_HOP_SIZE] __aligned(4);
__ccm_bss_section static float32_t stft_windowed[STFT_WINDOW_SIZE] __aligned(4);
__ccm_bss_section static float32_t stft_ola_buffer[STFT_HOP_SIZE] __aligned(4);
__ccm_bss_section static float32_t fft_bins[STFT_WINDOW_SIZE] __aligned(4);

/* Wiener Filter Noise Floor Tracker */
__ccm_bss_section static float32_t noise_floor_pwr[FFT_BINS] __aligned(4);
__ccm_bss_section static float32_t sqrt_hann_512[STFT_WINDOW_SIZE] __aligned(4);
__ccm_bss_section static float32_t mono_out[FRAMES_PER_BLOCK] __aligned(4);

__ccm_noinit_section static int16_t left_q15[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static int16_t right_q15[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static float32_t left_f32[FRAMES_PER_BLOCK] __aligned(4);
__ccm_noinit_section static float32_t right_f32[FRAMES_PER_BLOCK] __aligned(4);
__ccm_bss_section static float32_t echoline[4096] __aligned(4);
static uint32_t echo_idx = 0U;
__ccm_bss_section static float32_t biquad_state_l[4 * 2] __aligned(4);
__ccm_bss_section static float32_t biquad_state_r[4 * 2] __aligned(4);

/* Calibrated Biquad Cascade:
 * Stage 1: 150 Hz High-Pass (removes DC rumble and mic handling thump, exact 0 dB passband)
 * Stage 2: 3800 Hz Low-Pass (voice-band anti-aliasing filter, exact 0 dB passband) */
static const float32_t biquad_cofficient[10] = {
    /* Stage 1: HPF 150 Hz @ 48 kHz (b0, b1, b2, a1, a2) */
     0.9727f, -1.9454f,  0.9727f,  1.9722f, -0.9726f,
    /* Stage 2: LPF 3800 Hz @ 48 kHz (b0, b1, b2, a1, a2) */
     0.0453f,  0.0907f,  0.0453f,  1.3139f, -0.4953f
};

void dsp_filter_init(void)
{
    /* 1. Initialize CIC + DC Blocker + FIR Compensator */
    pdm_decimator_init(&pdm_decimator);

    /* 2. Initialize Left and Right Biquad Cascades */
    arm_biquad_cascade_df1_init_f32(&biquad_left, 2U, (float32_t *)biquad_cofficient, biquad_state_l);
    arm_biquad_cascade_df1_init_f32(&biquad_right, 2U, (float32_t *)biquad_cofficient, biquad_state_r);

    /* 3. Initialize 512-point Fast Real FFT */
    arm_rfft_fast_init_f32(&rfft_instance, STFT_WINDOW_SIZE);

    /* 4. Clear WOLA & Echo state buffers */
    memset(stft_history, 0, sizeof(stft_history));
    memset(stft_windowed, 0, sizeof(stft_windowed));
    memset(fft_bins, 0, sizeof(fft_bins));
    memset(stft_ola_buffer, 0, sizeof(stft_ola_buffer));
    memset(echoline, 0, sizeof(echoline));
    echo_idx = 0U;

    /* 5. Seed initial noise floor estimate */
    for (size_t k = 0; k < FFT_BINS; k++) {
        noise_floor_pwr[k] = 1e-5f;
    }

    /* 6. Periodic Square-Root Hann Window: sin(pi * n / N) */
    for (size_t n = 0; n < STFT_WINDOW_SIZE; n++) {
        sqrt_hann_512[n] = sinf((3.14159265358979323846f * (float32_t)n) / (float32_t)STFT_WINDOW_SIZE);
    }
}

void process_mic_and_dsp(const uint16_t *pdm_rx_buf, int16_t *pcm_tx_buf)
{
    /* Step 1: PDM Decimation (CIC + DC Blocker + Droop Compensator) */
    pdm_decimate_to_float(&pdm_decimator, pdm_rx_buf, mono_out, FRAMES_PER_BLOCK);

#if (!ENABLE_STFT_DSP)
    /* --- DIRECT CLEAN LOOPBACK (VERIFICATION MODE) ---
     * Bypasses STFT Wiener filter and echo so user hears pure raw microphone audio */
    arm_float_to_q15(mono_out, left_q15, FRAMES_PER_BLOCK);
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        pcm_tx_buf[2U * i]      = left_q15[i];
        pcm_tx_buf[2U * i + 1U] = left_q15[i];
    }
#else
    /* --- FULL ADVANCED DSP PIPELINE --- */
    /* 1. STFT Analysis with 50% Overlap */
    memcpy(&stft_windowed[0], stft_history, STFT_HOP_SIZE * sizeof(float32_t));
    memcpy(&stft_windowed[STFT_HOP_SIZE], mono_out, STFT_HOP_SIZE * sizeof(float32_t));
    memcpy(stft_history, mono_out, STFT_HOP_SIZE * sizeof(float32_t));

    arm_mult_f32(stft_windowed, sqrt_hann_512, stft_windowed, STFT_WINDOW_SIZE);
    arm_rfft_fast_f32(&rfft_instance, stft_windowed, fft_bins, 0);

    fft_bins[0] *= 0.25f;
    fft_bins[1] *= 0.25f;

    /* 2. Wiener Spectral Gate */
    for (size_t k = 1; k < FFT_BINS; k++) {
        float32_t re = fft_bins[2U * k];
        float32_t im = fft_bins[2U * k + 1U];
        float32_t pwr = (re * re) + (im * im);

        /* Asymmetric Noise Tracking: slow upward during speech, fast downward when quiet */
        if (pwr < noise_floor_pwr[k]) {
            noise_floor_pwr[k] = (0.90f * noise_floor_pwr[k]) + (0.10f * pwr);
        } else {
            noise_floor_pwr[k] = (0.999f * noise_floor_pwr[k]) + (0.001f * pwr);
        }

        /* Smooth continuous Wiener gain: G = P / (P + 1.2 * N) */
        float32_t noise = noise_floor_pwr[k];
        float32_t gain = pwr / (pwr + (1.2f * noise) + EPSILON);
        if (gain < 0.25f) {
            gain = 0.25f; /* Gentle -12 dB suppression floor — eliminates musical artifacts */
        }

        fft_bins[2U * k]      = re * gain;
        fft_bins[2U * k + 1U] = im * gain;
    }

    /* 3. Synthesis & Overlap-Add */
    arm_rfft_fast_f32(&rfft_instance, fft_bins, stft_windowed, 1);
    arm_mult_f32(stft_windowed, sqrt_hann_512, stft_windowed, STFT_WINDOW_SIZE);

    for (size_t i = 0; i < STFT_HOP_SIZE; i++) {
        mono_out[i]        = stft_windowed[i] + stft_ola_buffer[i];
        stft_ola_buffer[i] = stft_windowed[i + STFT_HOP_SIZE];
    }

    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        left_f32[i]  = mono_out[i];
        right_f32[i] = mono_out[i];
    }

    /* 4. Dual Biquad Filter (HPF 150 Hz + LPF 3800 Hz) */
    arm_biquad_cascade_df1_f32(&biquad_left, left_f32, left_f32, FRAMES_PER_BLOCK);
    arm_biquad_cascade_df1_f32(&biquad_right, right_f32, right_f32, FRAMES_PER_BLOCK);

    /* 5. Circular Feedforward Echo (balanced studio reflection, ~85.3 ms delay) */
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        float32_t delayed = echoline[echo_idx];
        echoline[echo_idx] = left_f32[i];
        echo_idx = (echo_idx + 1U) & 4095U;

        left_f32[i]  = (0.80f * left_f32[i])  + (0.20f * delayed);
        right_f32[i] = (0.80f * right_f32[i]) + (0.20f * delayed);
    }

    /* 6. Convert to Q15 PCM & Interleave */
    arm_float_to_q15(left_f32, left_q15, FRAMES_PER_BLOCK);
    arm_float_to_q15(right_f32, right_q15, FRAMES_PER_BLOCK);
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        pcm_tx_buf[2U * i]      = left_q15[i];
        pcm_tx_buf[2U * i + 1U] = right_q15[i];
    }
#endif
}
