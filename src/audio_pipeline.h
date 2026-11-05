#ifndef AUDIO_PIPELINE_H_
#define AUDIO_PIPELINE_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

/* Core Timing & Frame Macros */
#define SAMPLE_RATE            48000U
#define SAMPLE_BIT_WIDTH       16U
#define BYTES_PER_SAMPLE       sizeof(int16_t)
#define NUM_CHANNELS           2U
#define AUDIO_CHANNELS         2U
#define FRAMES_PER_BLOCK       256U  /* Power of 2 for FFT */
#define SAMPLES_PER_BLOCK      (FRAMES_PER_BLOCK * AUDIO_CHANNELS) /* 512 */
#define BLOCK_SIZE             PCM_BYTES_PER_BLOCK                 /* 1024 Bytes */

/* Input Buffer Geometry (I2S2 PDM RX Slab) */
#define PDM_DECIMATION_FACTOR  64U
#define PDM_WORDS_PER_FRAME    (PDM_DECIMATION_FACTOR / 16U) /* 4 */
#define PDM_WORDS_PER_BLOCK    (FRAMES_PER_BLOCK * PDM_WORDS_PER_FRAME) /* 1024 */
#define PDM_BYTES_PER_BLOCK    (PDM_WORDS_PER_BLOCK * sizeof(uint16_t)) /* 2048 Bytes */

/* Output Buffer Geometry (I2S3 PCM TX Slab) */
#define PCM_SAMPLES_PER_BLOCK  (FRAMES_PER_BLOCK * AUDIO_CHANNELS) /* 512 */
#define PCM_BYTES_PER_BLOCK    (PCM_SAMPLES_PER_BLOCK * sizeof(int16_t)) /* 1024 Bytes */
/* DSP STFT MACROS*/
#define STFT_WINDOW_SIZE 512U
#define STFT_HOP_SIZE 256U
#define FFT_BINS 256U
/* Wiener Spectral Gate Tuning Parameters */
#define WIENER_BETA         2.0f    /* Over-subtraction factor */
#define WIENER_GAIN_FLOOR   0.08f   /* -22 dB floor to prevent musical noise */
#define NOISE_ALPHA_DOWN    0.90f   /* Fast track downward when room quiets */
#define NOISE_ALPHA_UP      0.9995f /* Slow creep upward during speech */
#define EPSILON             1e-12f  /* Prevent divide-by-zero */

#define BLOCK_COUNT 8U

/* Memory Slabs */
extern struct k_mem_slab audio_mem_slab;
extern struct k_mem_slab pdm_rx_slab;
extern struct k_mem_slab pcm_tx_slab;

void audio_pipeline_start(const struct device *i2s_rx, const struct device *i2s_tx);
void process_mic_and_dsp(const uint16_t *pdm_rx_buf, int16_t *pcm_tx_buf);
void process_audio_block(int16_t *interleaved_dma_buf);
void dsp_filter_init(void);

extern const int16_t sine_500hz_table[96];


#endif /* AUDIO_PIPELINE_H_ */
