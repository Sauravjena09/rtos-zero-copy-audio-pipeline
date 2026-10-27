#ifndef AUDIO_PIPELINE_H_
#define AUDIO_PIPELINE_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

#define SAMPLE_RATE         48000U
#define SAMPLE_BIT_WIDTH    16U
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUM_CHANNELS        2U
#define SAMPLES_PER_BLOCK   ((SAMPLE_RATE / 100) * NUM_CHANNELS)
#define BLOCK_SIZE          (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT         16U

extern struct k_mem_slab audio_mem_slab;
extern const int16_t sine_500hz_table[96];

void audio_pipeline_start(const struct device *i2s_rx, const struct device *i2s_tx);

#endif /* AUDIO_PIPELINE_H_ */
