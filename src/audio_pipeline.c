#include "audio_pipeline.h"
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(audio_pipeline, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(audio_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

K_MSGQ_DEFINE(dsp_input_q, sizeof(void *), 4, 4);
K_MSGQ_DEFINE(dsp_output_q, sizeof(void *), 4, 4);

/* Precomputed 96-sample pure 500 Hz sine wave at 48 kHz (amplitude 8000) */
const int16_t sine_500hz_table[96] = {
       0,  523, 1044, 1560, 2070, 2571, 3061, 3538, 3999, 4444,
    4870, 5274, 5656, 6014, 6346, 6651, 6928, 7174, 7391, 7575,
    7727, 7846, 7931, 7982, 8000, 7982, 7931, 7846, 7727, 7575,
    7391, 7174, 6928, 6651, 6346, 6014, 5656, 5274, 4870, 4444,
    3999, 3538, 3061, 2571, 2070, 1560, 1044,  523,    0, -523,
   -1044,-1560,-2070,-2571,-3061,-3538,-3999,-4444,-4870,-5274,
   -5656,-6014,-6346,-6651,-6928,-7174,-7391,-7575,-7727,-7846,
   -7931,-7982,-8000,-7982,-7931,-7846,-7727,-7575,-7391,-7174,
   -6928,-6651,-6346,-6014,-5656,-5274,-4870,-4444,-4000,-3538,
   -3061,-2571,-2070,-1560,-1044, -523
};


static const struct device *dev_rx;
static const struct device *dev_tx;
static K_SEM_DEFINE(pipeline_start_sem, 0, 1);

static void audio_io_thread(void *arg1, void *arg2, void *arg3)
{
    void *rx_ptr;
    void *tx_ptr;
    size_t rx_size;
    int ret;

    /* Wait until audio_pipeline_start has configured devices and primed the pipeline */
    k_sem_take(&pipeline_start_sem, K_FOREVER);
    LOG_INF("Audio I/O thread active");

    while (1) {
        /* Retrieve processed audio block from DSP worker (paced by DSP queue) */
        if (k_msgq_get(&dsp_output_q, &tx_ptr, K_FOREVER) == 0) {
            ret = i2s_write(dev_tx, tx_ptr, BLOCK_SIZE);
            if (ret < 0) {
                k_mem_slab_free(&audio_mem_slab, tx_ptr);
            }
        }

        /* Check if an RX block has been captured by DMA (non-blocking) */
        ret = i2s_read(dev_rx, &rx_ptr, &rx_size);
        if (ret == 0) {
            if (k_msgq_put(&dsp_input_q, &rx_ptr, K_NO_WAIT) != 0) {
                k_mem_slab_free(&audio_mem_slab, rx_ptr);
            }
        }
    }
}

static void dsp_worker_thread(void *arg1, void *arg2, void *arg3)
{
    void *work_ptr;
    uint32_t phase = 0;

    LOG_INF("DSP worker thread active");

    while (1) {
       // bool tone_mode = false;

        /* If an RX block arrived from Thread 1, take it; otherwise allocate a buffer for test tone */
        if (k_msgq_get(&dsp_input_q, &work_ptr, K_NO_WAIT) != 0) {
          //  tone_mode = true;
            if (k_mem_slab_alloc(&audio_mem_slab, &work_ptr, K_MSEC(20)) != 0) {
                k_msleep(5);
                continue;
            }
        }
        process_audio_block(work_ptr);

        // int16_t *samples = (int16_t *)work_ptr;

        /*   if (tone_mode) {
            /* Fill block with smooth, continuous 500 Hz sine wave 
            for (int i = 0; i < SAMPLES_PER_BLOCK; i += 2) {
                int16_t val = sine_500hz_table[(phase + (i / 2)) % 96];
                samples[i]     = val; /* Left Channel
                samples[i + 1] = val; /* Right Channel 
            }
            phase = (phase + (SAMPLES_PER_BLOCK / 2)) % 96;
        }    */

        /* Push to output queue (K_FOREVER blocks when queue is full, naturally pacing to DAC rate) */
        if (k_msgq_put(&dsp_output_q, &work_ptr, K_FOREVER) != 0) {
            k_mem_slab_free(&audio_mem_slab, work_ptr);
        }
    }
}

K_THREAD_DEFINE(audio_io_tid, 1024, audio_io_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);
K_THREAD_DEFINE(dsp_work_tid, 2048, dsp_worker_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

/* PUBLIC INIT */
void audio_pipeline_start(const struct device *i2s_rx, const struct device *i2s_tx)
{
    dev_rx = i2s_rx;
    dev_tx = i2s_tx;
    for (int i = 0; i < 2; i++) {
        void *reserve_ptr;
        if (k_mem_slab_alloc(&audio_mem_slab, &reserve_ptr, K_NO_WAIT) == 0) {
            int16_t *samples = (int16_t *)reserve_ptr;
            for (int s = 0; s < SAMPLES_PER_BLOCK; s += 2) {
                int16_t val = sine_500hz_table[(s / 2) % 96];
                samples[s]     = val;
                samples[s + 1] = val;
            }
            k_msgq_put(&dsp_output_q, &reserve_ptr, K_NO_WAIT);
        }
    }
    LOG_INF("Two-thread audio pipeline queues primed with sine wave");
    k_sem_give(&pipeline_start_sem);
}
