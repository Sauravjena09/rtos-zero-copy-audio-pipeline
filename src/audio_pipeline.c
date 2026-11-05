#include "audio_pipeline.h"
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(audio_pipeline, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(pdm_rx_slab, PDM_BYTES_PER_BLOCK, BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE(pcm_tx_slab, PCM_BYTES_PER_BLOCK, BLOCK_COUNT, 4);

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
static K_SEM_DEFINE(pipeline_start_sem, 0, 3);
static uint32_t mic_rx_count = 0;
static uint32_t dsp_tx_count = 0;

/* THREAD 1: Dedicated Microphone PDM RX Thread.
 * Completely independent of TX — runs whenever I2S2 DMA delivers a 2048-byte block.
 * Never blocked by TX backpressure, so pdm_rx_slab NEVER exhausts. */
static void audio_rx_thread(void *arg1, void *arg2, void *arg3)
{
    void *rx_ptr;
    size_t rx_size;
    int ret;

    k_sem_take(&pipeline_start_sem, K_FOREVER);
    LOG_INF("Audio RX thread active (mic DMA)");

    while (1) {
        ret = i2s_read(dev_rx, &rx_ptr, &rx_size);
        if (ret == 0) {
            mic_rx_count++;
            if (mic_rx_count <= 5) {
                LOG_INF("Mic RX block #%u captured (%u bytes)", mic_rx_count, (uint32_t)rx_size);
            }
            if (k_msgq_put(&dsp_input_q, &rx_ptr, K_NO_WAIT) != 0) {
                /* Queue full — discard oldest buffer back to slab so DMA never stalls */
                k_mem_slab_free(&pdm_rx_slab, rx_ptr);
            }
        } else if (ret == -EIO) {
            LOG_WRN("RX DMA stream error — re-triggering");
            i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
            i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_START);
            k_msleep(2);
        }
    }
}

/* THREAD 2: Dedicated DAC PCM TX Thread.
 * Pulls processed audio blocks from dsp_output_q and feeds I2S3 TX DMA. */
static void audio_tx_thread(void *arg1, void *arg2, void *arg3)
{
    void *tx_ptr;
    int ret;

    k_sem_take(&pipeline_start_sem, K_FOREVER);
    LOG_INF("Audio TX thread active (DAC DMA)");

    while (1) {
        if (k_msgq_get(&dsp_output_q, &tx_ptr, K_FOREVER) == 0) {
            ret = i2s_write(dev_tx, tx_ptr, PCM_BYTES_PER_BLOCK);
            if (ret == -EIO) {
                LOG_WRN("TX DMA stream underrun — re-triggering");
                i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
                /* Put buffer into queue FIRST so DMA has data when START is triggered */
                int w_ret = i2s_write(dev_tx, tx_ptr, PCM_BYTES_PER_BLOCK);
                int t_ret = i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
                if (w_ret == 0 && t_ret == 0) {
                    dsp_tx_count++;
                } else {
                    k_mem_slab_free(&pcm_tx_slab, tx_ptr);
                }
            } else if (ret < 0) {
                /* Queue temporarily full (-EAGAIN / -EBUSY): free block without resetting */
                k_mem_slab_free(&pcm_tx_slab, tx_ptr);
            } else {
                dsp_tx_count++;
                if (dsp_tx_count <= 5) {
                    LOG_INF("DAC TX block #%u played", dsp_tx_count);
                }
            }
        }

        /* Periodic health report every ~200 blocks (~1.06 seconds) */
        if ((dsp_tx_count > 0) && (dsp_tx_count % 200 == 0)) {
            LOG_INF("Audio running: mic RX blocks=%u, DAC TX blocks=%u",
                    mic_rx_count, dsp_tx_count);
        }
    }
}

/* THREAD 3: DSP Worker Thread.
 * Processes real microphone blocks in lockstep with RX DMA. */
static void dsp_worker_thread(void *arg1, void *arg2, void *arg3)
{
    void *pdm_rx_ptr;
    void *pcm_tx_ptr;

    k_sem_take(&pipeline_start_sem, K_FOREVER);
    LOG_INF("DSP worker thread active");

    while (1) {
        /* Wait for captured PDM block from mic RX thread — strictly 1:1, no phantom silence blocks */
        if (k_msgq_get(&dsp_input_q, &pdm_rx_ptr, K_FOREVER) == 0) {
            if (k_mem_slab_alloc(&pcm_tx_slab, &pcm_tx_ptr, K_MSEC(10)) == 0) {
                process_mic_and_dsp((const uint16_t *)pdm_rx_ptr, (int16_t *)pcm_tx_ptr);
                k_mem_slab_free(&pdm_rx_slab, pdm_rx_ptr);

                if (k_msgq_put(&dsp_output_q, &pcm_tx_ptr, K_MSEC(10)) != 0) {
                    k_mem_slab_free(&pcm_tx_slab, pcm_tx_ptr);
                }
            } else {
                k_mem_slab_free(&pdm_rx_slab, pdm_rx_ptr);
            }
        }
    }
}

/* Thread definitions:
 * audio_tx_tid: Priority 1 — highest priority to guarantee DAC DMA never starves
 * audio_rx_tid: Priority 2 — high priority to immediately drain mic DMA buffers
 * dsp_work_tid: Priority 3 — runs heavy STFT / Wiener math with 8 KB stack */
K_THREAD_DEFINE(audio_tx_tid, 2048, audio_tx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);
K_THREAD_DEFINE(audio_rx_tid, 2048, audio_rx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, 0);
K_THREAD_DEFINE(dsp_work_tid, 8192, dsp_worker_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(3), 0, 0);

/* PUBLIC INIT */
void audio_pipeline_start(const struct device *i2s_rx, const struct device *i2s_tx)
{
    dev_rx = i2s_rx;
    dev_tx = i2s_tx;

    /* Pre-prime 2 blocks into dsp_output_q with smooth 500 Hz tone */
    for (int i = 0; i < 2; i++) {
        void *reserve_ptr;
        if (k_mem_slab_alloc(&pcm_tx_slab, &reserve_ptr, K_NO_WAIT) == 0) {
            int16_t *samples = (int16_t *)reserve_ptr;
            for (int s = 0; s < SAMPLES_PER_BLOCK; s += 2) {
                int16_t val = sine_500hz_table[(s / 2) % 96];
                samples[s]     = val;
                samples[s + 1] = val;
            }
            k_msgq_put(&dsp_output_q, &reserve_ptr, K_NO_WAIT);
        }
    }
    LOG_INF("Audio pipeline threads armed — 2 blocks primed in dsp_output_q");
    k_sem_give(&pipeline_start_sem);
    k_sem_give(&pipeline_start_sem);
    k_sem_give(&pipeline_start_sem);
}
