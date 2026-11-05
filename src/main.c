#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "audio_pipeline.h"

LOG_MODULE_REGISTER(main_boot, LOG_LEVEL_INF);

#define CS43L22_I2C_ADDR  0x4AU

static const struct gpio_dt_spec dac_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(cs43l22), reset_gpios);

static int cs43l22_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    int ret = i2c_write(i2c_dev, data, sizeof(data), CS43L22_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("CS43L22 write reg 0x%02X failed: %d", reg_addr, ret);
    }
    return ret;
}

static int cs43l22_init(const struct device *i2c_dev)
{
    int ret = 0;
    gpio_pin_configure_dt(&dac_reset, GPIO_OUTPUT_INACTIVE);
    k_msleep(5);
    gpio_pin_set_dt(&dac_reset, 1);
    k_msleep(10);

    /* Step 1: Power down before configuration */
    ret |= cs43l22_write_reg(i2c_dev, 0x02, 0x01);

    /* Step 2: Required "magic" init sequence (from CS43L22 datasheet errata / ST BSP) */
    ret |= cs43l22_write_reg(i2c_dev, 0x00, 0x99);
    ret |= cs43l22_write_reg(i2c_dev, 0x47, 0x80);
    ret |= cs43l22_write_reg(i2c_dev, 0x32, 0x80);
    ret |= cs43l22_write_reg(i2c_dev, 0x32, 0x00);
    ret |= cs43l22_write_reg(i2c_dev, 0x00, 0x00);

    /* Step 3: Power Control 2: Headphone channels always ON, speaker always OFF */
    ret |= cs43l22_write_reg(i2c_dev, 0x04, 0xAF);

    /* Step 4: Clocking Control: Auto-detect MCLK, MCLK divided by 2 (12.288 MHz -> 6.144 MHz) */
    ret |= cs43l22_write_reg(i2c_dev, 0x05, 0x81);

    /* Step 5: Interface Control 1: I2S format, 16-bit word length */
    ret |= cs43l22_write_reg(i2c_dev, 0x06, 0x07);

    /* Step 6: Analog ZC and Soft Ramp (required for clean transitions, no pops) */
    ret |= cs43l22_write_reg(i2c_dev, 0x0A, 0x00);
    ret |= cs43l22_write_reg(i2c_dev, 0x0E, 0x04);

    /* Step 7: Limiter/Noise Gate — disable */
    ret |= cs43l22_write_reg(i2c_dev, 0x27, 0x00);

    /* Step 8: PCM channel volume: 0x00 = 0 dB (digital path, before DAC) */
    ret |= cs43l22_write_reg(i2c_dev, 0x1A, 0x00); /* PCM A */
    ret |= cs43l22_write_reg(i2c_dev, 0x1B, 0x00); /* PCM B */

    /* Step 9: Master Volume A/B: 0x00 = 0 dB
     * IMPORTANT: 0x19..0xFF = negative / muted. 0x00 = 0 dB, 0x18 = +12 dB.
     * Previous value 0xF4 was ~-103 dB (essentially muted — silent output!) */
    ret |= cs43l22_write_reg(i2c_dev, 0x20, 0x00); /* Master A: 0 dB */
    ret |= cs43l22_write_reg(i2c_dev, 0x21, 0x00); /* Master B: 0 dB */

    /* Step 10: Power Control 1: Power Up */
    ret |= cs43l22_write_reg(i2c_dev, 0x02, 0x9E);

    if (ret == 0) {
        LOG_INF("CS43L22 DAC initialized and powered up successfully!");
    } else {
        LOG_ERR("CS43L22 DAC initialization encountered errors!");
    }
    return ret;
}

int main(void)
{
    /* If USB CDC-ACM console is used, wait briefly for host terminal DTR */
    const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(console_dev)) {
        uint32_t dtr = 0;
        for (int i = 0; i < 25 && !dtr; i++) {
            uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
            k_msleep(100);
        }
    }

    LOG_INF("=== STM32F4 Audio Pipeline Boot Sequence ===");

    const struct device *i2c_dev    = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    const struct device *i2s_tx_dev = DEVICE_DT_GET(DT_NODELABEL(i2s3));
    const struct device *i2s_rx_dev = DEVICE_DT_GET(DT_NODELABEL(i2s2));

    if (!device_is_ready(i2c_dev) || !device_is_ready(i2s_tx_dev) || !device_is_ready(i2s_rx_dev)) {
        LOG_ERR("Audio peripherals not ready!");
        return -EIO;
    }
    LOG_INF("Audio peripherals ready: I2C1, I2S3 (TX), I2S2 (RX)");

    struct i2s_config tx_cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_RATE,
        .mem_slab       = &pcm_tx_slab,
        .block_size     = PCM_BYTES_PER_BLOCK,
        .timeout        = 200,
    };

    struct i2s_config rx_cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_BIT_CLK_INV,
        .options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = 96000U,
        .mem_slab       = &pdm_rx_slab,
        .block_size     = PDM_BYTES_PER_BLOCK,
        .timeout        = 200,
    };

    if (i2s_configure(i2s_tx_dev, I2S_DIR_TX, &tx_cfg) < 0 ||
        i2s_configure(i2s_rx_dev, I2S_DIR_RX, &rx_cfg) < 0) {
        LOG_ERR("I2S configuration failed!");
        return -EIO;
    }
    LOG_INF("I2S interfaces configured (48 kHz, 16-bit stereo)");

    /* CS43L22 DAC: MCLK was already enabled by i2s_configure() on PC7.
     * Initialize DAC registers and power it up before starting audio streaming. */
    cs43l22_init(i2c_dev);

    dsp_filter_init();

    /* Pre-prime 2 blocks directly into I2S TX DMA queue so DMA has immediate buffers */
    for (int i = 0; i < 2; i++) {
        void *tx_block;
        if (k_mem_slab_alloc(&pcm_tx_slab, &tx_block, K_MSEC(100)) == 0) {
            int16_t *s = (int16_t *)tx_block;
            for (int n = 0; n < SAMPLES_PER_BLOCK; n += 2) {
                int16_t v = sine_500hz_table[(n / 2) % 96];
                s[n]     = v;   /* Left  */
                s[n + 1] = v;   /* Right */
            }
            if (i2s_write(i2s_tx_dev, tx_block, PCM_BYTES_PER_BLOCK) < 0) {
                k_mem_slab_free(&pcm_tx_slab, tx_block);
                LOG_ERR("TX pre-prime i2s_write failed!");
            }
        }
    }
    LOG_INF("TX DMA queue pre-primed with 2 sine-wave blocks");

    /* Trigger both RX (mic PDM) and TX (DAC PCM) DMA streams BEFORE starting threads.
     * Hardware is actively streaming so RX/TX threads never block on inactive streams. */
    i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    LOG_INF("I2S TX and RX streams triggered — DMA hardware active.");

    /* NOW pre-prime dsp_output_q and unblock audio pipeline threads */
    audio_pipeline_start(i2s_rx_dev, i2s_tx_dev);
    return 0;
}