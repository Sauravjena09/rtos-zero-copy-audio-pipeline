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

    /* Required initialization sequence for CS43L22 */
    ret |= cs43l22_write_reg(i2c_dev, 0x02, 0x01); /* Power down */
    ret |= cs43l22_write_reg(i2c_dev, 0x00, 0x99);
    ret |= cs43l22_write_reg(i2c_dev, 0x47, 0x80);
    ret |= cs43l22_write_reg(i2c_dev, 0x32, 0x80);
    ret |= cs43l22_write_reg(i2c_dev, 0x32, 0x00);
    ret |= cs43l22_write_reg(i2c_dev, 0x00, 0x00);

    /* Power Control 2: Headphone always ON, speaker OFF */
    ret |= cs43l22_write_reg(i2c_dev, 0x04, 0xAF);
    /* Clocking Control: Auto-detect clock speed */
    ret |= cs43l22_write_reg(i2c_dev, 0x05, 0x81);
    /* Interface Control 1: I2S data format, 16-bit */
    ret |= cs43l22_write_reg(i2c_dev, 0x06, 0x07);
    /* Headphone Channel A & B Volume (-6dB) */
    ret |= cs43l22_write_reg(i2c_dev, 0x20, 0xF4);
    ret |= cs43l22_write_reg(i2c_dev, 0x21, 0xF4);
    /* Power Control 1: Power Up */
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
        .mem_slab       = &audio_mem_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 1000,
    };

    struct i2s_config rx_cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
        .frame_clk_freq = SAMPLE_RATE,
        .mem_slab       = &audio_mem_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 0, /* Non-blocking: check if RX block ready without waiting */
    };

    if (i2s_configure(i2s_tx_dev, I2S_DIR_TX, &tx_cfg) < 0 ||
        i2s_configure(i2s_rx_dev, I2S_DIR_RX, &rx_cfg) < 0) {
        LOG_ERR("I2S configuration failed!");
        return -EIO;
    }
    LOG_INF("I2S interfaces configured (48 kHz, 16-bit stereo)");

    /* Prime TX DMA queue with smooth 500 Hz sine wave */
    for (int i = 0; i < 2; i++) {
        void *init_block;
        k_mem_slab_alloc(&audio_mem_slab, &init_block, K_FOREVER);
        int16_t *samples = (int16_t *)init_block;
        for (int s = 0; s < SAMPLES_PER_BLOCK; s += 2) {
            int16_t val = sine_500hz_table[(s / 2) % 96];
            samples[s]     = val;
            samples[s + 1] = val;
        }
        i2s_write(i2s_tx_dev, init_block, BLOCK_SIZE);
    }
    LOG_INF("TX DMA queue primed with sine wave");

    audio_pipeline_start(i2s_rx_dev, i2s_tx_dev);

    /* Start I2S DMA streams */
    i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    LOG_INF("I2S RX and TX streams triggered");

    /* Initialize DAC after MCLK is running from I2S3 */
    cs43l22_init(i2c_dev);

    LOG_INF("Commit 10b: Boot Sequence Complete. Audio pipeline active.");
    return 0;
}