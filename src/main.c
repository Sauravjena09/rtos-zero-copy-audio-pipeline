#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_loopback, LOG_LEVEL_INF);

#define SAMPLE_RATE 48000U
#define SAMPLE_BIT_WIDTH 16U
#define BYTES_PER_SAMPLE   sizeof(int16_t)
#define NUM_CHANNELS 2U
#define SAMPLES_PER_BLOCK   ((SAMPLE_RATE / 100) * NUM_CHANNELS)
#define BLOCK_SIZE          (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT         6U

K_MEM_SLAB_DEFINE(audio_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

#define CS43L22_I2C_ADDR  0x4AU

static const struct gpio_dt_spec dac_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(cs43l22), reset_gpios);

static int cs43l22_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    return i2c_write(i2c_dev, data, sizeof(data), CS43L22_I2C_ADDR);
}

static int cs43l22_init(const struct device *i2c_dev)
{
    gpio_pin_configure_dt(&dac_reset, GPIO_OUTPUT_INACTIVE);
    k_msleep(5);
    gpio_pin_set_dt(&dac_reset, 1);
    k_msleep(5);
    cs43l22_write_reg(i2c_dev, 0x02, 0x01);
    cs43l22_write_reg(i2c_dev, 0x00, 0x99);
    cs43l22_write_reg(i2c_dev, 0x47, 0x80);
    cs43l22_write_reg(i2c_dev, 0x32, 0x80);
    cs43l22_write_reg(i2c_dev, 0x32, 0x00);
    cs43l22_write_reg(i2c_dev, 0x00, 0x00);
    cs43l22_write_reg(i2c_dev, 0x04, 0xAF);
    cs43l22_write_reg(i2c_dev, 0x05, 0x81);
    cs43l22_write_reg(i2c_dev, 0x06, 0x07);
    cs43l22_write_reg(i2c_dev, 0x20, 0xF4);
    cs43l22_write_reg(i2c_dev, 0x21, 0xF4);
    return cs43l22_write_reg(i2c_dev, 0x02, 0x9E);
}

static void fill_test_tone(int16_t *pcm_buf, size_t num_samples)
{
    for (size_t i = 0; i < num_samples; i += 2) {
        int16_t sample = ((i / 2) % 96 < 48) ? 3000 : -3000;
        pcm_buf[i]     = sample;
        pcm_buf[i + 1] = sample;
    }
}

int main(void)
{
#if defined(CONFIG_USB_DEVICE_STACK) && !defined(CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT)
    usb_enable(NULL);
#endif
    k_msleep(1000); 

    const struct device *i2c_dev    = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    const struct device *i2s_tx_dev = DEVICE_DT_GET(DT_NODELABEL(i2s3));
    const struct device *i2s_rx_dev = DEVICE_DT_GET(DT_NODELABEL(i2s2));

    if (!device_is_ready(i2c_dev) || !device_is_ready(i2s_tx_dev) || !device_is_ready(i2s_rx_dev)) {
        LOG_ERR("Audio peripherals not ready!");
        return -EIO;
    }

    struct i2s_config tx_cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_RATE,
        .mem_slab       = &audio_mem_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 2000,
    };

    struct i2s_config rx_cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
        .frame_clk_freq = SAMPLE_RATE,
        .mem_slab       = &audio_mem_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 2000,
    };

    if (i2s_configure(i2s_tx_dev, I2S_DIR_TX, &tx_cfg) < 0 ||
        i2s_configure(i2s_rx_dev, I2S_DIR_RX, &rx_cfg) < 0) {
        LOG_ERR("I2S configuration failed!");
        return -EIO;
    }

    for (int i = 0; i < 2; i++) {
        void *init_block;
        k_mem_slab_alloc(&audio_mem_slab, &init_block, K_FOREVER);
        fill_test_tone((int16_t *)init_block, SAMPLES_PER_BLOCK);
        i2s_write(i2s_tx_dev, init_block, BLOCK_SIZE);
    }

    i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);

    cs43l22_init(i2c_dev);
    
    LOG_INF("Commit 10(a) Active: Zero-Copy I2S Loopback Running!");

    while (1) {
        void *mem_block;
        size_t bytes_read;

        int ret = i2s_read(i2s_rx_dev, &mem_block, &bytes_read);
        if (ret < 0) {
            LOG_WRN("RX ERR: %d. Recovering...", ret);
            i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
            i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
            continue;
        }

        ret = i2s_write(i2s_tx_dev, mem_block, bytes_read);
        if (ret < 0) {
            k_mem_slab_free(&audio_mem_slab, mem_block);
            LOG_WRN("TX ERR: %d. Recovering...", ret);
            i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
        }
    }
    return 0;
}