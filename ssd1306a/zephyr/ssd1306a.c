#define DT_DRV_COMPAT solomon_ssd1306a
#define MODULE ssd1306a



// Header
#include "ssd1306a.h"



// Zephyr Header
#ifdef CONFIG_LOG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DISPLAY_LOG_LEVEL);
#endif
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>



struct ssd1306_config {
#if DT_INST_ON_BUS(0, i2c)
    struct i2c_dt_spec bus;
#elif DT_INST_ON_BUS(0, spi)
    struct spi_dt_spec bus;
    struct gpio_dt_spec data_cmd;
#endif
    struct gpio_dt_spec reset;
    struct gpio_dt_spec supply_pin;
};



struct ssd1306_data {
    uint8_t contrast;
    uint8_t scan_mode;
};



#if DT_INST_ON_BUS(0, i2c)

static inline bool ssd1306_bus_ready(const struct device *dev) {
    const struct ssd1306_config *config = dev->config;

    return device_is_ready(config->bus.bus);
}



static inline int ssd1306_write_bus(const struct device *dev,
                                    uint8_t *buf, size_t len, bool command) {
    const struct ssd1306_config *config = dev->config;

    return i2c_burst_write_dt(&config->bus,
                              command ? SSD1306_CONTROL_ALL_BYTES_CMD : SSD1306_CONTROL_ALL_BYTES_DATA,
                              buf, len);
}

#elif DT_INST_ON_BUS(0, spi)

static inline bool ssd1306_bus_ready(const struct device *dev) {
    const struct ssd1306_config *config = dev->config;

    if (gpio_pin_configure_dt(&config->data_cmd, GPIO_OUTPUT_INACTIVE) < 0) {
        return false;
    }

    return spi_is_ready_dt(&config->bus);
}

static inline int ssd1306_write_bus(const struct device *dev,
                                    uint8_t *buf, size_t len, bool command) {
    const struct ssd1306_config *config = dev->config;
    int errno;

    gpio_pin_set_dt(&config->data_cmd, command ? 0 : 1);
    struct spi_buf tx_buf = {
        .buf = buf,
        .len = len};

    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1};

    errno = spi_write_dt(&config->bus, &tx_bufs);

    return errno;
}
#endif


static inline int ssd1306_set_multiplex_ratio(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_MULTIPLEX_RATIO,
        DT_INST_PROP(0, multiplex_ratio)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_display_offset(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_DISPLAY_OFFSET,
        DT_INST_PROP(0, display_offset)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_display_start_line(const struct device *dev) {
    uint8_t cmd_buf[] = {
        DT_INST_PROP(0, display_start_line)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_memory_addressing_mode(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_MEM_ADDRESSING_MODE,
        DT_INST_PROP(0, memory_addressing_mode)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_segment_remap(const struct device *dev) {
    uint8_t cmd_buf[] = {
#if DT_INST_PROP(0, segment_remap) == 1
        SSD1306_SET_SEGMENT_MAP_REMAPED
#else
        SSD1306_SET_SEGMENT_MAP_NORMAL
#endif
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_com_scan_direction(const struct device *dev) {
    uint8_t cmd_buf[] = {
#if DT_INST_PROP(0, com_invert_direction) == 1
        SSD1306_SET_COM_OUTPUT_SCAN_FLIPPED
#else
        SSD1306_SET_COM_OUTPUT_SCAN_NORMAL
#endif
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_com_pins_hw_config(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_COM_PINS_HW_CONFIG,
        DT_INST_PROP(0, com_pins_config)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_disable_entire_display_on(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_DISABLE_ENTIRE_DISPLAY_ON,
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static inline int ssd1306_set_display_normal(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_NORMAL_DISPLAY,
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static inline int ssd1306_set_clock_div_osc_frequency(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_CLOCK_DIV_OSC_FREQ,
        DT_INST_PROP(0, clock_div_osc_frequency)};

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static inline int ssd1306_set_charge_pump(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_CHARGE_PUMP_SETTING,
#if DT_INST_PROP(0, charge_pump_enable) == 1
        SSD1306_ENABLE_CHARGE_PUMP,
#else
        SSD1306_DISABLE_CHARGE_PUMP,
#endif
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}


/**
 * Turn OLED panel on
 */
static inline int ssd1306_set_display_on(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_DISPLAY_ON,
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static int ssd1306_suspend(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1306_DISPLAY_OFF,
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static int ssd1306_write(const struct device *dev, const uint16_t x, const uint16_t y,
                         const struct display_buffer_descriptor *desc,
                         const void *buf) {
    size_t buf_len;

    if (desc->pitch < desc->width) {
        LOG_ERR("Pitch is smaller then width");
        return -1;
    }

    buf_len = MIN(desc->buf_size, desc->height * desc->width / 8);
    if (buf == NULL || buf_len == 0U) {
        LOG_ERR("Display buffer is not available");
        return -1;
    }

    if (desc->pitch > desc->width) {
        LOG_ERR("Unsupported mode");
        return -1;
    }

    if ((y & 0x7) != 0U) {
        LOG_ERR("Unsupported origin");
        return -1;
    }

    LOG_DBG("x %u, y %u, pitch %u, width %u, height %u, buf_len %u",
            x, y, desc->pitch, desc->width, desc->height, buf_len);

    return ssd1306_write_bus(dev, (uint8_t *)buf, buf_len, false);
}



static int ssd1306_read(const struct device *dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor *desc, void *buf) {
    LOG_ERR("Unsupported");
    return -ENOTSUP;
}



static void *ssd1306_get_framebuffer(const struct device *dev) {
    LOG_ERR("Unsupported");
    return NULL;
}



static int ssd1306_set_brightness(const struct device *dev, const uint8_t brightness) {
    LOG_WRN("Unsupported");
    return -ENOTSUP;
}



static int ssd1306_set_contrast(const struct device *dev, const uint8_t contrast) {
    uint8_t cmd_buf[] = {
        SSD1306_SET_CONTRAST_CTRL,
        contrast,
    };

    return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}



static void ssd1306_get_capabilities(const struct device *dev, struct display_capabilities *caps) {
    memset(caps, 0, sizeof(struct display_capabilities));
    caps->x_resolution = DT_INST_PROP(0, width);
    caps->y_resolution = DT_INST_PROP(0, height);
    caps->supported_pixel_formats = PIXEL_FORMAT_MONO10;
    caps->current_pixel_format = PIXEL_FORMAT_MONO10;
    caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

static int ssd1306_set_orientation(const struct device *dev, const enum display_orientation orientation) {
    LOG_ERR("Unsupported");
    return -ENOTSUP;
}



static int ssd1306_set_pixel_format(const struct device *dev, const enum display_pixel_format pf) {
    if (pf == PIXEL_FORMAT_MONO10) {
        return 0;
    }
    LOG_ERR("Unsupported");
    return -ENOTSUP;
}



static int ssd1306_init_flow(const struct device *dev) {
    int err = 0;
    const struct ssd1306_config *config = dev->config;

    /* Reset if pin connected */
    if (config->reset.port) {
        k_sleep(K_MSEC(SSD1306_RESET_DELAY));
        gpio_pin_set_dt(&config->reset, 1);
        k_sleep(K_MSEC(SSD1306_RESET_DELAY));
        gpio_pin_set_dt(&config->reset, 0);
    }

    /// The following init flow is a slightly modified version of the init flow from the SSD1306 datasheet. Modifications are commented


    if (ssd1306_set_multiplex_ratio(dev)) {
        return -EIO;
    }

    if (ssd1306_set_display_offset(dev)) {
        return -EIO;
    }

    if (ssd1306_set_display_start_line(dev)) {
        return -EIO;
    }

    // This is not in the normal init flow. It can however be configured.
    if (ssd1306_set_memory_addressing_mode(dev)) {
        return -EIO;
    }

    if (ssd1306_set_segment_remap(dev)) {
        return -EIO;
    }

    if (ssd1306_set_com_scan_direction(dev)) {
        return -EIO;
    }

    if (ssd1306_set_com_pins_hw_config(dev)) {
        return -EIO;
    }

    if (ssd1306_set_com_pins_hw_config(dev)) {
        return -EIO;
    }

    if (ssd1306_set_contrast(dev, DT_INST_PROP(0, default_contrast))) {
        return -EIO;
    }

    // Display reads from GDDRAM instead of all pixels on
    if (ssd1306_disable_entire_display_on(dev)) {
        return -EIO;
    }

    // Normal = Colors not inverted
    if (ssd1306_set_display_normal(dev)) {
        return -EIO;
    }

    if (ssd1306_set_clock_div_osc_frequency(dev)) {
        return -EIO;
    }

    if (ssd1306_set_charge_pump(dev)) {
        return -EIO;
    }

    return err;
}



static int ssd1306_resume(const struct device *dev) {
    int err = 0;
    const struct ssd1306_config *config = dev->config;

    if (config->supply_pin.port) {
        // If supply_pin is set, start supply and go through init flow
        err = gpio_pin_set_dt(&config->supply_pin, 1);
        k_msleep(50);
        err = ssd1306_init_flow(dev);
    }

    err = ssd1306_set_display_on(dev);
    return err;
}



static int ssd1306_init_device(const struct device *dev) {
    int err = 0;
    const struct ssd1306_config *config = dev->config;

    if (!config->supply_pin.port) {
        err = ssd1306_init_flow(dev);
    }

    return err;
}



static int ssd1306_init(const struct device *dev) {
    int err = 0;
    const struct ssd1306_config *config = dev->config;

    if (!ssd1306_bus_ready(dev)) {
        LOG_ERR("Bus device %s not ready!", config->bus.bus->name);
        return -EINVAL;
    }

    if (config->reset.port) {
        err = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
        if (err < 0) {
            return err;
        }
    }

    if (config->supply_pin.port) {
        err = gpio_pin_configure_dt(&config->supply_pin, GPIO_OUTPUT_INACTIVE);
        if (err < 0) {
            return err;
        }
    }

    if (ssd1306_init_device(dev)) {
        LOG_ERR("Failed to initialize device!");
        return -EIO;
    }

    return err;
}



static const struct ssd1306_config ssd1306_config = {
#if DT_INST_ON_BUS(0, i2c)
    .bus = I2C_DT_SPEC_INST_GET(0),
#elif DT_INST_ON_BUS(0, spi)
    .bus = SPI_DT_SPEC_INST_GET(
        0, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),
    .data_cmd = GPIO_DT_SPEC_INST_GET(0, data_cmd_gpios),
#endif
    .reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, {0}),
    .supply_pin = GPIO_DT_SPEC_INST_GET_OR(0, supply_gpios, {0})};



static struct ssd1306_data ssd1306_driver;



static struct display_driver_api ssd1306_driver_api = {
    .blanking_on = ssd1306_suspend,
    .blanking_off = ssd1306_resume,
    .write = ssd1306_write,
    .read = ssd1306_read,
    .get_framebuffer = ssd1306_get_framebuffer,
    .set_brightness = ssd1306_set_brightness,
    .set_contrast = ssd1306_set_contrast,
    .get_capabilities = ssd1306_get_capabilities,
    .set_pixel_format = ssd1306_set_pixel_format,
    .set_orientation = ssd1306_set_orientation,
};



DEVICE_DT_INST_DEFINE(0, ssd1306_init, NULL,
                      &ssd1306_driver, &ssd1306_config,
                      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
                      &ssd1306_driver_api);
