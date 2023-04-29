#define DT_DRV_COMPAT solomon_ssd1327

#define MODULE ssd1327

// Header
#include "ssd1327.h"

// Zephyr header
#include <device.h>
#include <drivers/display.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/spi.h>
#ifdef CONFIG_LOG
#include <logging/log.h>
#endif

// Logging
#ifdef CONFIG_LOG
LOG_MODULE_REGISTER(MODULE, CONFIG_DISPLAY_LOG_LEVEL);
#endif

#if DT_INST_PROP(0, com_invdir) == 1
#define SSD1327_PANEL_COM_INVDIR true
#else
#define SSD1327_PANEL_COM_INVDIR false
#endif

#if DT_INST_PROP(0, com_sequential) == 1
#define SSD1327_COM_PINS_HW_CONFIG SSD1327_SET_PADS_HW_SEQUENTIAL
#else
#define SSD1327_COM_PINS_HW_CONFIG SSD1327_SET_PADS_HW_ALTERNATIVE
#endif

#define SSD1327_PANEL_NUMOF_PAGES (DT_INST_PROP(0, height) / 8)
#define SSD1327_PANEL_VCOM_DESEL_LEVEL 0x20
#define SSD1327_PANEL_PUMP_VOLTAGE SSD1327_SET_PUMP_VOLTAGE_90

#ifndef SSD1327_ADDRESSING_MODE
#define SSD1327_ADDRESSING_MODE (SSD1327_SET_MEM_ADDRESSING_HORIZONTAL)
#endif

// Struct definitions
struct ssd1327_config {
#if DT_INST_ON_BUS(0, i2c)
    struct i2c_dt_spec bus;
#elif DT_INST_ON_BUS(0, spi)
    struct spi_dt_spec bus;
    struct gpio_dt_spec data_cmd;
#endif
    struct gpio_dt_spec reset;
};

struct ssd1327_data {
    uint8_t contrast;
    uint8_t scan_mode;
};

// bus_ready & write_bus
#if DT_INST_ON_BUS(0, i2c)

static inline bool ssd1327_bus_ready(const struct device *dev) {
    const struct ssd1327_config *config = dev->config;

    return device_is_ready(config->bus.bus);
}

static inline int ssd1327_write_bus(const struct device *dev, uint8_t *buf, size_t len, bool command) {
    const struct ssd1327_config *config = dev->config;

    return i2c_burst_write_dt(&config->bus, command ? 0x00 : 0x40, buf, len);
}

#elif DT_INST_ON_BUS(0, spi)

static inline bool ssd1327_bus_ready(const struct device *dev) {
    const struct ssd1327_config *config = dev->config;

    if (gpio_pin_configure_dt(&config->data_cmd, GPIO_OUTPUT_INACTIVE) < 0) {
        return false;
    }

    return spi_is_ready(&config->bus);
}

static inline int ssd1327_write_bus(const struct device *dev, uint8_t *buf, size_t len, bool command) {
    const struct ssd1327_config *config = dev->config;
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

static inline int ssd1327_set_remap(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_REMAP,
        DT_INST_PROP(0, remap),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_timing_settings
static inline int ssd1327_set_phase_length(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_PHASE_LENGTH,
        DT_INST_PROP(0, phase_length),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_timing_settings
static inline int ssd1327_set_clock_div_frequency(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_CLK_OSC,
        DT_INST_PROP(0, front_clock_frequency),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_func_b
static inline int ssd1327_set_func_b(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_FUNCTION_SELB,
        DT_INST_PROP(0, function_b),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_precharge_voltage
static inline int ssd1327_set_precharge_voltage(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_PRECHARGE,
        DT_INST_PROP(0, precharge_voltage),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_vcom_voltage
static inline int ssd1327_set_vcom_voltage(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_VCOM_VOLTAGE,
        DT_INST_PROP(0, vcom_voltage),
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_set_vcom_voltage
static inline int ssd1327_set_grayscale_table(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_GRAYSCALE_TABLE,
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x10,
        0x18,
        0x20,
        0x2f,
        0x38,
        0x3f,
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_resume
static int ssd1327_resume(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_DISPLAYON,
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_suspend
static int ssd1327_suspend(const struct device *dev) {
    uint8_t cmd_buf[] = {
        SSD1327_DISPLAYOFF,
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int ssd1327_write(const struct device *dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor *desc, const void *buf) {
    size_t buf_len;
    int err;

    if (desc->pitch < desc->width) {
#ifdef CONFIG_LOG
        LOG_ERR("Pitch is smaller then width");
#endif
        return -1;
    }

    uint8_t *buffer = (uint8_t *)buf;
    buf_len = MIN(desc->buf_size, (desc->height * desc->width) >> 1);
    if (!buf || buf_len == 0U) {
#ifdef CONFIG_LOG
        LOG_ERR("Display buffer is not available");
#endif
        return -1;
    }

    if (desc->pitch > desc->width) {
#ifdef CONFIG_LOG
        LOG_ERR("Unsupported mode");
#endif
        return -1;
    }

    if ((y & 0x7) != 0U) {
#ifdef CONFIG_LOG
        LOG_ERR("Unsupported origin");
#endif
        return -1;
    }

    //LOG_DBG("inc. buf: x: %u, y: %u, pitch %u, wdth: %u, hght: %u, len %u", x, y, desc->pitch, desc->width, desc->height, buf_len);

    uint8_t cmd_buf[] = {
        SSD1327_SET_COLUMN_ADDRESS,
        x / 2,
        (x / 2) + (desc->width / 2) - 1,
        SSD1327_SET_ROW_ADDRESS,
        y,
        (y + desc->height - 1)};

    if (ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
#ifdef CONFIG_LOG
        LOG_ERR("Failed to write I2C/SPI command");
#endif
        return -1;
    }

    // Check if the incoming buffer is from LittleVGL
    if (desc->buf_size == (desc->height * desc->width) >> 3) {
        uint8_t *new_buffer;
        uint16_t new_buffer_size = (desc->height * desc->width) >> 1;
        new_buffer = (uint8_t *)k_malloc(new_buffer_size);

        for (uint8_t rows = 0; rows < desc->height / 8; rows++) {
            for (uint8_t cols = 0; cols < desc->width; cols += 2) {
                // maybe pitch instead of width is more correct here??
                uint8_t byte_1 = buffer[rows * desc->width + cols];

                // maybe pitch instead of width is more correct here??
                uint8_t byte_2 = buffer[rows * desc->width + cols + 1];

                uint8_t new_val;
                for (uint8_t i = 0; i < 8; i++) {
                    new_val = 0;
                    new_val = byte_1 & BIT(i) ? new_val | 0b11110000 : new_val;
                    new_val = byte_2 & BIT(i) ? new_val | 0b1111 : new_val;

                    // maybe pitch instead of width is more correct here??
                    uint16_t new_buffer_position = x + i * (desc->width / 2) + (cols / 2) + (rows * 8 * (desc->width / 2));
                    new_buffer[new_buffer_position] = new_val;
                }
            }
        }

        err = ssd1327_write_bus(dev, new_buffer, new_buffer_size, false);
        k_free(new_buffer);
        return err;
    }

    err = ssd1327_write_bus(dev, buffer, buf_len, false);
    return err;
}

// ssd1327_read
static int ssd1327_read(const struct device *dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor *desc, void *buf) {
#ifdef CONFIG_LOG
    LOG_ERR("Unsupported");
#endif
    return -ENOTSUP;
}

// ssd1327_get_framebuffer
static void *ssd1327_get_framebuffer(const struct device *dev) {
#ifdef CONFIG_LOG
    LOG_ERR("Unsupported");
#endif
    return NULL;
}

// ssd1327_set_brightness
static int ssd1327_set_brightness(const struct device *dev,
                                  const uint8_t brightness) {
#ifdef CONFIG_LOG
    LOG_WRN("Unsupported");
#endif
    return -ENOTSUP;
}

// ssd1327_set_contrast
static int ssd1327_set_contrast(const struct device *dev, const uint8_t contrast) {
    uint8_t cmd_buf[] = {
        SSD1327_SET_CONTRAST,
        contrast,
    };

    return ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

// ssd1327_get_capabilities
static void ssd1327_get_capabilities(const struct device *dev, struct display_capabilities *caps) {
    memset(caps, 0, sizeof(struct display_capabilities));
    caps->x_resolution = DT_INST_PROP(0, width);
    caps->y_resolution = DT_INST_PROP(0, height);
    caps->supported_pixel_formats = PIXEL_FORMAT_MONO10;
    caps->current_pixel_format = PIXEL_FORMAT_MONO10;
    caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

// ssd1327_set_orientation
static int ssd1327_set_orientation(const struct device *dev, const enum display_orientation orientation) {
#ifdef CONFIG_LOG
    LOG_ERR("Unsupported");
#endif
    return -ENOTSUP;
}

// set_pixel_format
static int ssd1327_set_pixel_format(const struct device *dev, const enum display_pixel_format pf) {
    if (pf == PIXEL_FORMAT_MONO10) {
        return 0;
    }
#ifdef CONFIG_LOG
    LOG_ERR("Unsupported");
#endif
    return -ENOTSUP;
}

// init_device
static int ssd1327_init_device(const struct device *dev) {
    const struct ssd1327_config *config = (const struct ssd1327_config *)dev->config;

    uint8_t cmd_buf[] = {
        SSD1327_DISPLAYALLOFF,
#ifdef CONFIG_SSD1327_INVERSE_MODE
        SSD1327_INVERVSEDISPLAY,
#else
        SSD1327_NORMALDISPLAY,
#endif
    };

    /* Reset if pin connected */
    if (config->reset.port) {
        k_sleep(K_MSEC(SSD1327_RESET_DELAY));
        gpio_pin_set_dt(&config->reset, 1);
        k_sleep(K_MSEC(SSD1327_RESET_DELAY));
        gpio_pin_set_dt(&config->reset, 0);
    }

    /* Turn display off */
    if (ssd1327_suspend(dev)) {
        return -EIO;
    }

    // SSD1327 Register mapping (Adafruit: 81 (0x51))
    if (ssd1327_set_remap(dev)) {
        return -EIO;
    }

    // Set default contrast (Adafruit: 128 (0x80))
    if (ssd1327_set_contrast(dev, 128)) {
        return -EIO;
    }

    // Set phase length (Adafruit: 17 (0x11))
    if (ssd1327_set_phase_length(dev)) {
        return -EIO;
    }

    // Set Function B (Adafruit: 98 (0x62))
    if (ssd1327_set_func_b(dev)) {
        return -EIO;
    }

    // Set precharge voltage (Adafruit: 8 (0x08))
    if (ssd1327_set_precharge_voltage(dev)) {
        return -EIO;
    }

    // Set precharge voltage (Adafruit: 8 (0x08))
    if (ssd1327_set_clock_div_frequency(dev)) {
        return -EIO;
    }

    // Adafruit additionally sets:
    // - CMD_LOCK (0xFD) to 0x12 (which is reset default)
    // - FUNC_A (0xAB) to 0x01 (which is reset default)
    // - PRECHARGE_2 (0xB6) to 0x04 (which is reset default)
	// - GRAYSCALE_TABLE 

    // Set VCOM voltage level (Adafruit 15 (0x0F) [actually 0x07 is the max value according to datasheet])
    if (ssd1327_set_vcom_voltage(dev)) {
        return -EIO;
    }

    // if (ssd1327_set_grayscale_table(dev)) {
    //     return -EIO;
    // }

    // Turn display all off and set to normal/inverse mode
    if (ssd1327_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
        return -EIO;
    }

    // Turn display back on
    ssd1327_resume(dev);

    return 0;
}

// ssd1327_init
static int ssd1327_init(const struct device *dev) {
    const struct ssd1327_config *config = (const struct ssd1327_config *)dev->config;

    if (!ssd1327_bus_ready(dev)) {
#ifdef CONFIG_LOG
        LOG_ERR("Bus device %s not ready!", config->bus.bus->name);
#endif
        return -EINVAL;
    }

    if (config->reset.port) {
        int ret;

        ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            return ret;
        }
    }

    if (ssd1327_init_device(dev)) {
#ifdef CONFIG_LOG
        LOG_ERR("Failed to initialize device!");
#endif
        return -EIO;
    }

    return 0;
}

static const struct ssd1327_config ssd1327_config = {
#if DT_INST_ON_BUS(0, i2c)
    .bus = I2C_DT_SPEC_INST_GET(0),
#elif DT_INST_ON_BUS(0, spi)
    .bus = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),
    .data_cmd = GPIO_DT_SPEC_INST_GET(0, data_cmd_gpios),
#endif
    .reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, {0})};

static struct ssd1327_data ssd1327_driver;

static struct display_driver_api ssd1327_driver_api = {
    .blanking_on = ssd1327_suspend,
    .blanking_off = ssd1327_resume,
    .write = ssd1327_write,
    .read = ssd1327_read,
    .get_framebuffer = ssd1327_get_framebuffer,
    .set_brightness = ssd1327_set_brightness,
    .set_contrast = ssd1327_set_contrast,
    .get_capabilities = ssd1327_get_capabilities,
    .set_pixel_format = ssd1327_set_pixel_format,
    .set_orientation = ssd1327_set_orientation,
};

DEVICE_DT_INST_DEFINE(0, ssd1327_init, NULL,
                      &ssd1327_driver, &ssd1327_config,
                      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
                      &ssd1327_driver_api);
