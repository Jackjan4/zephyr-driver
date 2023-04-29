#define DT_DRV_COMPAT ti_dac5571

#define MODULE dac5571

// Header
#include "dac5571.h"

// Zephyr Header
#include <drivers/i2c.h>
#include <drivers/dac.h>
#ifdef CONFIG_LOG
#include <logging/log.h>
#endif


#ifdef CONFIG_LOG
LOG_MODULE_REGISTER(MODULE, CONFIG_DAC_LOG_LEVEL);
#endif

/* Information in this file comes from DAC5571 datasheet
 * found at https://www.ti.com/lit/ds/symlink/dac5571.pdf
 */



struct dac5571_config {
	struct i2c_dt_spec i2c;
};


// 
static int dac5571_get_ack(const struct device *dev) {
	
	return 0;
}

/* DAC5571 is a single channel 8 bit DAC */
static int dac5571_channel_setup(const struct device *dev, const struct dac_channel_cfg *channel_cfg) {
	if (channel_cfg->channel_id != 0) {
		return -EINVAL;
	}

	if (channel_cfg->resolution != 8) {
		return -ENOTSUP;
	}

	return 0;
}

static int dac5571_write_value(const struct device *dev, uint8_t channel, uint32_t value) {
	const struct dac5571_config *config = dev->config;
	uint8_t tx_data[2];
	tx_data[0] = 0;
	tx_data[1] = 0;
	int result;

	if (channel != 0) {
		return -EINVAL;
	}

	/* Check value isn't over 12 bits */
	if (value > DAC5571_DAC_MAX_VAL) {
		return -ENOTSUP;
	}

	/* WRITE_MODE_FAST message format (2 bytes):
	 *
	 * ||     15 14     |        13 12        |    11 10 9 8    ||       7 6 5 4      |    3 2 1 0   ||
	 * ||      0  0     | Power-down bits (0) | DAC value[11:8] ||   DAC value[7:4]   |  Don't care  ||
	 */
	tx_data[0] = (value >> 4);
	tx_data[1] = ((value & 0b00001111) << 4);
	result = i2c_write_dt(&config->i2c, tx_data, sizeof(tx_data));

	if (result != 0) {

		return result;
	}

	return result;
}

static int dac_dac5571_init(const struct device *dev) {
	const struct dac5571_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
#ifdef CONFIG_LOG
		LOG_ERR("I2C device not found");
#endif
		return -EINVAL;
	}

	// TODO: Check if DAC5571 ACKs only its address
	if (dac5571_get_ack(dev) != 0) {
		return -EBUSY;
	}

	return 0;
}

static const struct dac_driver_api dac5571_driver_api = {
	.channel_setup = dac5571_channel_setup,
	.write_value = dac5571_write_value,
};


#define INST_DT_DAC5571(index)						                \
	static const struct dac5571_config dac5571_config_##index = {	\
		.i2c = I2C_DT_SPEC_INST_GET(index),			                \
	};								                                \
									                                \
	DEVICE_DT_INST_DEFINE(index, dac_dac5571_init,			        \
			    NULL,					                            \
			    NULL,					                            \
			    &dac5571_config_##index, POST_KERNEL,	            \
			    CONFIG_DAC_DAC5571_INIT_PRIORITY,		            \
			    &dac5571_driver_api);


DT_INST_FOREACH_STATUS_OKAY(INST_DT_DAC5571);
