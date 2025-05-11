// compatible = "ti,tmp102"
#define DT_DRV_COMPAT ti_tmp102

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "tmp102.h"

LOG_MODULE_REGISTER(TMP102, CONFIG_SENSOR_LOG_LEVEL);

static int tmp102_reg_read(const struct tmp102_config* cfg, uint8_t reg, uint16_t* val) {
    int ret;
    ret = i2c_burst_read_dt(&cfg->bus, reg, (uint8_t*)val, sizeof(*val));
    if (ret < 0) {
        return -EIO;
    }

    *val = sys_be16_to_cpu(*val);

	return 0;
}

static int tmp102_reg_write(const struct tmp102_config* cfg, uint8_t reg, uint16_t val) {
    int ret;
    uint16_t be_val = sys_cpu_to_be16(val);

    ret = i2c_burst_write_dt(&cfg->bus, reg, (uint8_t*)&be_val, sizeof(be_val));

    if (ret < 0) {
        return -EIO;
    }

    return 0;
}

static int set_config_flags(struct tmp102_data* data, uint16_t mask, uint16_t value) {
    return (data->config_reg & ~mask) | (value & mask);
}

static int tmp102_update_config(const struct device* dev, uint16_t mask, uint16_t value) {
    int ret;
    struct tmp102_data* data = dev->data;
    const uint16_t new_val = set_config_flags(data, mask, value);

    ret = tmp102_reg_write(dev->config, TMP102_REG_CONFIG, new_val);
    if (ret == 0) {
        data->config_reg = new_val;
    }

    return ret;
}

static int tmp102_set_threshold(const struct device* dev, uint8_t reg, int64_t micro_c) {
    struct tmp102_data *drv_data = dev->data;
	int64_t v;
	uint16_t reg_value;

	v = DIV_ROUND_CLOSEST(micro_c, TMP102_TEMP_SCALE);

    if (drv_data->config_reg & TMP102_CONFIG_EM) {
        if (!IN_RANGE(v, TMP102_TEMP_EM_MIN, TMP102_TEMP_EM_MAX)) {
            return -EINVAL;
        }
        reg_value = (uint16_t)v << TMP102_DATA_EXTENDED_SHIFT;
    } else {
        if (!IN_RANGE(v, TMP102_TEMP_MIN, TMP102_TEMP_MAX)) {
            return -EINVAL;
        }
        reg_value = (uint16_t)v << TMP102_DATA_NORMAL_SHIFT;
    }

    return tmp102_reg_write(dev->config, reg, reg_value);
}

static int tmp102_attr_set(const struct device* dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {
    uint16_t value;
	uint16_t cr;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    switch(attr) {
        case SENSOR_ATTR_FULL_SCALE:
            /* the sensor supports two ranges -55 to 128 and -55 to 150 */
            /* the value contains the upper limit */
            if (val->val1 == 128) {
                value = 0x0000;
            } else if (val->val1 == 150) {
                value = TMP102_CONFIG_EM;
            } else {
                return -ENOTSUP;
            }

            if (tmp102_update_config(dev, TMP102_CONFIG_EM, value) < 0) {
                LOG_DBG("Failed to set attribute!");
                return -EIO;
            }
            break;

        case SENSOR_ATTR_SAMPLING_FREQUENCY:
            /* conversion rate in mHz */
            cr = val->val1 * 1000 + val->val2 / 1000;
    
            /* the sensor supports 0.25Hz, 1Hz, 4Hz and 8Hz */
            /* conversion rate */
            switch (cr) {
                case 250:
                    value = TMP102_CONV_RATE(TMP102_CONV_RATE_025);
                    break;
        
                case 1000:
                    value = TMP102_CONV_RATE(TMP102_CONV_RATE_1000);
                    break;
        
                case 4000:
                    value = TMP102_CONV_RATE(TMP102_CONV_RATE_4);
                    break;
        
                case 8000:
                    value = TMP102_CONV_RATE(TMP102_CONV_RATE_8);
                    break;
        
                default:
                    return -ENOTSUP;
            }
    
            if (tmp102_update_config(dev, TMP102_CONV_RATE_MASK, value) < 0) {
                LOG_DBG("Failed to set attribute!");
                return -EIO;
            }
    
            break;
        case SENSOR_ATTR_LOWER_THRESH:
		    return tmp102_set_threshold(dev, TMP102_REG_TLOW, sensor_value_to_micro(val));

        case SENSOR_ATTR_UPPER_THRESH:
            return tmp102_set_threshold(dev, TMP102_REG_THIGH, sensor_value_to_micro(val));

        default:
            return -ENOTSUP;
    }

    return 0;
}

static int tmp102_sample_fetch(const struct device* dev, enum sensor_channel chan) {
    struct tmp102_data* drv_data = dev->data;
    const struct tmp102_config* drv_cfg = dev->config;
    uint16_t val;
    int ret;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

    ret = tmp102_reg_read(drv_cfg, TMP102_REG_TEMPERATURE, &val);
    if (ret < 0) {
        return -EIO;
    }

    if (val & TMP102_DATA_EXTENDED) {
        drv_data->sample = arithmetic_shift_right(val, TMP102_DATA_EXTENDED_SHIFT);
    } else {
        drv_data->sample = arithmetic_shift_right(val, TMP102_DATA_NORMAL_SHIFT);
    }

    return 0;
}

static int tmp102_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val) {
    struct tmp102_data* data = dev->data;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    return sensor_value_from_micro(val, (int32_t)data->sample * TMP102_TEMP_SCALE);
}

static DEVICE_API(sensor, tmp102_driver_api) = {
	.attr_set = tmp102_attr_set,
	.sample_fetch = tmp102_sample_fetch,
	.channel_get = tmp102_channel_get,
};

int tmp102_init(const struct device *dev)
{
	const struct tmp102_config *cfg = dev->config;
	struct tmp102_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
		return -EINVAL;
	}

	data->config_reg = TMP102_CONV_RATE(cfg->cr) | TMP102_CONV_RES_MASK |
			   (cfg->extended_mode ? TMP102_CONFIG_EM : 0);

	ret = tmp102_update_config(dev, 0, 0);
	if (ret) {
		LOG_ERR("Failed to set configuration (%d)", ret);
		return ret;
	}

	ret = tmp102_set_threshold(dev, TMP102_REG_TLOW, cfg->t_low_micro_c);
	if (ret) {
		LOG_ERR("Failed to set tLow threshold (%d)", ret);
		return ret;
	}

	ret = tmp102_set_threshold(dev, TMP102_REG_THIGH, cfg->t_high_micro_c);
	if (ret) {
		LOG_ERR("Failed to set tHigh threshold (%d)", ret);
		return ret;
	}

	return 0;
}

#define TMP102_INST(inst)                                                                          \
	static struct tmp102_data tmp102_data_##inst;                                              \
	static const struct tmp102_config tmp102_config_##inst = {                                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.cr = DT_INST_ENUM_IDX(inst, conversion_rate),                                     \
		.t_low_micro_c = DT_INST_PROP(inst, t_low_micro_c),                                \
		.t_high_micro_c = DT_INST_PROP(inst, t_high_micro_c),                              \
		.extended_mode = DT_INST_PROP(inst, extended_mode),                                \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tmp102_init, NULL, &tmp102_data_##inst,                 \
				     &tmp102_config_##inst, POST_KERNEL,                           \
				     CONFIG_SENSOR_INIT_PRIORITY, &tmp102_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMP102_INST)
