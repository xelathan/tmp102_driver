#ifndef ZEPHYR_DRIVERS_SENSOR_TMP102_TMP102_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP102_TMP102_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define TMP102_REG_TEMPERATURE 0x00
#define TMP102_DATA_INVALID_BIT         (BIT(1) | BIT(2))
#define TMP102_DATA_EXTENDED            (BIT(0))
#define TMP102_DATA_EXTENDED_SHIFT      3
#define TMP102_DATA_NORMAL_SHIFT        4

#define TMP102_REG_CONFIG 0x01
#define TMP102_CONFIG_EM    BIT(4)

#define TMP102_ALERT_EN_BIT     BIT(5)
#define TMP102_CONV_RATE_SHIFT  6
#define TMP102_CONV_RATE_MASK   (BIT_MASK(2) <<  TMP102_CONV_RATE_SHIFT)
#define TMP102_CONV_RATE_025    0
#define TMP102_CONV_RATE_1000   1
#define TMP102_CONV_RATE_4      2
#define TMP102_CONV_RATE_8      3

#define TMP102_CONV_RATE(cr)    ((cr) << TMP102_CONV_RATE_SHIFT)

#define TMP102_CONV_RES_SHIFT   13
#define TMP102_CONV_RES_MASK    (BIT_MASK(2) << TMP102_CONV_RES_SHIFT)

#define TMP102_ONE_SHOT         BIT(15)
#define TMP102_SHUT_DOWN BIT(8)
#define TMP102_THERMOSTAT BIT(9)
#define TMP102_POLARITY BIT(10)


#define TMP102_REG_TLOW 0x02
#define TMP102_REG_THIGH 0x03

/* scale in micro degrees Celsius -> 0.0625 */
#define TMP102_TEMP_SCALE       62500

/**
 * 128 / 0.0625 - 1
 * 0111 1111 1111 -> 2027
 */
#define TMP102_TEMP_MAX 2047

/**
 * -55 / 0.0625 + 1
 * 1100 1001 0000 -> 0011 0110 1111 -> 0011 0111 0000 (negative)
 */
#define TMP102_TEMP_MIN -880

/**
 * 150 / 0.0625
 */
#define TMP102_TEMP_EM_MAX 2400

#define TMP102_TEMP_EM_MIN TMP102_TEMP_MIN

struct tmp102_data {
	int16_t sample;
	uint16_t config_reg;
};

struct tmp102_config {
	const struct i2c_dt_spec bus;
	uint8_t cr;
	int32_t t_low_micro_c;
	int32_t t_high_micro_c;
	bool extended_mode : 1;
};

#endif