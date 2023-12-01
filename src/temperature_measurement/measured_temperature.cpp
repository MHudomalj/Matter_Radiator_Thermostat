/*
 * Marko Hudomalj 11.2023
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "sensor.h"
#include "temp_sensor_manager.h"

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_PATH(temperature_adc)) || \
	!DT_NODE_HAS_PROP(DT_PATH(temperature_adc), io_channels)
#error "No suitable devicetree overlay specified"
#endif
static const struct adc_dt_spec temp_adc =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(temperature_adc), 0);

MeasuredSensor::MeasuredSensor()
{
	int err;
    if (!adc_is_ready_dt(&temp_adc)) {
        LOG_ERR("ADC controller device %s not ready", temp_adc.dev->name);
        return;
    }

    err = adc_channel_setup_dt(&temp_adc);
    if (err < 0) {
        LOG_ERR("Could not setup channel (%d)", err);
        return;
    }
    LOG_INF("Temperature ADC initialized.");
}

void MeasuredSensor::TemperatureMeasurement()
{
    int err;
	uint16_t buf;
    struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};
    LOG_INF("ADC reading:");
    int32_t val_mv;

    LOG_INF("%s, channel %d: ",
    temp_adc.dev->name,
    temp_adc.channel_id);

    (void)adc_sequence_init_dt(&temp_adc, &sequence);

    err = adc_read(temp_adc.dev, &sequence);
    if (err < 0) {
        LOG_ERR("Could not read (%d)\n", err);
        return;
    }
    /*
    * If using differential mode, the 16 bit value
    * in the ADC sample buffer should be a signed 2's
    * complement value.
    */
    if (temp_adc.channel_cfg.differential) {
        val_mv = (int32_t)((int16_t)buf);
    } else {
        val_mv = (int32_t)buf;
    }
    LOG_INF("Measured voltage %d V", val_mv);
    err = adc_raw_to_millivolts_dt(&temp_adc, &val_mv);
    /* conversion to mV may not be supported, skip if not */
    if (err < 0) {
        LOG_ERR(" (value in mV not available)\n");
    } else {
        LOG_INF(" = %d mV", val_mv);
    }
    float tmp = ((float)val_mv/1000 - 0.5) * 100;
    int16_t temperature = (int16_t) (tmp*100);

    LOG_INF("Temp measured: %d", temperature);
    TempSensorManager::Instance().SetLocalTemperature(temperature);
}
