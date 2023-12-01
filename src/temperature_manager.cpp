/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "temperature_manager.h"
#include "app_config.h"
#include "app_task.h"
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_NODELABEL(motor1))
#error "Overlay for motor1 node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(motor2))
#error "Overlay for motor2 node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(motor3))
#error "Overlay for motor3 node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(motor4))
#error "Overlay for motor4 node not properly defined."
#endif

static const struct gpio_dt_spec motor_1 =
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor1), gpios);

static const struct gpio_dt_spec motor_2 =
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor2), gpios);

static const struct gpio_dt_spec motor_3 =
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor3), gpios);

static const struct gpio_dt_spec motor_4 =
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor4), gpios);


#if !DT_NODE_EXISTS(DT_PATH(motor_adc)) || \
	!DT_NODE_HAS_PROP(DT_PATH(motor_adc), io_channels)
#error "No suitable devicetree overlay specified"
#endif
static const struct adc_dt_spec motor_adc =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(motor_adc), 0);

using namespace chip;
using namespace ::chip::DeviceLayer;
using namespace ::chip::app::Clusters;

constexpr EndpointId kThermostatEndpoint = 1;

k_timer sStallTimer;

namespace
{

/* Convert and return only complete part of value to printable type */
uint8_t ReturnCompleteValue(int16_t Value)
{
	return static_cast<uint8_t>(Value / 100);
}

/* Converts and returns only reminder part of value to printable type.
 * This formula rounds reminder value to one significant figure
 */

uint8_t ReturnRemainderValue(int16_t Value)
{
	return static_cast<uint8_t>((Value % 100 + 5) / 10);
}

} // namespace


void motor_open_close(bool open_close){
	if(open_close){
		gpio_pin_set_dt(&motor_1, 1);
		gpio_pin_set_dt(&motor_2, 0);
		gpio_pin_set_dt(&motor_3, 1);
		gpio_pin_set_dt(&motor_4, 0);
	}
	else{
		gpio_pin_set_dt(&motor_1, 0);
		gpio_pin_set_dt(&motor_2, 1);
		gpio_pin_set_dt(&motor_3, 0);
		gpio_pin_set_dt(&motor_4, 1);
	}
}


CHIP_ERROR TemperatureManager::Init()
{
	app::DataModel::Nullable<int16_t> temp;

	if (!device_is_ready(motor_1.port) || !device_is_ready(motor_2.port) || !device_is_ready(motor_3.port) || !device_is_ready(motor_4.port)) {
		LOG_ERR("Motor pins not configured.");
		return CHIP_ERROR_NO_MEMORY;
	}

	gpio_pin_configure_dt(&motor_1, GPIO_OUTPUT);
	gpio_pin_configure_dt(&motor_2, GPIO_OUTPUT);
	gpio_pin_configure_dt(&motor_3, GPIO_OUTPUT);
	gpio_pin_configure_dt(&motor_4, GPIO_OUTPUT);
	open_closed = true;
	motor_open_close(open_closed);

	int err;
    if (!adc_is_ready_dt(&motor_adc)) {
        LOG_ERR("ADC controller device %s not ready", motor_adc.dev->name);
        return CHIP_ERROR_NO_MEMORY;
    }
    err = adc_channel_setup_dt(&motor_adc);
    if (err < 0) {
        LOG_ERR("Could not setup channel (%d)", err);
        return CHIP_ERROR_NO_MEMORY;
    }
    LOG_INF("Motor ADC initialized.");

	k_timer_init(&sStallTimer, TemperatureManager::StallTimerEventHandler, nullptr);
	k_timer_start(&sStallTimer, K_MSEC(200), K_FOREVER);

	PlatformMgr().LockChipStack();
	Thermostat::Attributes::LocalTemperature::Get(kThermostatEndpoint, temp);
	Thermostat::Attributes::OccupiedHeatingSetpoint::Get(kThermostatEndpoint, &mHeatingCelsiusSetPoint);
	Thermostat::Attributes::SystemMode::Get(kThermostatEndpoint, &mThermMode);
	PlatformMgr().UnlockChipStack();

	mCurrentTempCelsius = temp.Value();

	LogThermostatStatus();

	return CHIP_NO_ERROR;
}

void StallWorkHandle(struct k_work *work)
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
    motor_adc.dev->name,
    motor_adc.channel_id);

    if(adc_sequence_init_dt(&motor_adc, &sequence) != 0)
	{
		LOG_ERR("Problem");
	}

    err = adc_read(motor_adc.dev, &sequence);
    if (err < 0) {
        LOG_ERR("Could not read (%d)\n", err);
        return;
    }
    /*
    * If using differential mode, the 16 bit value
    * in the ADC sample buffer should be a signed 2's
    * complement value.
    */
    if (motor_adc.channel_cfg.differential) {
        val_mv = (int32_t)((int16_t)buf);
    } else {
        val_mv = (int32_t)buf;
    }
    LOG_INF("Measured voltage %d V", val_mv);
    err = adc_raw_to_millivolts_dt(&motor_adc, &val_mv);
    /* conversion to mV may not be supported, skip if not */
    if (err < 0) {
        LOG_ERR(" (value in mV not available)\n");
    } else {
        LOG_INF(" = %d mV", val_mv);
    }
	if(val_mv < 1330){
		gpio_pin_set_dt(&motor_1, 0);
		gpio_pin_set_dt(&motor_2, 0);
		gpio_pin_set_dt(&motor_3, 0);
		gpio_pin_set_dt(&motor_4, 0);
		LOG_INF("Stall detected.");
	}
	else{
		k_timer_start(&sStallTimer, K_MSEC(200), K_FOREVER);
	}
}

K_WORK_DEFINE(stall_work, StallWorkHandle);

void TemperatureManager::StallTimerEventHandler(k_timer *timer)
{
	k_work_submit(&stall_work);
}

void TemperatureManager::ContorlThermostat()
{
	if(mThermMode == 0x04){
		if(mCurrentTempCelsius < mHeatingCelsiusSetPoint){
			if(open_closed){
				open_closed = true;
				motor_open_close(open_closed);
				k_timer_start(&sStallTimer, K_MSEC(200), K_FOREVER);
			}
		}
		else{
			if(open_closed){
				open_closed = false;
				motor_open_close(open_closed);
				k_timer_start(&sStallTimer, K_MSEC(200), K_FOREVER);
			}
		}
	}
	else{
		if(open_closed){
			open_closed = false;
			motor_open_close(open_closed);
			k_timer_start(&sStallTimer, K_MSEC(200), K_FOREVER);
		}
	}
}

void TemperatureManager::LogThermostatStatus()
{
	LOG_INF("Thermostat:");
	LOG_INF("Mode - %d", GetMode());
	LOG_INF("Temperature - %d,%d'C", ReturnCompleteValue(GetCurrentTemp()), ReturnRemainderValue(GetCurrentTemp()));
	LOG_INF("HeatingSetpoint - %d,%d'C", ReturnCompleteValue(GetHeatingSetPoint()),
		ReturnRemainderValue(GetHeatingSetPoint()));
}

void TemperatureManager::AttributeChangeHandler(EndpointId endpointId, AttributeId attributeId, uint8_t *value,
						uint16_t size)
{
	switch (attributeId) {
	case Thermostat::Attributes::LocalTemperature::Id: {
		int16_t temp = *reinterpret_cast<int16_t *>(value);
		mCurrentTempCelsius = temp;
	} break;

	case Thermostat::Attributes::OccupiedHeatingSetpoint::Id: {
		int16_t heatingTemp = *reinterpret_cast<int16_t *>(value);
		mHeatingCelsiusSetPoint = heatingTemp;
		LOG_INF("Heating TEMP %d", mHeatingCelsiusSetPoint);
	} break;

	case Thermostat::Attributes::SystemMode::Id: {
		uint8_t mode = *value;
		mThermMode = mode;
		LOG_INF("Mode: %d", mThermMode);
	} break;

	default: {
		LOG_INF("Unhandled thermostat attribute %x", attributeId);
		return;
	} break;
	}

	LogThermostatStatus();
}

const uint8_t TemperatureManager::GetMode()
{
	return mThermMode;
}

const uint16_t TemperatureManager::GetCurrentTemp()
{
	return mCurrentTempCelsius;
}
const uint16_t TemperatureManager::GetHeatingSetPoint()
{
	return mHeatingCelsiusSetPoint;
}
