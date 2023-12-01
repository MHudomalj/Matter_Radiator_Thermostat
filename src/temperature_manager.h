/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_event.h"

#include <app-common/zap-generated/attributes/Accessors.h>

#include <lib/core/CHIPError.h>

using namespace chip;

class TemperatureManager {
public:
	static TemperatureManager &Instance()
	{
		static TemperatureManager sTemperatureManager;
		return sTemperatureManager;
	};

	CHIP_ERROR Init(void);
	void AttributeChangeHandler(EndpointId endpointId, AttributeId attributeId, uint8_t *value, uint16_t size);
	const uint8_t GetMode(void);
	const uint16_t GetCurrentTemp(void);
	const uint16_t GetHeatingSetPoint(void);

	void LogThermostatStatus(void);

private:
	void ContorlThermostat(void);
	static void StallTimerEventHandler(k_timer *timer);
	int16_t mCurrentTempCelsius;
	int16_t mHeatingCelsiusSetPoint;
	uint8_t mThermMode;
	bool open_closed;
};
