/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "board/board.h"

#include <platform/CHIPDeviceLayer.h>

struct Identify;

#ifndef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
enum class ButtonState { None, SoftwareUpdate, UAT };
#endif

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

	/* Defined by cluster temperature measured value = 100 x temperature in degC with resolution of
	 * 0.01 degC. */
	void UpdateMeasurement();

	int16_t GetCurrentTemperature() const { return mCurrentTemperature; }
	int16_t GetCurrentHumidity() const { return mCurrentHumidity; }

private:
	CHIP_ERROR Init();
	k_timer mTimer;

	static constexpr uint16_t kMeasurementIntervalMs = 15000; /* 15 seconds */
	static constexpr uint16_t kMeasurementStep = 100; /* 1 degree Celsius */

	static void UpdateMeasurementTimeoutCallback(k_timer *timer);

	static void ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged);

	int16_t mCurrentTemperature = 0;
	int16_t mCurrentHumidity = 0;
};
