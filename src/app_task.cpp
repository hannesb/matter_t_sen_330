/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#define CONF_FLASH_SLEEP 1
#define CONF_SUSPEND_CONSOLE 0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "clusters/identify.h"
#include "lib/core/CHIPError.h"

#include <app-common/zap-generated/attributes/Accessors.h>

#ifdef CONFIG_BME280
#include <zephyr/drivers/sensor.h>

static const struct device *const bme280_dev = DEVICE_DT_GET_ANY(bosch_bme280);
#endif

#if CONF_FLASH_SLEEP
#if DT_NODE_HAS_STATUS(DT_NODELABEL(py25q64), okay)
static const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(py25q64));
static const struct device *const flash_bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(py25q64)));
#endif
#endif

#if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console))
static const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

namespace
{
constexpr chip::EndpointId kTemperatureSensorEndpointId = 1;
constexpr chip::EndpointId kHumiditySensorEndpointId = 2;

Nrf::Matter::IdentifyCluster sIdentifyClusterTemperature(kTemperatureSensorEndpointId);
Nrf::Matter::IdentifyCluster sIdentifyClusterHumidity(kHumiditySensorEndpointId);

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
#ifdef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
#define UAT_BUTTON_MASK DK_BTN3_MSK
#endif
#endif

#ifndef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
constexpr int kSoftwareUpdateTimeout = 1500;
constexpr int kUatTimeout = 1500;
constexpr int kFactoryResetTimeout = 3000;
constexpr int kUatBlinkPeriod = 200;
ButtonState sBtnState = ButtonState::None;
k_timer sBtn1Timer;
#endif

} /* namespace */

void HandleUAT()
{
#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
	LOG_INF("ICD UserActiveMode has been triggered.");
	Server::GetInstance().GetICDManager().OnNetworkActivity();
#endif
}
/* nRF54T15 TAG has only a single button,
 * therefore the default handler for factory data and software update had to be overriden.
 * On other DK's only UAT is handled by the application.
 */
void AppTask::ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged)
{
#ifdef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
	if (UAT_BUTTON_MASK & state & hasChanged) {
		HandleUAT();
	}
#else
	if (DK_BTN1_MSK & hasChanged) {
		if (DK_BTN1_MSK & state) {
			LOG_INF("Release the button within %ums to trigger Software Update", kSoftwareUpdateTimeout);
			k_timer_start(&sBtn1Timer, K_MSEC(kSoftwareUpdateTimeout), K_NO_WAIT);
			sBtnState = ButtonState::SoftwareUpdate;
		} else {
			if (sBtnState == ButtonState::SoftwareUpdate) {
#ifndef CONFIG_NCS_SAMPLE_MATTER_CUSTOM_BLUETOOTH_ADVERTISING
				if (Nrf::GetBoard().GetDeviceState() == Nrf::DeviceState::DeviceProvisioned) {
/* In this case we need to run only Bluetooth LE SMP advertising if it is available */
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
					GetDFUOverSMP().StartServer();
#else
					LOG_INF("Software update is disabled");
#endif
				} else {
					/* In this case we start both Bluetooth LE SMP and Matter advertising at the
					 * same time */
					Nrf::GetBoard().StartBLEAdvertisement();
				}
#endif
			} else if (sBtnState == ButtonState::UAT) {
				HandleUAT();
			}
			/* Restore LED's state and cancel the timer */
			k_timer_stop(&sBtn1Timer);
			sBtnState = ButtonState::None;
			Nrf::GetBoard().RestoreAllLedsState();
			Nrf::GetBoard().RunLedStateHandler();
		}
	}
#endif
}
#ifndef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
void ButtonTimerEventHandler()
{
	if (sBtnState == ButtonState::SoftwareUpdate) {
		LOG_INF("Release the button within %ums to trigger UAT", kUatTimeout);
		k_timer_start(&sBtn1Timer, K_MSEC(kUatTimeout), K_NO_WAIT);
		sBtnState = ButtonState::UAT;

		/* Turn off all LEDs before starting blink to make sure blink is coordinated. */
		Nrf::GetBoard().ResetAllLeds();
		Nrf::GetBoard().ForEachLED([](Nrf::LEDWidget &led) { led.Blink(kUatBlinkPeriod); });
	} else if (sBtnState == ButtonState::UAT) {
		LOG_INF("Factory reset has been triggered. Release button within %ums to cancel.",
			kFactoryResetTimeout);

		/* Start timer for sFactoryResetTimeout to allow user to cancel, if required. */
		k_timer_start(&sBtn1Timer, K_MSEC(kFactoryResetTimeout), K_NO_WAIT);
		sBtnState = ButtonState::None;

		Nrf::GetBoard().ForEachLED([](Nrf::LEDWidget &led) { led.Blink(Nrf::LedConsts::kBlinkRate_ms); });
		/* If we reached here, the button was held past FactoryResetTriggerTimeout, initiate factory reset */
	} else if (sBtnState == ButtonState::None) {
		/* Actually trigger Factory Reset */
		chip::Server::GetInstance().ScheduleFactoryReset();
	}
}

void ButtonTimerTimeoutCallback(k_timer *timer)
{
	Nrf::PostTask([] { ButtonTimerEventHandler(); });
}

#endif

void AppTask::UpdateMeasurement()
{
#ifdef CONFIG_BME280
	/* Real data from the onboard sensor*/
	pm_device_action_run(bme280_dev, PM_DEVICE_ACTION_RESUME);
	const int result_bme = sensor_sample_fetch(bme280_dev);

	if (result_bme == 0) {
		struct sensor_value sSensorValue;
		int resultT = sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &sSensorValue);
		if (resultT == 0) {
			// The MeasuredValue attribute is in 1/100ths of a degree Celsius.
			// First, get the temperature in 1/100ths of a degree.
			int32_t tmp = sSensorValue.val1 * 100 + sSensorValue.val2 / 10000;
			// Clamp to min..max
			if (tmp < -2000) {
				tmp = -2000;
			} else if (tmp > +8500) {
				tmp = +8500;
			}
#if 1
			// Reduce precision to 1/10ths of a degree, with 0.01° hysteresis
			// If no change, no packet will be send
			if (abs(mCurrentTemperature - tmp) >= 6) {
				mCurrentTemperature = (int16_t)((tmp + 5) / 10) * 10;
			}
#else
			mCurrentTemperature = (int16_t)tmp;
#endif			
			LOG_DBG("New temperature measurement: %d.%06d *C, attribute value: %d", sSensorValue.val1,
				sSensorValue.val2, mCurrentTemperature);

		} else {
			LOG_ERR("Getting temperature measurement data from BME280 failed with: %d", resultT);
		}
		int resultH = sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &sSensorValue);
		if (resultH == 0) {
			// The MeasuredValue attribute is in 1/100ths percent.
			// First, get the temperature in 1/100ths percent.
			int32_t tmp = sSensorValue.val1 * 100 + sSensorValue.val2 / 10000;
			// Clamp to min..max
			if (tmp < 0) {
				tmp = 0;
			} else if (tmp > +10000) {
				tmp = +10000;
			}
#if 1
			// Reduce precision to percent, with 0.1% hysteresis
			// If no change, no packet will be send
			if (abs(mCurrentHumidity - tmp) >= 60) {
				mCurrentHumidity = (int16_t)((tmp + 50) / 100) * 100;
			}
#else
			mCurrentHumidity = (int16_t)tmp;
#endif			
			LOG_DBG("New humidity measurement: %d.%06d %%, attribute value: %d", sSensorValue.val1,
				sSensorValue.val2, mCurrentHumidity);

		} else {
			LOG_ERR("Getting humidity measurement data from BME280 failed with: %d", resultH);
		}
	} else {
		LOG_ERR("Fetching data from bme280 sensor failed with: %d", result_bme);
	}
	pm_device_action_run(bme280_dev, PM_DEVICE_ACTION_SUSPEND);
#else
	/* Linear temperature increase that is wrapped around to min value after reaching the max value. */
	if (mCurrentTemperature < mTemperatureSensorMaxValue) {
		mCurrentTemperature += kTemperatureMeasurementStep;
	} else {
		mCurrentTemperature = mTemperatureSensorMinValue;
	}
#endif
}

void AppTask::UpdateMeasurementTimeoutCallback(k_timer *timer)
{
	if (!timer || !timer->user_data) {
		return;
	}

	DeviceLayer::PlatformMgr().ScheduleWork(
		[](intptr_t p) {
			AppTask::Instance().UpdateMeasurement();

			Protocols::InteractionModel::Status statusT =
				Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
					kTemperatureSensorEndpointId, AppTask::Instance().GetCurrentTemperature());

			if (statusT != Protocols::InteractionModel::Status::Success) {
				LOG_ERR("Updating temperature measurement failed %x", to_underlying(statusT));
			}
			Protocols::InteractionModel::Status statusH =
				Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
					kHumiditySensorEndpointId, AppTask::Instance().GetCurrentHumidity());

			if (statusH != Protocols::InteractionModel::Status::Success) {
				LOG_ERR("Updating humidity measurement failed %x", to_underlying(statusH));
			}
		},
		reinterpret_cast<intptr_t>(timer->user_data));
}

#if CONF_FLASH_SLEEP
/*
 * Put the external flash pins into deterministic, low-leakage states before
 * System OFF. These pin numbers are confirmed by the board pinctrl and DTS.
 */
static int configure_spi_pins_for_system_off(void)
{
	const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));
	int rc;

	if (!device_is_ready(gpio2)) {
		LOG_ERR("GPIO2 not ready.");
		return -ENODEV;
	}

	rc = gpio_pin_configure(gpio2, 5, GPIO_OUTPUT_HIGH);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_configure(gpio2, 0, GPIO_OUTPUT_HIGH);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_configure(gpio2, 3, GPIO_OUTPUT_HIGH);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_configure(gpio2, 1, GPIO_OUTPUT_LOW);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_configure(gpio2, 2, GPIO_OUTPUT_LOW);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_configure(gpio2, 4, GPIO_INPUT | GPIO_PULL_DOWN);
	if (rc < 0) {
		return rc;
	}

	return 0;
}

static int suspend_external_flash(void)
{
	int first_error = 0;
	int rc;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(py25q64), okay)
	if (device_is_ready(flash_dev)) {
		rc = pm_device_action_run(flash_dev, PM_DEVICE_ACTION_SUSPEND);
		if ((rc < 0) && (first_error == 0) && rc != -EALREADY) {
			first_error = rc;
			LOG_WRN("Warning: could not suspend external flash (%d)", rc);
		}
	} else {
		first_error = -ENODEV;
		LOG_WRN("Warning: flash device is not ready; skipping driver DPD.");
	}

	if (device_is_ready(flash_bus)) {
		rc = pm_device_action_run(flash_bus, PM_DEVICE_ACTION_SUSPEND);
		if ((rc < 0) && (first_error == 0) && rc != -EALREADY) {
			first_error = rc;
			LOG_WRN("Warning: could not suspend SPI bus (%d)", rc);
		}
	} else if (first_error == 0) {
		first_error = -ENODEV;
		LOG_WRN("Warning: flash SPI bus is not ready.");
	}
#else
	first_error = -ENODEV;
	LOG_WRN("Warning: py25q64 is not enabled in DTS.");
#endif

	rc = configure_spi_pins_for_system_off();
	if ((rc < 0) && (first_error == 0)) {
		first_error = rc;
		LOG_WRN("Warning: could not configure flash SPI pins (%d)", rc);
	}

	return first_error;
}
#endif

#if CONF_SUSPEND_CONSOLE
static int suspend_console_best_effort(void)
{
#if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console))
	if (!device_is_ready(cons)) {
		return -1;
	}
	int rc = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	if (rc < 0 && rc != -EALREADY) {
		return rc;
	}
	return 0;
#endif
}
#endif

CHIP_ERROR AppTask::Init()
{
#if CONF_FLASH_SLEEP
	int rc = suspend_external_flash();
	if (rc < 0) {
		LOG_WRN("Warning: flash low-power preparation incomplete (%d)", rc);
	}
#endif
#if CONF_SUSPEND_CONSOLE
	rc = suspend_console_best_effort();
	if (rc < 0) {
		LOG_WRN("Warning: could not suspend console (%d)", rc);
	}
#endif
	/* Initialize Matter stack */
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));
#ifdef CONFIG_BME280
	if (!device_is_ready(bme280_dev)) {
		LOG_ERR("BME280 sensor device not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}
	pm_device_action_run(bme280_dev, PM_DEVICE_ACTION_SUSPEND);	
#endif

	ReturnErrorOnFailure(sIdentifyClusterTemperature.Init());
	ReturnErrorOnFailure(sIdentifyClusterHumidity.Init());

	return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());
#if 0
	DataModel::Nullable<int16_t> val;
	Protocols::InteractionModel::Status status =
		Clusters::TemperatureMeasurement::Attributes::MinMeasuredValue::Get(kTemperatureSensorEndpointId, val);

	if (status != Protocols::InteractionModel::Status::Success || val.IsNull()) {
		LOG_ERR("Failed to get temperature measurement min value %x", to_underlying(status));
		return CHIP_ERROR_INCORRECT_STATE;
	}

	mTemperatureSensorMinValue = val.Value();

	status = Clusters::TemperatureMeasurement::Attributes::MaxMeasuredValue::Get(kTemperatureSensorEndpointId, val);

	if (status != Protocols::InteractionModel::Status::Success || val.IsNull()) {
		LOG_ERR("Failed to get temperature measurement max value %x", to_underlying(status));
		return CHIP_ERROR_INCORRECT_STATE;
	}

	mTemperatureSensorMaxValue = val.Value();
#endif
	k_timer_init(&mTimer, AppTask::UpdateMeasurementTimeoutCallback, nullptr);
	k_timer_user_data_set(&mTimer, this);
	k_timer_start(&mTimer, K_MSEC(kMeasurementIntervalMs), K_MSEC(kMeasurementIntervalMs));
#ifndef CONFIG_NCS_SAMPLE_MATTER_USE_DEFAULT_BUTTON_HANDLER
	k_timer_init(&sBtn1Timer, &ButtonTimerTimeoutCallback, nullptr);
#endif
	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
