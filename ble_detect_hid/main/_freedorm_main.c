/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_task_wdt.h"
#include "multi_button.h"
#include "button.h"
#include "ble_module.h"
#include "ws2812b_led.h"
#include "lock_control.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define BUTTON1_ID 0

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // 按键初始化
    uint64_t gpio_output_sel = (1ULL << OUTPUT_LED_D4) | (1ULL << OUTPUT_LED_D5) | (1ULL << CTL_LOCK) | (1ULL << CTL_D0);

    gpio_config_t output_io_conf = {
        .pin_bit_mask = gpio_output_sel,
        .mode = GPIO_MODE_OUTPUT,              // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 不需要上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 不需要下拉
        .intr_type = GPIO_INTR_DISABLE         // 不需要中断
    };
    gpio_config(&output_io_conf);

    ble_module_init();
    ws2812b_led_init(); // 按键在之后初始化，因为按键依赖ws2812b的队列
    freedorm_button_init();
    lock_control_init();
    xTaskCreate(&button_task, "button_task", 2048, NULL, 1, NULL);
}
