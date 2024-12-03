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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_mac.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "esp_task_wdt.h"
#include "multi_button.h"

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

#define HID_DEMO_TAG "HID_DEMO"
#define GATT_TAG "GATT_SERVICE"

#define GATTS_SERVICE_UUID 0x00FF
#define GATTS_CHAR_UUID 0xFF01
#define GATTS_NUM_HANDLE 4 // 服务句柄数量, 服务句柄、特征句柄、描述符句柄、描述符配置句柄, 4个句柄

static uint8_t gatt_char_value[64] = "Default_GATT_Value";
static uint16_t gatt_handle_table[GATTS_NUM_HANDLE];
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
static esp_bd_addr_t connected_bd_addr; // 用于存储连接设备的地址

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "Freedorm Lite"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x12,
    0x18,
    0x00,
    0x00,
};

#define BUTTON1_ID 0

struct Button btn1;

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x0180,   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // Always set the discoverable mode and BLE only
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define MAX_WHITELIST_SIZE 10
typedef struct
{
    uint8_t num_devices;
    esp_bd_addr_t devices[MAX_WHITELIST_SIZE];
} whitelist_t;

static whitelist_t whitelist;
static bool pairing_mode = false;
static esp_bd_addr_t connected_bd_addr;
static SemaphoreHandle_t pairing_semaphore;

#define PAIRING_BUTTON_GPIO GPIO_NUM_0 // 修改为您的按键GPIO编号
#define OUTPUT_LED GPIO_NUM_12

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void pairing_mode_task(void *arg);

void flip_led(int gpio_num)
{
    // verify the gpio is in the gpio_num_t
    if (gpio_num < GPIO_NUM_0 || gpio_num >= GPIO_NUM_MAX)
    {
        ESP_LOGE(HID_DEMO_TAG, "Invalid GPIO number");
        return;
    }

    ESP_LOGI("LED_FLIP", "Toggling GPIO %d", gpio_num);
    ESP_LOGI("LED_FLIP", "Current GPIO %d level: %d", gpio_num, gpio_get_level(gpio_num));

    gpio_get_level(gpio_num) ? gpio_set_level(gpio_num, 0) : gpio_set_level(gpio_num, 1);

    ESP_LOGI("LED_FLIP", "New GPIO %d level: %d", gpio_num, gpio_get_level(gpio_num));
}

uint8_t read_button_GPIO(uint8_t button_id)
{
    // ESP_LOGI(HID_DEMO_TAG, "Reading GPIO %d ,LEVEL: %d", PAIRING_BUTTON_GPIO, gpio_get_level(PAIRING_BUTTON_GPIO));
    return gpio_get_level(PAIRING_BUTTON_GPIO);
}

void button_task(void *arg)
{
    while (1)
    {
        button_ticks();                // 按键状态检测
        vTaskDelay(pdMS_TO_TICKS(10)); // 每 5 毫秒调用一次
    }
}

void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(HID_DEMO_TAG, "Single click detected");
    flip_led(OUTPUT_LED); // 翻转 LED 状态
    // 进入配对模式
}

void BTN1_LONG_PRESS_HOLD_Handler(void *btn)
{
    ESP_LOGI(HID_DEMO_TAG, "Long press hold detected");
    // pairing_mode = true;
    // xSemaphoreGive(pairing_semaphore);
    // 长按启动后的逻辑
}

/* 加载白名单 */
esp_err_t load_whitelist_from_nvs(whitelist_t *list)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;

    size_t required_size = sizeof(whitelist_t);
    err = nvs_get_blob(nvs_handle, "whitelist", list, &required_size);
    nvs_close(nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        // 白名单不存在，初始化
        list->num_devices = 0;
        memset(list->devices, 0, sizeof(list->devices));
        return ESP_OK;
    }
    return err;
}

/* 保存白名单 */
esp_err_t save_whitelist_to_nvs(whitelist_t *list)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(nvs_handle, "whitelist", list, sizeof(whitelist_t));
    if (err != ESP_OK)
    {
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

esp_err_t delete_whitelist_from_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_erase_key(nvs_handle, "whitelist");
    if (err != ESP_OK)
    {
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}
/* 配对模式任务 */
void pairing_mode_task(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(pairing_semaphore, portMAX_DELAY) == pdTRUE)
        {
            // 进入配对模式
            pairing_mode = true;
            ESP_LOGI(HID_DEMO_TAG, "Entering pairing mode");
            // 设置广播参数，允许所有设备连接
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&hidd_adv_params);
            // 配对模式持续60秒
            vTaskDelay(6000 / portTICK_PERIOD_MS);
            // 退出配对模式
            pairing_mode = false;
            ESP_LOGI(HID_DEMO_TAG, "Exiting pairing mode");
            // 设置广播参数，只允许白名单设备连接
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&hidd_adv_params);
        }
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
    {
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG:
    {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
    {
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
    {
        sec_conn = false;
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
    {
        ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
    {
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
        ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
        break;
    }
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
            ESP_LOGD(HID_DEMO_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        memcpy(connected_bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t)); // 保存连接设备地址
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (param->ble_security.auth_cmpl.success)
        {
            memcpy(connected_bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t)); // 保存连接设备地址
            // 检查设备是否已在白名单中
            bool device_in_whitelist = false;
            for (int i = 0; i < whitelist.num_devices; i++)
            {
                if (memcmp(whitelist.devices[i], connected_bd_addr, sizeof(esp_bd_addr_t)) == 0)
                {
                    device_in_whitelist = true;
                    break;
                }
            }
            if (!device_in_whitelist)
            {
                if (whitelist.num_devices < MAX_WHITELIST_SIZE)
                {
                    memcpy(whitelist.devices[whitelist.num_devices], connected_bd_addr, sizeof(esp_bd_addr_t));
                    whitelist.num_devices++;
                    save_whitelist_to_nvs(&whitelist);
                    // 添加设备到控制器的白名单
                    esp_ble_gap_update_whitelist(true, connected_bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
                    ESP_LOGI(HID_DEMO_TAG, "Device added to whitelist");
                }
                else
                {
                    ESP_LOGW(HID_DEMO_TAG, "Whitelist is full");
                }
            }
            // 如果不在配对模式，修改广播参数为只允许白名单设备
            if (!pairing_mode)
            {
                hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
                esp_ble_gap_stop_advertising();
                esp_ble_gap_start_advertising(&hidd_adv_params);
            }
            esp_ble_gap_read_rssi(connected_bd_addr); // 读取 RSSI 值
        }
        else
        {
            ESP_LOGE(HID_DEMO_TAG, "Authentication failed, reason: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: // 处理读取 RSSI 的回调
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(HID_DEMO_TAG, "RSSI for connected device: %d", param->read_rssi_cmpl.rssi);
            // 在此根据 RSSI 值执行开门操作
        }
        else
        {
            ESP_LOGE(HID_DEMO_TAG, "Failed to read RSSI, status: %d", param->read_rssi_cmpl.status);
        }
        break;

    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn)
        {
            ESP_LOGI(HID_DEMO_TAG, "Read RSSI value");
            send_volum_up = true;
            // uint8_t key_vaule = {HID_KEY_A};
            // esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (send_volum_up)
            {
                esp_ble_gap_read_rssi(connected_bd_addr); // 读取 RSSI 值
                send_volum_up = false;
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
                // vTaskDelay(3000 / portTICK_PERIOD_MS);
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);

                // // send keyboard value "Hello Freedorm !"
                // uint8_t key_vaule[] = {HID_KEY_H, HID_KEY_E, HID_KEY_L, HID_KEY_L, HID_KEY_O, HID_KEY_SPACEBAR, HID_KEY_F, HID_KEY_R, HID_KEY_E, HID_KEY_E, HID_KEY_D, HID_KEY_O, HID_KEY_R, HID_KEY_M};
                // esp_hidd_send_keyboard_value(hid_conn_id, 0, key_vaule, sizeof(key_vaule));
            }
        }
    }
}

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

    // 加载白名单
    if (load_whitelist_from_nvs(&whitelist) == ESP_OK)
    {
        ESP_LOGI(HID_DEMO_TAG, "Whitelist loaded, num_devices: %d", whitelist.num_devices);
        for (int i = 0; i < whitelist.num_devices; i++)
        {
            esp_ble_gap_update_whitelist(true, whitelist.devices[i], BLE_WL_ADDR_TYPE_PUBLIC);
            ESP_LOGI(HID_DEMO_TAG, "Device %d: %08x%04x", i, (whitelist.devices[i][0] << 24) + (whitelist.devices[i][1] << 16) + (whitelist.devices[i][2] << 8) + whitelist.devices[i][3], (whitelist.devices[i][4] << 8) + whitelist.devices[i][5]);
        }
    }

    // 设置广播参数
    if (whitelist.num_devices == 0)
    {
        hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    }
    else
    {
        hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
    }

    // 注册回调函数
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* 设置安全参数 */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // 按键初始化
    gpio_config_t input_io_conf = {};
    input_io_conf.intr_type = GPIO_INTR_POSEDGE;
    input_io_conf.mode = GPIO_MODE_INPUT;
    input_io_conf.pin_bit_mask = (1ULL << PAIRING_BUTTON_GPIO);
    input_io_conf.pull_up_en = 1;
    gpio_config(&input_io_conf);

    // 当按下GPIO0时，使GPIO IO12 的 LED亮起
    gpio_config_t output_io_conf = {
        .pin_bit_mask = (1ULL << OUTPUT_LED),  // 配置 GPIO2 为输出
        .mode = GPIO_MODE_OUTPUT,              // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 不需要上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 不需要下拉
        .intr_type = GPIO_INTR_DISABLE         // 不需要中断
    };
    gpio_config(&output_io_conf);

    // 初始化按键对象
    button_init(&btn1, read_button_GPIO, 1, 0); // 第三个参数为有效电平 0（低电平有效），第四个参数为按键 ID 艹，tmd debug半天结果是这里参考电平的问题，艹

    button_attach(&btn1, SINGLE_CLICK, BTN1_SINGLE_CLICK_Handler);
    button_attach(&btn1, LONG_PRESS_START, BTN1_LONG_PRESS_HOLD_Handler);
    button_start(&btn1);

    xTaskCreate(&button_task, "button_task", 2048, NULL, 6, NULL);

    pairing_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(&pairing_mode_task, "pairing_mode_task", 2048, NULL, 5, NULL);

    // xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
