#ifndef BUTTON_H
#define BUTTON_H

#include "button.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "multi_button.h"
#include "esp_mac.h"

#define PAIRING_BUTTON_GPIO GPIO_NUM_0 // 修改为您的按键GPIO编号

typedef enum
{
    BUTTON_SINGLE_CLICK = 0,
    BUTTON_DOUBLE_CLICK,
    BUTTON_LONG_PRESS,
    BUTTON_SINGLE_CLICK_THEN_LONG_PRESS,

} button_state_t;

extern uint32_t led_state_mask; // 在这里初始化，位图，记录每个 GPIO 的当前状态

// 函数声明

uint8_t read_button_GPIO(uint8_t button_id);
void freedorm_button_init();
void button_task(void *arg);

void BTN1_PRESS_REPEAT_Handler(void *btn);
void BTN1_SINGLE_CLICK_Handler(void *btn);
void BTN1_DOUBLE_CLICK_Handler(void *btn);
void BTN1_LONG_PRESS_START_Handler(void *btn);
void BTN1_LONG_PRESS_HOLD_Handler(void *btn);

#endif
