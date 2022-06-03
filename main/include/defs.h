#ifndef DEFS_H_
#define DEFS_H_

#include "iot_button.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//#define LOG_DEBUG

extern const char * WC_TAG;

extern const char const DEVICE_NAME [];

#define ADC_ENABLED
#define OUT_ENABLED
#define INP_ENABLED

#ifdef OUT_ENABLED
#define OUT_LED         GPIO_NUM_33
#define OUT_LED_FLASH   GPIO_NUM_4
#define OUT_1           GPIO_NUM_2
#define OUT_2           GPIO_NUM_14

#define OUT_ON  1
#define OUT_OFF 0

struct _out_state {
    uint8_t current;
    uint8_t previous;
    uint8_t pin;
};

void board_out_operation(uint8_t pin, uint8_t onoff);

#endif

#ifdef INP_ENABLED
#define BUTTON_1        GPIO_NUM_12
#define BUTTON_2        GPIO_NUM_13

struct _button_state {
    button_handle_t handle;
    uint8_t pin;
    button_active_t active_level;
    char * arg;
};

#endif

#ifdef ADC_ENABLED
#define ADC_PIN         GPIO_NUM_15
#define ADC_NO_OF_SAMPLES   4
uint32_t board_get_adc_mV(void);
#endif


/* esp32-cam pins description */
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif
