/* HTTP2 Web Camera Client Device

   Part of WCWebCamServer project

   Copyright (c) 2022 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "defs.h"

#include "wcprotocol.h"
#include "http2_protoclient.h"
#include "wch2pcapp.h"

#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_camera.h"

const char *WC_TAG = "camhttp2-rsp";

/* h2pca */
static h2pca_config app_cfg;
static h2pca_status * h2pca_app;

/* io config */
#ifdef OUT_ENABLED
#define OUT_CNT 4
struct _out_state out_pins[OUT_CNT] = {
    { OUT_OFF, OUT_OFF, OUT_LED_FLASH },
    { OUT_ON,  OUT_ON,  OUT_LED }, // inverted
    { OUT_OFF, OUT_OFF, OUT_1 },
    { OUT_OFF, OUT_OFF, OUT_2 },
};
#endif

#ifdef INP_ENABLED
#define BUTTONS_CNT 2
// dont forget - all buttons are pull-upped
struct _button_state buttons[BUTTONS_CNT] = {
    { 0, BUTTON_1,  BUTTON_ACTIVE_HIGH, "btn1"},
    { 0, BUTTON_2,  BUTTON_ACTIVE_HIGH, "btn2"},
};
#endif

#ifdef ADC_ENABLED
static esp_adc_cal_characteristics_t *adc_chars;
static adc_channel_t adc_channel;
static SemaphoreHandle_t adc_mux = NULL;
volatile uint32_t adc_value = 0; // value in mV
static esp_timer_handle_t adc_probe;
#define ADC_PROBE_TIMER_DELTA 2000000
#endif

/* Frame size for snap */
#define CAM_SNAP_FRAMESIZE FRAMESIZE_SXGA
/* Frame size for streaming. must be less or equal CAM_SNAP_FRAMESIZE */
#define CAM_STREAM_FRAMESIZE FRAMESIZE_VGA

#define WC_SUB_PROTO "RAW_JPEG"

/* camera config - ov2640 driver */
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,   //PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = CAM_SNAP_FRAMESIZE, //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_LATEST,

    .fb_location = CAMERA_FB_IN_PSRAM
};

volatile int8_t cur_cam_mode = CAM_MODE_SNAP;

/* MSGS */
static const char * JSON_RPC_DOSNAP      =  "dosnap";
#ifdef ADC_ENABLED
static const char * JSON_RPC_GET_ADCVAL  =  "getadcval";
static const char * JSON_RPC_ADCVAL      =  "adcval";
#endif
#ifdef INP_ENABLED
static const char * JSON_RPC_BTN         =  "btn";
static const char * JSON_RPC_BTNEVENT    =  "btnevent";
#endif
#ifdef OUT_ENABLED
static const char * JSON_RPC_OUTPUT      =  "output";
static const char * JSON_RPC_LEVEL       =  "level";
static const char * JSON_RPC_PIN         =  "pin";
#endif

/* Modes in state-machina */
// add new frame to server. is need to send camera framebuffer
#define  MODE_SEND_FB               BIT10
#define  MODE_STREAM_NEXT_FRAME     BIT11
#ifdef ADC_ENABLED
// is need to get new voltage value
#define  MODE_ADC_PROBE             BIT12
#endif

/* timers */
#define STREAM_NEXT_FRAME_TIMER_DELTA 1000000
#define LOC_GET_MSG_TIMER_DELTA       5000000
#define LOC_SEND_MSG_TIMER_DELTA      5000000

/* forward decrlarations */
#ifdef ADC_ENABLED
uint32_t locked_get_adc_voltage();
#endif

static camera_fb_t * camera_take_pic() {
    camera_fb_t *pic = esp_camera_fb_get();

    // use pic->buf to access the image
    ESP_LOGI(WC_TAG, "Picture taken. Its size was: %zu bytes", pic->len);

    return pic;
}

static void send_snap() {

    camera_fb_t *pic = camera_take_pic();

    int res = h2pc_req_send_media_record_sync((char *) pic->buf, pic->len);

    esp_camera_fb_return(pic);

    if (res == ESP_OK)
        h2pca_locked_CLR_STATE(MODE_SEND_FB);
}

static void send_next_frame() {

    camera_fb_t *pic = camera_take_pic();

    // prepare path?query string

    h2pc_os_prepare_frame((char *) pic->buf, pic->len);

    if (!h2pc_get_is_streaming())
        h2pc_os_prepare(WC_SUB_PROTO);

    h2pc_os_wait_for_frame();

    esp_camera_fb_return(pic);

    if (h2pc_get_connected())
        h2pca_locked_CLR_STATE(MODE_STREAM_NEXT_FRAME);
}

bool on_incoming_msg(const cJSON * src, const cJSON * kind, const cJSON * iparams, const cJSON * msg_id) {
    char * src_s = src->valuestring;
    if (strcmp(src_s, h2pca_app->device_name) != 0) {

        if (kind) {
            char * msgk = kind->valuestring;
            cJSON * params = cJSON_CreateObject();
            if (msg_id)
                cJSON_AddNumberToObject(params, JSON_RPC_MID, msg_id->valuedouble);
            #ifdef ADC_ENABLED
            if (strcmp(JSON_RPC_GET_ADCVAL, msgk) == 0) {
                cJSON_AddNumberToObject(params, JSON_RPC_ADCVAL, (double) locked_get_adc_voltage());
                h2pc_om_add_msg_res(JSON_RPC_ADCVAL, src_s, params, true);
            } else
            #endif
            if (strcmp(JSON_RPC_DOSNAP, msgk) == 0) {
                h2pc_om_add_msg_res(JSON_RPC_DOSNAP, src_s, params, true);
                h2pca_locked_SET_STATE(MODE_SEND_FB);
            } else
            #ifdef OUT_ENABLED
            if (strcmp(JSON_RPC_OUTPUT, msgk) == 0) {
                if (iparams) {
                    cJSON * spin = cJSON_GetObjectItem(iparams,   JSON_RPC_PIN);   //selected pin
                    cJSON * slevel = cJSON_GetObjectItem(iparams, JSON_RPC_LEVEL); //level value
                    bool ok;
                    if (spin && slevel) {
                        uint8_t pinv, levelv;
                        pinv = (uint8_t) spin->valueint;
                        levelv = (uint8_t) slevel->valueint;
                        board_out_operation(pinv, levelv);
                        ok = true;
                    } else {
                        ok = false;
                    }
                    h2pc_om_add_msg_res(JSON_RPC_OUTPUT, src_s, params, ok);
                } else {
                    h2pc_om_add_msg_res(JSON_RPC_OUTPUT, src_s, params, false);
                }
            } else
            #endif
            {
                // you should delete params - no msg is sended
                cJSON_Delete(params);
            }
        }
    }

    return true;
}

static esp_err_t init_camera()
{
    //initialize the camera
    //
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(WC_TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

static esp_err_t set_camera_buffer_size(int8_t  acam_mode)
{
    if (acam_mode != cur_cam_mode) {
        cur_cam_mode = acam_mode;
        uint8_t fsz;
        switch (cur_cam_mode)
        {
        case CAM_MODE_SNAP:
            fsz = CAM_SNAP_FRAMESIZE;
            break;
        case CAM_MODE_STREAM:
            fsz = CAM_STREAM_FRAMESIZE;
            break;
        default:
            fsz = 0;
            break;
        }
        if (fsz > 0) {
            //do change framesize
            return esp_camera_set_framesize(fsz);
        }
    }
    return 0;
}

#ifdef INP_ENABLED
static void button_tap_cb(void* arg)
{
    ESP_LOGI(WC_TAG, "tap cb (%s)", (char *)arg);

    /* find the pressed button and react */
    for (int i = 0; i < BUTTONS_CNT; i++) {
        if (strcmp(buttons[i].arg, (char *)arg) == 0) {
            /* react here */
            cJSON * params = cJSON_CreateObject();
            cJSON_AddStringToObject(params, JSON_RPC_BTN, arg);
            h2pc_om_add_msg_res(JSON_RPC_BTNEVENT, "", params, true); // params owned by msg now
            return;
        }
    }
}

static void board_buttons_init(void)
{
    for (int i = 0; i < BUTTONS_CNT; i++) {
        buttons[i].handle = iot_button_create(buttons[i].pin, buttons[i].active_level);
        if (buttons[i].handle)
            iot_button_set_evt_cb(buttons[i].handle, BUTTON_CB_PUSH, button_tap_cb, buttons[i].arg);
    }
}
#endif

#ifdef OUT_ENABLED
void board_out_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < OUT_CNT; i++) {
        if (out_pins[i].pin != pin) {
            continue;
        }
        if (onoff != out_pins[i].previous) {
            gpio_set_level(pin, onoff);
            out_pins[i].previous = onoff;
        }
        return;
    }
    ESP_LOGE(WC_TAG, "Out is not found!");
}

static void board_out_init(void)
{
    for (int i = 0; i < OUT_CNT; i++) {
        gpio_pad_select_gpio(out_pins[i].pin);
        gpio_set_direction(out_pins[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(out_pins[i].pin, OUT_OFF);
        out_pins[i].previous = OUT_OFF;
    }
}
#endif

#ifdef ADC_ENABLED

static void board_adc_init(void)
{
    switch ( ADC_PIN )
    {
        case GPIO_NUM_2:
            adc_channel = 2;
            break;
        case GPIO_NUM_12:
            adc_channel = 5;
            break;
        case GPIO_NUM_13:
            adc_channel = 4;
            break;
        case GPIO_NUM_14:
            adc_channel = 6;
            break;
        case GPIO_NUM_15:
            adc_channel = 3;
            break;
    }
    adc_mux = xSemaphoreCreateMutex();
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_value_t val_type =
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, adc_chars);
    adc2_config_channel_atten((adc2_channel_t)adc_channel, ADC_ATTEN_DB_0);
}

uint32_t board_get_adc_mV(void) {
    int adc_new_value = 0;
    //Multisampling
    for (int i = 0; i < ADC_NO_OF_SAMPLES; i++) {
        int raw;
        adc2_get_raw((adc2_channel_t)adc_channel, ADC_WIDTH_BIT_12, &raw);
        adc_new_value += raw;
    }
    adc_new_value /= ADC_NO_OF_SAMPLES;
    if (xSemaphoreTake(adc_mux, portMAX_DELAY) == pdTRUE) {
        adc_value = adc_new_value;
        xSemaphoreGive(adc_mux);
    }
    h2pca_locked_CLR_STATE(MODE_ADC_PROBE);
    return ( adc_new_value );
}

uint32_t locked_get_adc_voltage() {
    uint32_t v = 0;
    if (xSemaphoreTake(adc_mux, portMAX_DELAY) == pdTRUE) {
        v = adc_value;
        xSemaphoreGive(adc_mux);
    }
    return v;
}

static void sync_adc_probe_task_cb(h2pca_task_id id,
                                     h2pca_state cur_state,
                                     void * user_data,
                                     uint32_t * restart_period) {
    board_get_adc_mV();
}

#endif

static void sync_stream_task_cb(h2pca_task_id id,
                                     h2pca_state cur_state,
                                     void * user_data,
                                     uint32_t * restart_period) {
    ESP_ERROR_CHECK(set_camera_buffer_size(CAM_MODE_STREAM));
    esp_camera_do_snap();
    send_next_frame();
}

static void on_step_finished() {
    if (h2pca_locked_CHK_STATE(AUTHORIZED_BIT|MODE_SEND_FB)) {
        /* send framebuffer */
        ESP_ERROR_CHECK(set_camera_buffer_size(CAM_MODE_SNAP));
        esp_camera_do_snap();
        vTaskDelay(500);
        send_snap();
        ESP_ERROR_CHECK(set_camera_buffer_size(CAM_MODE_STREAM));
    }
}

static void on_ble_cfg_finished() {
    /* intialize io */
    #ifdef INP_ENABLED
    board_buttons_init();
    #endif
    #ifdef OUT_ENABLED
    board_out_init();
    #endif
    #ifdef ADC_ENABLED
    board_adc_init();
    #endif
}

void app_main()
{
    esp_err_t err = h2pca_init_cfg(&app_cfg);
    ESP_ERROR_CHECK(err);

    app_cfg.LOG_TAG = WC_TAG;

    app_cfg.h2pcmode = H2PC_MODE_MESSAGING|H2PC_MODE_OUTGOING;
    app_cfg.recv_msgs_period = LOC_GET_MSG_TIMER_DELTA;
    app_cfg.send_msgs_period = LOC_SEND_MSG_TIMER_DELTA;
    app_cfg.inmsgs_proceed_chunk = 16;

    app_cfg.on_ble_cfg_finished = &on_ble_cfg_finished;
    app_cfg.on_next_inmsg = &on_incoming_msg;
    app_cfg.on_finish_step = &on_step_finished;

    h2pca_task * tsk;

    tsk = h2pca_new_task("Streaming", 1, NULL, &err);
    ESP_ERROR_CHECK(err);
    tsk->on_sync = &sync_stream_task_cb;
    tsk->apply_bitmask = MODE_STREAM_NEXT_FRAME;
    tsk->req_bitmask = AUTHORIZED_BIT;
    tsk->period = STREAM_NEXT_FRAME_TIMER_DELTA;
    ESP_ERROR_CHECK(h2pca_task_pool_add_task(&(app_cfg.tasks), tsk));

    #ifdef ADC_ENABLED
    esp_timer_start_periodic(adc_probe, ADC_PROBE_TIMER_DELTA);
    tsk = h2pca_new_task("ADC", 2, NULL, &err);
    ESP_ERROR_CHECK(err);
    tsk->on_sync = &sync_adc_probe_task_cb;
    tsk->apply_bitmask = MODE_ADC_PROBE;
    tsk->req_bitmask = AUTHORIZED_BIT;
    tsk->period = ADC_PROBE_TIMER_DELTA;
    ESP_ERROR_CHECK(h2pca_task_pool_add_task(&(app_cfg.tasks), tsk));
    #endif

    h2pca_app = h2pca_init(&app_cfg, &err);

    if (h2pca_app == NULL) {
        ESP_LOGE(WC_TAG, "Application not initialized, error code %d", err);
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK( init_camera() );

    /* start main task */
    h2pca_start(0);
}
