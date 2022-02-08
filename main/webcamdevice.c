/* HTTP2 Web Camera Client Device

   Part of WCWebCamServer project

   Copyright (c) 2022 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include "defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "http2_protoclient.h"
#include "esp_event_loop.h"
#include "lwip/apps/sntp.h"
#include "driver/gpio.h"
#ifdef LOG_DEBUG
#include "esp_log.h"
#endif
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include <cJSON.h>

const char *WC_TAG = "camhttp2-rsp";

/* mac address for device */
static char mac_str[13];
const char UPPER_XDIGITS[] = "0123456789ABCDEF";

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

    .pixel_format = PIXFORMAT_JPEG, //PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_XGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* wifi config */
#define APP_WIFI_SSID CONFIG_WIFI_SSID
#define APP_WIFI_PASS CONFIG_WIFI_PASSWORD

/* HTTP2 HOST config */
// The HTTP/2 server to connect to
#define HTTP2_SERVER_URI   CONFIG_SERVER_URI
// The user's name
#define HTTP2_SERVER_NAME  CONFIG_SERVER_NAME
// The user's password
#define HTTP2_SERVER_PASS  CONFIG_SERVER_PASS

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
/* Events for the event group: */
//are we connected to the AP with an IP?
const int WIFI_CONNECTED_BIT = BIT0;

/* Commands */
#define HTTP2_STREAMING_AUTH_PATH     "/authorize.json"
#define HTTP2_STREAMING_ADDREC_PATH   "/addRecord.json?shash="
#define HTTP2_STREAMING_GETMSGS_PATH  "/getMsgs.json"
#define HTTP2_STREAMING_ADDMSGS_PATH  "/addMsgs.json"

/* JSON-RPC fields */
static const char * JSON_RPC_OK      = "OK";
static const char * JSON_RPC_BAD     = "BAD";
static const char * JSON_RPC_MSG     = "msg";
static const char * JSON_RPC_MSGS    =  "msgs";
static const char * JSON_RPC_RESULT  =  "result";
static const char * JSON_RPC_NAME    =  "name";
static const char * JSON_RPC_PASS    =  "pass";
static const char * JSON_RPC_SHASH   =  "shash";
static const char * JSON_RPC_META    =  "meta";
static const char * JSON_RPC_REC     =  "record";
static const char * JSON_RPC_STAMP   =  "stamp";
static const char * JSON_RPC_RID     =  "rid";
static const char * JSON_RPC_MID     =  "mid";
static const char * JSON_RPC_DEVICE  =  "device";
static const char * JSON_RPC_TARGET  =  "target";
static const char * JSON_RPC_PARAMS  =  "params";


/* MSGS */
static const char * JSON_RPC_SYNC        =  "sync";
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
// mode undefined
#define MODE_NONE 0x0000
// connection step. is camera need to connect to server
#define MODE_CONN 0x0001
// autorization step. is camera need to authorize
#define MODE_AUTH 0x0002
// add new frame to server. is need to send camera framebuffer
#define MODE_SEND_FB  0x0004
// get messages from server. is need to get messages
#define MODE_GET_MSG  0x0008
// send sync msg. is need to send sync msg to server
#define MODE_SYNC 0x0010
// send msg. is need to send msg from pool to server
#define MODE_SEND_MSG 0x0020
#ifdef ADC_ENABLED
// is need to get new voltage value
#define MODE_ADC_PROBE 0x0100
#endif

/* global vars */
static char * sid = NULL;                   // current session id
volatile int  connect_errors = 0;           // connection failed tryes count
volatile int  protocol_errors = 0;          // protocol errors count
static char * sync_msg = NULL;              // cached sync msg
static char * sync_stamp = NULL;            // cached sync msg with sid
static char * last_stamp = NULL;            // last time stamp from server
volatile int incoming_msgs_pos = 0;         // helpers work with the pool of incoming msgs
volatile int incoming_msgs_size = 0;

/* state routes */
volatile uint16_t http2_state = MODE_NONE;  // client state mask
static SemaphoreHandle_t states_mux = NULL; // state mask locker for multi-thread access
// bit operations with state mask
#define SET_STATE(astate) http2_state |= (uint16_t) astate
#define CLR_STATE(astate) http2_state &= ~((uint16_t) astate)
#define CLR_ALL_STATES http2_state = MODE_NONE
#define CHK_STATE(astate) (http2_state & (uint16_t) astate)

/* thread-safe get states route */
static uint16_t locked_GET_STATES() {
    uint16_t val;
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        val =  http2_state;
        xSemaphoreGive(states_mux);
    } else val = MODE_NONE;
    return val;
}

/* thread-safe check state route */
static bool locked_CHK_STATE(uint16_t astate) {
    bool val;
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        val =  CHK_STATE(astate);
        xSemaphoreGive(states_mux);
    } else val = false;
    return val;
}

/* thread-safe set state route */
static void locked_SET_STATE(uint16_t astate) {
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        SET_STATE(astate);
        xSemaphoreGive(states_mux);
    }
}

/* thread-safe clear state route */
static void locked_CLR_STATE(uint16_t astate) {
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        CLR_STATE(astate);
        xSemaphoreGive(states_mux);
    }
}

/* thread-safe clear all states route */
static void locked_CLR_ALL_STATES() {
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        CLR_ALL_STATES;
        xSemaphoreGive(states_mux);
    }
}

/* timers */
static esp_timer_handle_t frame_send;
static esp_timer_handle_t msgs_get;
static esp_timer_handle_t msgs_send;
#define GET_MSG_TIMER_DELTA 5000000
#define SEND_MSG_TIMER_DELTA 5000000
#define SEND_FRAME_TIMER_DELTA 60000000

/* forward decrlarations */
void finalize_app();
void add_outgoing_msg(char * amsg, char * atarget, cJSON * content);
#ifdef ADC_ENABLED
uint32_t locked_get_adc_voltage();
#endif

static void set_time(void)
{
    struct timeval tv = {
        .tv_sec = 1509449941,
    };
    struct timezone tz = {
        0, 0
    };
    settimeofday(&tv, &tz);

    /* Start SNTP service */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();
}

/* encode sid to percent-string */
static void encode_sid(char * dst) {
    if (!sid) return 0;
    int p =0;
    for (int i = 0; i < strlen(sid); i++) {
        char c = sid[i];
        if ( ((c >= 48) && (c <= 57)) ||
             ((c >= 65) && (c <= 90)) ||
             ((c >= 97) && (c <= 122)) ) {
            dst[p++] = c;
            continue;
        }

        dst[p++] = '%';
        dst[p++] = UPPER_XDIGITS[(c >> 4) & 0x0f];
        dst[p++] = UPPER_XDIGITS[(c & 0x0f)];
    }
    dst[p] = 0;
}

/* disconnect from host. reset all states */
static void disconnect_host() {
    if (locked_CHK_STATE(MODE_CONN)) h2pc_disconnect_http2();
    locked_CLR_ALL_STATES();
    h2pc_reset_buffers();
    protocol_errors = 0;
}

/* connect to host */
static void connect_to_http2() {
    disconnect_host();

    if (h2pc_connect_to_http2(HTTP2_SERVER_URI)) {
        connect_errors = 0;
        locked_SET_STATE(MODE_CONN | MODE_AUTH);
    } else
        connect_errors++;
}

static void send_authorize() {
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Trying to authorize");
    #endif

    if (sid) {
        cJSON_free(sid);
        sid = NULL;
    }
    /* HTTP GET SID */
    cJSON * tosend = cJSON_CreateObject();
    cJSON_AddStringToObject(tosend, JSON_RPC_NAME, HTTP2_SERVER_NAME);
    cJSON_AddStringToObject(tosend, JSON_RPC_PASS, HTTP2_SERVER_PASS);
    cJSON_AddStringToObject(tosend, JSON_RPC_DEVICE, mac_str);
    h2pc_prepare_to_send(tosend);
    cJSON_Delete(tosend);
    h2pc_do_post(HTTP2_STREAMING_AUTH_PATH);
    h2pc_wait_for_response();
    /* extract sid */
    cJSON * resp = h2pc_consume_response_content();
    if (resp) {
        cJSON * shash = cJSON_GetObjectItem(resp, JSON_RPC_SHASH);
        if (shash) {
            char * hash = shash->valuestring;
            sid = cJSON_malloc(strlen(hash) + 1);
            strcpy(sid, hash);
            locked_CLR_STATE(MODE_AUTH);
            locked_SET_STATE(MODE_GET_MSG);
            #ifdef LOG_DEBUG
            ESP_LOGI(WC_TAG, "hash=%s",sid);
            #endif
            if (sync_msg) {
                cJSON_free(sync_msg);
                sync_msg = NULL;
            }
            cJSON * tosend = cJSON_CreateObject();
            cJSON_AddStringToObject(tosend, JSON_RPC_SHASH, sid);
            cJSON_AddStringToObject(tosend, JSON_RPC_MSG, JSON_RPC_SYNC);
            sync_msg = cJSON_PrintUnformatted(tosend);
            cJSON_Delete(tosend);

            strcpy(last_stamp, sync_stamp);
        } else
            protocol_errors++; // some server error
        cJSON_Delete(resp);
    }
}

static void send_framebuffer() {
    camera_fb_t *pic = esp_camera_fb_get();
    // use pic->buf to access the image
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Picture taken. Its size was: %zu bytes", pic->len);
    #endif

    // prepare path?query string
    char * aPath = cJSON_malloc(128);
    memset(aPath, 0, 128);
    int len = strlen(HTTP2_STREAMING_ADDREC_PATH);
    memcpy(aPath, HTTP2_STREAMING_ADDREC_PATH, len);
    encode_sid(&(aPath[len]));

    h2pc_prepare_to_send_static((char *) pic->buf, pic->len);
    h2pc_do_post(aPath);
    h2pc_wait_for_response();
    cJSON_free(aPath);

    esp_camera_fb_return(pic);

    /* extract result */
    cJSON * resp = h2pc_consume_response_content();
    if (resp) {
        cJSON * result = cJSON_GetObjectItem(resp, JSON_RPC_RESULT);
        if (result &&
            (strcmp(result->valuestring, JSON_RPC_OK) == 0)) {
            locked_CLR_STATE(MODE_SEND_FB);
        } else
            protocol_errors++; // some server error
        cJSON_Delete(resp);
    }
}

static void send_get_msgs() {
    cJSON * tosend = cJSON_CreateObject();
    cJSON_AddStringToObject(tosend, JSON_RPC_SHASH, sid);
    cJSON_AddStringToObject(tosend, JSON_RPC_STAMP, last_stamp);
    h2pc_prepare_to_send(tosend);
    cJSON_Delete(tosend);
    h2pc_do_post(HTTP2_STREAMING_GETMSGS_PATH);
    h2pc_wait_for_response();
    /* extract result */
    if (h2pc_lock_incoming_pool()) {
        cJSON * resp = h2pc_consume_response_content();
        incoming_msgs_size = 0;
        incoming_msgs_pos = 0;
        if (resp) {
            cJSON * result = cJSON_GetObjectItem(resp, JSON_RPC_RESULT);
            if (result &&
                (strcmp(result->valuestring, JSON_RPC_OK) == 0)) {
                cJSON* msgs = cJSON_DetachItemFromObject(resp, JSON_RPC_MSGS);
                if (msgs) {
                    incoming_msgs_size = cJSON_GetArraySize(msgs);
                    incoming_msgs_pos = 0;
                    h2pc_set_incoming_pool(msgs);
                }
            } else
                protocol_errors++; // some server error
            cJSON_Delete(resp);
        }
        h2pc_unlock_incoming_pool();
    }
    locked_CLR_STATE(MODE_GET_MSG);
}

static void send_sync() {
    h2pc_prepare_to_send_static(sync_msg, strlen(sync_msg));
    h2pc_do_post(HTTP2_STREAMING_ADDMSGS_PATH);
    h2pc_wait_for_response();
    /* extract result */
    cJSON * resp  = h2pc_consume_response_content();
    if (resp) {
        cJSON * result = cJSON_GetObjectItem(resp, JSON_RPC_RESULT);
        if (result &&
            (strcmp(result->valuestring, JSON_RPC_OK) == 0)) {
            locked_CLR_STATE(MODE_SYNC);
        } else
            protocol_errors++; // some server error
        cJSON_Delete(resp);
    }
}

static void send_msgs() {
    cJSON * outgoing_msgs_dub = NULL;
    if (h2pc_lock_outgoing_pool()) {
        cJSON * outgoing_msgs = h2pc_outgoing_pool();
        if ((outgoing_msgs) && (cJSON_GetArraySize(outgoing_msgs) > 0)) {
            /* dublicate outgoing data to restore on error */
            outgoing_msgs_dub = cJSON_Duplicate(outgoing_msgs, true);
            h2pc_clr_outgoing_pool();
        }
        //
        h2pc_unlock_outgoing_pool();
    }
    if (outgoing_msgs_dub) {
        cJSON * tosend = cJSON_CreateObject();
        cJSON_AddStringToObject(tosend, JSON_RPC_SHASH, sid);
        cJSON_AddItemToObject(tosend, JSON_RPC_MSGS, outgoing_msgs_dub);
        h2pc_prepare_to_send(tosend);

        h2pc_do_post(HTTP2_STREAMING_ADDMSGS_PATH);
        h2pc_wait_for_response();
        /* extract result */
        cJSON * resp  = h2pc_consume_response_content();
        if (resp) {
            cJSON * result = cJSON_GetObjectItem(resp, JSON_RPC_RESULT);
            if (result &&
                (strcmp(result->valuestring, JSON_RPC_OK) == 0)) {
                locked_CLR_STATE(MODE_SEND_MSG);
            } else {
                /* restore not-sended data */
                if (h2pc_lock_outgoing_pool()) {
                    cJSON * outgoing_msgs = h2pc_outgoing_pool();
                    if (outgoing_msgs) {
                        while (cJSON_GetArraySize(outgoing_msgs_dub) > 0) {
                            cJSON * item = cJSON_DetachItemFromArray(outgoing_msgs_dub, 0);
                            cJSON_AddItemToArray(outgoing_msgs, item);
                        }
                    } else {
                        h2pc_set_outgoing_pool(cJSON_Duplicate(outgoing_msgs_dub, true));
                    }
                    h2pc_unlock_outgoing_pool();
                }
                protocol_errors++; // some server error
            }
            cJSON_Delete(resp);
        }
        cJSON_Delete(tosend);
    }
}

static void proceed_incoming_msgs() {
    bool need_to_sync = false;
    if (h2pc_lock_incoming_pool()) {
        cJSON * incoming_msgs = h2pc_incoming_pool();
        if (incoming_msgs && (incoming_msgs_pos < incoming_msgs_size)) {
            int cnt = 0;
            while (1) {
                cJSON * msg = cJSON_GetArrayItem(incoming_msgs, incoming_msgs_pos);

                if (msg) {
                    /* proceed message */
                    cJSON * ssrc = cJSON_GetObjectItem(msg,  JSON_RPC_DEVICE);  //who sent
                    cJSON * skind = cJSON_GetObjectItem(msg, JSON_RPC_MSG);     //what sent
                    cJSON * stmp = cJSON_GetObjectItem(msg,  JSON_RPC_STAMP);   //when sent
                    cJSON * spars = cJSON_GetObjectItem(msg, JSON_RPC_PARAMS);  //params
                    if (stmp) strcpy(last_stamp, stmp->valuestring);
                    cJSON * smid;
                    if (spars) {
                        cJSON * smid = cJSON_GetObjectItem(spars, JSON_RPC_MID); //msg id
                    } else smid = NULL;

                    /* check completeness */
                    if (ssrc && skind) {
                        char * src = ssrc->valuestring;
                        char * msgk = skind->valuestring;
                        cJSON * params = cJSON_CreateObject();
                        if (smid)
                            cJSON_AddNumberToObject(params, JSON_RPC_MID, smid->valuedouble);
                        #ifdef ADC_ENABLED
                        if (strcmp(JSON_RPC_GET_ADCVAL, msgk) == 0) {
                            cJSON_AddNumberToObject(params, JSON_RPC_ADCVAL, (double) locked_get_adc_voltage());
                            add_outgoing_msg(JSON_RPC_ADCVAL, src, params);
                        } else
                        #endif
                        if (strcmp(JSON_RPC_DOSNAP, msgk) == 0) {
                            cJSON_AddStringToObject(params, JSON_RPC_RESULT, JSON_RPC_OK);
                            add_outgoing_msg(JSON_RPC_DOSNAP, src, params);
                            locked_SET_STATE(MODE_SEND_FB);
                        } else
                        #ifdef OUT_ENABLED
                        if (strcmp(JSON_RPC_OUTPUT, msgk) == 0) {
                            if (spars) {
                                cJSON * spin = cJSON_GetObjectItem(spars,   JSON_RPC_PIN);   //selected pin
                                cJSON * slevel = cJSON_GetObjectItem(spars, JSON_RPC_LEVEL); //level value
                                if (spin && slevel) {
                                    uint8_t pinv, levelv;
                                    pinv = (uint8_t) spin->valueint;
                                    levelv = (uint8_t) slevel->valueint;
                                    board_out_operation(pinv, levelv);
                                    cJSON_AddStringToObject(params, JSON_RPC_RESULT, JSON_RPC_OK);
                                } else {
                                    cJSON_AddStringToObject(params, JSON_RPC_RESULT, JSON_RPC_BAD);
                                }
                                add_outgoing_msg(JSON_RPC_OUTPUT, src, params);
                            } else {
                                cJSON_AddStringToObject(params, JSON_RPC_RESULT, JSON_RPC_BAD);
                                add_outgoing_msg(JSON_RPC_OUTPUT, src, params);
                            }
                        } else
                        #endif
                        {
                            // you should delete params - no msg is sended
                            cJSON_Delete(params);
                        }
                        // no you should't delete here params -> there are owned by outgoing msg now

                        cnt++;
                    }
                }

                incoming_msgs_pos++;
                if (incoming_msgs_pos >= incoming_msgs_size) {
                    h2pc_clr_incoming_pool();
                    need_to_sync = true;
                    break;
                }
                if (cnt > 2) {
                    break;
                }
                vTaskDelay(2);
            }
        }
        h2pc_unlock_incoming_pool();
    }
    if (need_to_sync) locked_SET_STATE(MODE_SYNC);
}

void add_outgoing_msg(char * amsg, char * atarget, cJSON * content) {
    if (h2pc_lock_outgoing_pool()) {
        cJSON * outgoing_msgs = h2pc_outgoing_pool();
        if (outgoing_msgs == NULL) {
            outgoing_msgs = cJSON_CreateArray();
            h2pc_set_outgoing_pool(outgoing_msgs);
        }
        cJSON * msg = cJSON_CreateObject();
        cJSON_AddStringToObject(msg, JSON_RPC_MSG, amsg);
        if (atarget)
            cJSON_AddStringToObject(msg, JSON_RPC_TARGET, atarget);
        if (content)
            cJSON_AddItemToObject(msg, JSON_RPC_PARAMS, content);

        cJSON_AddItemToArray(outgoing_msgs, msg);
        //
        h2pc_unlock_outgoing_pool();
    }
}

static void camera_task(void *args)
{
    /* Waiting for connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    /* Set current time: proper system time is required for TLS based
     * certificate verification.
     */
    set_time();

    while (1)
    {
        #ifdef LOG_DEBUG
        ESP_LOGI(WC_TAG, "New step. states: %d", locked_GET_STATES());
        #endif
        if (!locked_CHK_STATE(MODE_CONN)) {
            connect_to_http2();
            if (connect_errors) {
                if (connect_errors > 10) {
                    vTaskDelay(300 * configTICK_RATE_HZ); // 5 minutes
                } else {
                    vTaskDelay(connect_errors * 10 * configTICK_RATE_HZ);
                }
            }
        }
        /* authorize the device on server */
        if (locked_CHK_STATE(MODE_AUTH)) {
            send_authorize();
        }
        /* send sync msg to server */
        if (locked_CHK_STATE(MODE_SYNC)) {
            send_sync();
        }
        /* gathering incoming msgs from server */
        if (locked_CHK_STATE(MODE_GET_MSG)) {
            esp_timer_stop(msgs_get);
            send_get_msgs();
            esp_timer_start_periodic(msgs_get, GET_MSG_TIMER_DELTA);
        }
        /* proceed incoming messages */
        proceed_incoming_msgs();

        /* send outgoing messages */
        if (locked_CHK_STATE(MODE_SEND_MSG)) {
            esp_timer_stop(msgs_send);
            send_msgs();
            esp_timer_start_periodic(msgs_send, SEND_MSG_TIMER_DELTA);
        }
        /* send framebuffer */
        if (locked_CHK_STATE(MODE_SEND_FB)) {
            esp_timer_stop(frame_send);
            send_framebuffer();
            esp_timer_start_once(frame_send, SEND_FRAME_TIMER_DELTA);
        }

        #ifdef ADC_ENABLED
        /* measure the current voltage value with adc */
        if (locked_CHK_STATE(MODE_ADC_PROBE)) {
            esp_timer_stop(adc_probe);
            board_get_adc_mV();
            esp_timer_start_periodic(adc_probe, MODE_ADC_PROBE);
        }
        #endif

        if (protocol_errors > 10) {
            disconnect_host();
        }

        vTaskDelay(1000);
    }

    finalize_app();

    vTaskDelete(NULL);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        #ifdef LOG_DEBUG
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_START");
        #endif
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        #ifdef LOG_DEBUG
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_GOT_IP");
        ESP_LOGI(WC_TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        #endif
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        #ifdef LOG_DEBUG
        ESP_LOGI(WC_TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        #endif
        if (locked_CHK_STATE(MODE_CONN)) h2pc_disconnect_http2();
        locked_CLR_ALL_STATES();

        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        h2pc_reset_buffers();
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = APP_WIFI_SSID,
            .password = APP_WIFI_PASS,
        },
    };
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    #endif
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        #ifdef LOG_DEBUG
        ESP_LOGE(WC_TAG, "Camera Init Failed");
        #endif
        return err;
    }

    return ESP_OK;
}

#ifdef INP_ENABLED
static void button_tap_cb(void* arg)
{
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "tap cb (%s)", (char *)arg);
    #endif

    /* find the pressed button and react */
    for (int i = 0; i < BUTTONS_CNT; i++) {
        if (strcmp(buttons[i].arg, (char *)arg) == 0) {
            /* react here */
            cJSON * params = cJSON_CreateObject();
            cJSON_AddStringToObject(params, JSON_RPC_BTN, arg);
            add_outgoing_msg(JSON_RPC_BTNEVENT, "", params); // params owned by msg now
            return;
        }
    }
}

static void board_buttons_init(void)
{
    for (int i = 0; i < BUTTONS_CNT; i++) {
        buttons[i].handle = iot_button_create(buttons[i].pin, buttons[i].active_level);
        if (buttons[i].handle) {
            iot_button_set_evt_cb(buttons[i].handle, BUTTON_CB_PUSH, button_tap_cb, buttons[i].arg);
        }
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
    #ifdef LOG_DEBUG
    ESP_LOGE(WC_TAG, "Out is not found!");
    #endif
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
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, adc_chars);
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
    locked_CLR_STATE(MODE_ADC_PROBE);
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

void adc_probe_cb(void* arg)
{
    if (CHK_STATE(MODE_CONN)) {
        SET_STATE(ADC_PROBE_TIMER_DELTA);
    }
}

#endif

void frames_send_cb(void* arg)
{
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Frame send fired");
    #endif
    if (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE) {
        if (CHK_STATE(MODE_CONN)) {
            SET_STATE(MODE_SEND_FB);
        }
        xSemaphoreGive(states_mux);
    }
}

void msgs_get_cb(void* arg)
{
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Get msgs fired");
    #endif
    bool isempty = h2pc_locked_incoming_pool_waiting();
    if (isempty && (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE)) {
        if (CHK_STATE(MODE_CONN)) {
            SET_STATE(MODE_GET_MSG);
        }
        xSemaphoreGive(states_mux);
    }
}

void msgs_send_cb(void* arg)
{
    #ifdef LOG_DEBUG
    ESP_LOGI(WC_TAG, "Send msgs fired");
    #endif
    bool isnempty = h2pc_locked_outgoing_pool_waiting();
    if (isnempty && (xSemaphoreTake(states_mux, portMAX_DELAY) == pdTRUE)) {
        if (CHK_STATE(MODE_CONN)) {
            SET_STATE(MODE_SEND_MSG);
        }
        xSemaphoreGive(states_mux);
    }
}

void finalize_app()
{
    disconnect_host();

    esp_timer_stop(frame_send);
    esp_timer_stop(msgs_send);
    esp_timer_stop(msgs_get);
    #ifdef ADC_ENABLED
    esp_timer_stop(adc_probe);
    #endif

    h2pc_finalize();

    if (sync_msg) cJSON_free(sync_msg);
    if (sync_stamp) cJSON_free(sync_stamp);
}

void app_main()
{
    states_mux = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    ESP_ERROR_CHECK( init_camera() );
    h2pc_initialize();

    /* generate the sync msg */
    uint8_t sta_mac[6];
    ESP_ERROR_CHECK( esp_efuse_mac_get_default(sta_mac) );
    for (int i = 0; i < 6; i++) {
        mac_str[i<<1] = UPPER_XDIGITS[(sta_mac[i] >> 4) & 0x0f];
        mac_str[(i<<1) + 1] = UPPER_XDIGITS[(sta_mac[i] & 0x0f)];
    }
    mac_str[12] = 0;
    last_stamp = cJSON_malloc(64);
    cJSON * sync = cJSON_CreateObject();
    cJSON_AddStringToObject( sync, JSON_RPC_MSG, JSON_RPC_SYNC );
    sync_stamp = cJSON_PrintUnformatted(sync);
    strcpy(last_stamp, sync_stamp);
    cJSON_Delete(sync);

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

    /* init timers */
    esp_timer_create_args_t timer_args = {
        .callback = &frames_send_cb,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &frame_send));

    timer_args.callback = &msgs_get_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &msgs_get));

    timer_args.callback = &msgs_send_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &msgs_send));

    #ifdef ADC_ENABLED
    timer_args.callback = &adc_probe_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &msgs_send));
    #endif

    /* start main task */
    xTaskCreate(&camera_task, "camera_task", (1024 * 64), NULL, 5, NULL);


    /* start timers */
    esp_timer_start_periodic(msgs_get, GET_MSG_TIMER_DELTA);
    esp_timer_start_periodic(msgs_send, SEND_MSG_TIMER_DELTA);
    esp_timer_start_once(frame_send, SEND_FRAME_TIMER_DELTA);
    #ifdef ADC_ENABLED
    esp_timer_start_periodic(adc_probe, ADC_PROBE_TIMER_DELTA);
    #endif
}
