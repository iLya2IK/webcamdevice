/* HTTP2 helper msgs pools for client

   Part of WCWebCamServer project

   Copyright (c) 2022 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event_loop.h"
#include "lwip/apps/sntp.h"
#include "http2_protoclient.h"
#include <cJSON.h>
#include "sh2lib.h"
#include "esp_log.h"

/* current http2 connection */
static struct sh2lib_handle hd;

/* request data */
volatile bool   bytes_need_to_free = false; // is current raw bytes need to free after request sent
static char *   bytes_tosend = NULL;        // current raw bytes request content
volatile int    bytes_tosend_len = 0;       // current raw bytes request content length
volatile int    bytes_tosend_pos = 0;       // current raw bytes request content pos
volatile bool   request_finished = false;   // is current request finished

static char *   bytes_frame = NULL;         // current raw bytes frame content
volatile int    bytes_frame_len = 0;        // current raw bytes frame content length
volatile int    bytes_frame_pos = 0;        // current raw bytes frame content pos
volatile int    bytes_frame_pos_sended = 0;
volatile int32_t streaming_strm_id = -1;
volatile bool   sending_finished = false;

/* response data */
volatile int    resp_buffer_size = INITIAL_RESP_BUFFER;  // current response content buffer size
volatile int    resp_len = 0;               // current response content length
static char * resp_buffer = NULL;           // current response content

/* messages pools */
static SemaphoreHandle_t incoming_msgs_mux = NULL;
static SemaphoreHandle_t outgoing_msgs_mux = NULL;
static cJSON * incoming_msgs = NULL;
static cJSON * outgoing_msgs = NULL;

volatile bool client_connected = false;

#define FRAME_HEADER_SIZE (sizeof(uint16_t) + sizeof(uint32_t))
#define FRAME_START_SEQ ((uint16_t)0xaaaa)

void h2pc_initialize() {
    /* allocating responsing content */
    resp_buffer = cJSON_malloc(resp_buffer_size);
    resp_len = 0;

    incoming_msgs_mux = xSemaphoreCreateMutex();
    outgoing_msgs_mux = xSemaphoreCreateMutex();
}

bool h2pc_connected() {
    return client_connected;
}

bool h2pc_is_streaming() {
    return streaming_strm_id > 0;
}

bool h2pc_lock_incoming_pool() {
    return (xSemaphoreTake(incoming_msgs_mux, portMAX_DELAY) == pdTRUE);
}

cJSON * h2pc_incoming_pool() {
    return incoming_msgs;
}

void h2pc_set_incoming_pool(cJSON * data) {
    if (incoming_msgs) cJSON_Delete(incoming_msgs);
    incoming_msgs = data;
}

cJSON * h2pc_set_incoming_from_response() {
    if (incoming_msgs) cJSON_Delete(incoming_msgs);
    incoming_msgs = h2pc_consume_response_content();
    return incoming_msgs;
}

void h2pc_clr_incoming_pool() {
    if (incoming_msgs) cJSON_Delete(incoming_msgs);
    incoming_msgs = NULL;
}

void h2pc_unlock_incoming_pool() {
    xSemaphoreGive(incoming_msgs_mux);
}

bool h2pc_lock_outgoing_pool() {
    return (xSemaphoreTake(outgoing_msgs_mux, portMAX_DELAY) == pdTRUE);
}

cJSON * h2pc_outgoing_pool() {
    return outgoing_msgs;
}

void h2pc_clr_outgoing_pool() {
    if (outgoing_msgs) cJSON_Delete(outgoing_msgs);
    outgoing_msgs = NULL;
}

void h2pc_set_outgoing_pool(cJSON * data) {
    if (outgoing_msgs) cJSON_Delete(outgoing_msgs);
    outgoing_msgs = data;
}

void h2pc_unlock_outgoing_pool() {
    xSemaphoreGive(outgoing_msgs_mux);
}

bool h2pc_locked_incoming_pool_waiting() {
    bool val = true;
    if (xSemaphoreTake(incoming_msgs_mux, portMAX_DELAY) == pdTRUE) {
        if (incoming_msgs)
          val = cJSON_GetArraySize(incoming_msgs) == 0;
        xSemaphoreGive(incoming_msgs_mux);
    }
    return val;
}

bool h2pc_locked_outgoing_pool_waiting() {
    bool val = false;
    if (xSemaphoreTake(outgoing_msgs_mux, portMAX_DELAY) == pdTRUE) {
        val = (outgoing_msgs) && (cJSON_GetArraySize(outgoing_msgs) > 0);
        xSemaphoreGive(outgoing_msgs_mux);
    }
    return val;
}

void h2pc_reset_buffers() {
    resp_len = 0;
    if (bytes_need_to_free && bytes_tosend) {
        cJSON_free(bytes_tosend);
        bytes_tosend = NULL;
        bytes_need_to_free = false;
    }
    if (bytes_frame) {
        cJSON_free(bytes_frame);
        bytes_frame = NULL;
    }
}

void h2pc_finalize() {
    if (incoming_msgs) cJSON_Delete(incoming_msgs);
    if (outgoing_msgs) cJSON_Delete(outgoing_msgs);
}

void h2pc_disconnect_http2() {
    if (client_connected) {
        sh2lib_free(&hd);
        streaming_strm_id = -1;
        client_connected = false;
    }
}

bool h2pc_connect_to_http2(char * aserver) {
    /* HTTP2: one connection multiple requests. Do the TLS/TCP connection first */
    ESP_LOGI(WC_TAG, "Connecting to server: %s", aserver);
    if (sh2lib_connect(&hd, aserver) != 0) {
        ESP_LOGI(WC_TAG, "Failed to connect");
        return false;
    }
    ESP_LOGI(WC_TAG, "Connection done");

    client_connected = true;
    return true;
}

void h2pc_prepare_to_send(cJSON * tosend) {
    bytes_tosend = cJSON_PrintUnformatted(tosend);
    bytes_tosend_len = strlen(bytes_tosend);
    bytes_tosend_pos = 0;
    bytes_need_to_free = true;
    request_finished = false;
}

void h2pc_prepare_to_send_static(char * buf, int size) {
    bytes_tosend = buf;
    bytes_tosend_len = size;
    bytes_tosend_pos = 0;
    bytes_need_to_free = false;
    request_finished = false;
}

void h2pc_prepare_frame(char * buf, int size) {
    bytes_frame = buf;
    bytes_frame_len = size + FRAME_HEADER_SIZE;
    bytes_frame_pos = 0;
    bytes_frame_pos_sended = 0;
    sending_finished = false;
}

int handle_get_response(struct sh2lib_handle *handle, int32_t stream_id, const char *data, size_t len, int flags)
{
    if (len) {
        ESP_LOGI(WC_TAG, "[get-response] %.*s", len, data);
        int new_resp_buffer_size = resp_len + len;
        if (new_resp_buffer_size >= resp_buffer_size) {
            if (new_resp_buffer_size < MAXIMUM_RESP_BUFFER) {
                new_resp_buffer_size = (new_resp_buffer_size / 1024 + 1) * 1024;
                if (new_resp_buffer_size > MAXIMUM_RESP_BUFFER) {
                    new_resp_buffer_size = MAXIMUM_RESP_BUFFER;
                }
                resp_buffer = realloc(resp_buffer, new_resp_buffer_size);
                resp_buffer_size = new_resp_buffer_size;
            } else {
                ESP_LOGI(WC_TAG, "[get-response] response buffer overflow");
                return 0;
            }
        }
        memcpy(&(resp_buffer[resp_len]), data, len);
        resp_len += len;
    }
    if (flags == DATA_RECV_FRAME_COMPLETE) {
        ESP_LOGI(WC_TAG, "[get-response] Frame fully received");
    } else
    if ( flags == DATA_RECV_RST_STREAM ) {
        ESP_LOGI(WC_TAG, "[get-response] Stream Closed");
        if (resp_len == resp_buffer_size) {
            /* not often but may be */
            resp_buffer = realloc(resp_buffer, resp_buffer_size + 1);
        }
        resp_buffer[resp_len] = 0; // terminate string
        request_finished = true;
    } else
    if ( flags == DATA_RECV_GOAWAY ) {
        h2pc_disconnect_http2();
    }
    return 0;
}

int send_post_data(struct sh2lib_handle *handle, int32_t stream_id, char *buf, size_t length, uint32_t *data_flags)
{
    int cur_bytes_tosend_len = bytes_tosend_len - bytes_tosend_pos;
    if (cur_bytes_tosend_len < length) length = cur_bytes_tosend_len;

    if (length > 0) {
        /* dst - buf,
         * src - bytes_tosend at bytes_tosend_pos */
        memcpy(buf, &(bytes_tosend[bytes_tosend_pos]), length);
        ESP_LOGI(WC_TAG, "[data-prvd] Sending %d bytes", length);
        bytes_tosend_pos += length;
    }

    if (bytes_tosend_len == bytes_tosend_pos) {
        (*data_flags) |= NGHTTP2_DATA_FLAG_EOF;
    }

    return length;
}

int send_put_data(struct sh2lib_handle *handle, int32_t stream_id, char *buf, size_t length, uint32_t *data_flags)
{
    int cur_bytes_tosend_len = bytes_frame_len - bytes_frame_pos;
    if (cur_bytes_tosend_len < length) length = cur_bytes_tosend_len;

    if ( length > 0 ) {

        /* dst - buf,
         * src - bytes_tosend at bytes_tosend_pos */

        int off;
        if (bytes_frame_pos == 0) {
            *((uint16_t *)buf) = FRAME_START_SEQ;
            uint32_t sz = bytes_frame_len - FRAME_HEADER_SIZE;
            *((uint32_t *)&(buf[sizeof(uint16_t)])) = (uint32_t)(sz);

            off = FRAME_HEADER_SIZE;
            bytes_frame_pos += off;
        } else
            off = 0;

        size_t len = length - off;
        memcpy(&(buf[off]), &(bytes_frame[bytes_frame_pos - FRAME_HEADER_SIZE]), len);
        ESP_LOGI(WC_TAG, "[data-prvd] Sending %d bytes", length);
        bytes_frame_pos += len;
    }

    if (bytes_frame_len == bytes_frame_pos) {
        (*data_flags) |= NGHTTP2_DATA_FLAG_NO_END_STREAM;
        if (length == 0)
            return NGHTTP2_ERR_DEFERRED;
    }

    return length;
}

int handle_response(struct sh2lib_handle *handle, int32_t stream_id, const char *data, size_t len, int flags)
{
    if (flags == DATA_SEND_FRAME_DATA) {
        size_t lenv = *((size_t*) data);
        bytes_frame_pos_sended += lenv;
        if (bytes_frame_pos_sended == bytes_frame_len) {
            sending_finished = true;
        }
    } else
    if (flags == DATA_RECV_FRAME_COMPLETE) {
        ESP_LOGI(WC_TAG, "[put-response] Frame fully received");
    } else
    if ( flags == DATA_RECV_RST_STREAM ) {
        ESP_LOGI(WC_TAG, "[put-response] Stream Closed");
        sending_finished = true;
        streaming_strm_id = -1;
    } else
    if ( flags == DATA_RECV_GOAWAY ) {
        h2pc_disconnect_http2();
    }
    return NGHTTP2_ERR_WOULDBLOCK;
}

void h2pc_do_post(char * aPath) {
    sh2lib_do_post(&hd, aPath, bytes_tosend_len, send_post_data, handle_get_response);
}

void h2pc_prepare_out_stream(char * aPath) {
    streaming_strm_id = sh2lib_do_put(&hd, aPath, send_put_data, handle_response);
    ESP_LOGD(WC_TAG, "[data-prvd] Streaming stream id = %d", streaming_strm_id);
}

bool h2pc_wait_for_response() {
    bool res = true;
    resp_len = 0;
    while (1) {
        /* Process HTTP2 send/receive */
        if (sh2lib_execute(&hd) < 0) {
            ESP_LOGE(WC_TAG, "Error in send/receive");
            h2pc_disconnect_http2();
            res = false;
            break;
        }
        if (request_finished || !h2pc_connected())
            break;

        vTaskDelay(2);
    }
    if (bytes_need_to_free) {
        cJSON_free(bytes_tosend);
        //
        bytes_need_to_free = false;
    }
    bytes_tosend = NULL;
    bytes_tosend_len = 0;
    bytes_tosend_pos = 0;
    return res;
}

bool h2pc_wait_for_frame_sending() {
    bool res = true;

    while (1) {
        if (h2pc_is_streaming())
            nghttp2_session_resume_data(hd.http2_sess, streaming_strm_id);

        /* Process HTTP2 send/receive */
        int ret = nghttp2_session_recv(hd.http2_sess);
        if (ret != 0) {
            ESP_LOGE(WC_TAG, "[sh2-frame-send] HTTP2 session recv failed %d", ret);
            h2pc_disconnect_http2();
            res = false;
            break;
        }
        ret = nghttp2_session_send(hd.http2_sess);
        if (ret != 0) {
            ESP_LOGE(WC_TAG, "[sh2-frame-send] HTTP2 session send failed %d", ret);
            h2pc_disconnect_http2();
            res = false;
            break;
        }

        if (sending_finished || !h2pc_connected())
            break;

        vTaskDelay(2);
    }
    ESP_LOGD(WC_TAG, "Frame sended");
    bytes_frame = NULL;
    bytes_frame_len = 0;
    bytes_frame_pos = 0;
    bytes_frame_pos_sended = 0;
    return res;
}

cJSON * h2pc_consume_response_content() {
    if (resp_len > 0) {
        cJSON * resp = cJSON_Parse(resp_buffer);
        if (resp) {
            return resp;
        } else return NULL;
    } else return NULL;
}