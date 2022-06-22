/* Module to configure device via BLE

   Part of WCWebCamServer project

   Copyright (c) 2022 Ilya Medvedkov <sggdev.im@gmail.com>

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "defs.h"

#include <string.h>
#include <stdlib.h>

#include "ble_config.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_system.h"

#define GATTS_TABLE_TAG "BLE_CFG"
#define GATTS_TAG "BLE_GATTS"
#define GATTC_TAG "BLE_GATTC"

cJSON * WC_CFG_VALUES;
static bool ble_enabled = false;

///Declare the static function
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const uint16_t GATTS_SERVICE_UUID_CFG    = CONFIG_WC_DEVICE_SERVICE_UUID;
static const uint16_t BCONFIGPROFILE_CHAR1_UUID = CONFIG_WC_DEVICE_CHAR1_UUID;
static const uint16_t BCONFIGPROFILE_CHAR2_UUID = CONFIG_WC_DEVICE_CHAR2_UUID;

const char const DEVICE_NAME [] = CONFIG_WC_DEVICE_NAME;

#define ESP_APP_ID                  0x14

#define GATTS_NUM_HANDLE_CFG        4
#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define SVC_INST_ID                 0

#define PREPARE_BUF_MAX_SIZE            256
#define BCONFIGPROFILE_VALUE_MAX_LEN    21
#define BCONFIGPROFILE_COMMAND_MAX_LEN	128

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

typedef struct {
    uint8_t                *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct {
    prepare_type_env_t prepare_write_env_cfg;
    uint16_t * cfg_handle_table;
    uint8_t * raw_adv_data;
    struct gatts_profile_inst cfg_profile_tab[PROFILE_NUM];
    esp_ble_adv_params_t adv_params;
    uint8_t * BCONFIGPROFILEChar1;
    uint8_t BCONFIGPROFILEChar1Len;
    uint8_t * BCONFIGPROFILEChar2;
    uint8_t BCONFIGPROFILEChar2Len;
    char * cfgTotal;
    int  cfgTotalLen;
    int  cfgReadLoc;
    bool cfgNeedRebuild;
    bool cfgReadyToNotify;
    int8_t cfgRebuildFrom;

    uint8_t adv_config_done;

    esp_timer_handle_t tim_stop_adv;
    esp_timer_handle_t tim_send_new_data_portion;
} ble_mode_config_t;

static ble_mode_config_t * ble_cfg;

static const uint8_t const_raw_adv_data[12] = {
        /* flags */
        0x02, 0x01, 0x06, //GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
        /* tx power*/
        0x02, 0x0a, 0x00,
        /* service uuid */
        0x03, 0x03, 0xF0, 0x9E,
        /* device name */
        sizeof(DEVICE_NAME), 0x09
};
static const uint8_t raw_scan_rsp_data[10] = {
        /* flags */
        0x02, 0x01, 0x06, //GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
        /* tx power */
        0x02, 0x0a, 0x00,
        /* service uuid */
        0x03, 0x03, 0xF0, 0x9E
};

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t BCONFIGPROFILEChar2Config[2]  = {0x00, 0x00};
static const uint8_t char_value[1]                 = {0x11};
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

/* busyness logic */
// Length of bd addr as a string
#define B_ADDR_STR_LEN    15

#define MAX_CMD_STR_LEN   12
#define MAX_JSON_STR_LEN  120

#define CFG_WRITE_ONLY    1
#define CFG_READ_ONLY     2
#define CFG_WRITEREAD     0

static const char* cfgIds[CFG_IDS_CNT] =
{
  "u",
  "p",
  "h",
  "s",
  "k",
  "d"
};
static const uint8_t cfgOpts[CFG_IDS_CNT] =
{
  CFG_WRITEREAD,
  CFG_WRITE_ONLY,
  CFG_WRITEREAD,
  CFG_WRITEREAD,
  CFG_WRITE_ONLY,
  CFG_READ_ONLY
};

const char * get_cfg_id(int id) {
    return cfgIds[id];
}

char * get_cfg_value(int id) {
    if (WC_CFG_VALUES == NULL) return NULL;

    cJSON * value_item = cJSON_GetArrayItem(WC_CFG_VALUES, id);
    if (value_item != NULL) {
        cJSON * value = cJSON_GetObjectItem(value_item, get_cfg_id(id));
        if (value != NULL) {
            return cJSON_GetStringValue(value);
        } else
            return NULL;
    } else {
        return NULL;
    }
}

#define CFG_TOTAL_LEN 1024

void restartCfg() {
  ble_cfg->cfgReadLoc = 0;
  ble_cfg->cfgRebuildFrom = 0;
}

void rebuildCfg() {
  if (ble_cfg->cfgNeedRebuild) {
    ble_cfg->cfgNeedRebuild = false;

    ble_cfg->cfgReadLoc = 0;
    ble_cfg->cfgTotalLen = 1;
    ble_cfg->cfgTotal[0] = '\t';

    for (uint8_t id = ble_cfg->cfgRebuildFrom; id < CFG_IDS_CNT; id++) {
        char * state = &(ble_cfg->cfgTotal[ble_cfg->cfgTotalLen]);

        cJSON * value_item = cJSON_GetArrayItem(WC_CFG_VALUES, id);

        int len  = 0;
        if ((value_item == NULL) || ((cfgOpts[id] & CFG_WRITE_ONLY) > 0)) {
            state[len] = '{';len++;
            state[len] = '"';len++;
            memcpy(&(state[len]), cfgIds[id], strlen(cfgIds[id]));
            len += strlen(cfgIds[id]);
            state[len] = '"';len++;
            state[len] = ':';len++;
            state[len] = '"';len++;
            state[len] = '"';len++;
            state[len] = '}';len++;
        } else {
            char * str_value = cJSON_PrintUnformatted(value_item);
            len = strlen(str_value);
            memcpy(state, str_value, len);
            cJSON_free(str_value);
        }
        state[len] = '\r';len++;
        state[len] = '\n';len++;
        ble_cfg->cfgTotalLen += len;
    }
    ble_cfg->cfgTotal[ble_cfg->cfgTotalLen] = 0;
  }
}

void setConfigValue(int id, cJSON * val) {
    if ((id >= 0) && (id < CFG_IDS_CNT)) {
        cJSON_DeleteItemFromArray(WC_CFG_VALUES, id);
        cJSON_InsertItemInArray(WC_CFG_VALUES, id, val);

        ble_cfg->cfgNeedRebuild = true;
        ble_cfg->cfgRebuildFrom = id;
    }
}

bool writeValueToProfile() {
    if (ble_cfg->cfgReadyToNotify) {
        rebuildCfg();
        if ((ble_cfg->cfgTotalLen > 0) && (ble_cfg->cfgReadLoc < ble_cfg->cfgTotalLen)) {
            int len = ble_cfg->cfgTotalLen - ble_cfg->cfgReadLoc;
            if (len >= BCONFIGPROFILE_VALUE_MAX_LEN)
                len = BCONFIGPROFILE_VALUE_MAX_LEN - 1;
            memcpy(ble_cfg->BCONFIGPROFILEChar2, &(ble_cfg->cfgTotal[ble_cfg->cfgReadLoc]), len);
            ble_cfg->cfgReadLoc += len;
            ble_cfg->BCONFIGPROFILEChar2[len] = 0;
            ble_cfg->BCONFIGPROFILEChar2Len = len;
            ble_cfg->cfgReadyToNotify = false;
            return true;
        } else {
            restartCfg();
            return false;
        }
    } else return false;
}

#define NEXT_DATA_TIMER_DELTA       250000
#define STOP_ADV_TIMER_DELTA        30000000
#define STOP_ADV_TIMER_SHORT_DELTA  10000000

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[CFG_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_CFG), (uint8_t *) &GATTS_SERVICE_UUID_CFG}},

    /* Characteristic Declaration */
    [IDX_CHAR_1]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_1] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&BCONFIGPROFILE_CHAR1_UUID, ESP_GATT_PERM_WRITE,
      BCONFIGPROFILE_VALUE_MAX_LEN, 0, (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_2]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&BCONFIGPROFILE_CHAR2_UUID, ESP_GATT_PERM_READ,
      BCONFIGPROFILE_VALUE_MAX_LEN, 0, (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(BCONFIGPROFILEChar2Config), (uint8_t *)BCONFIGPROFILEChar2Config}}
};


void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ble_cfg->adv_config_done &= (~ADV_CONFIG_FLAG);
        if (ble_cfg->adv_config_done == 0){
            esp_ble_gap_start_advertising(&(ble_cfg->adv_params));
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ble_cfg->adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (ble_cfg->adv_config_done == 0){
            esp_ble_gap_start_advertising(&(ble_cfg->adv_params));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void exec_write_char1(int len, int offset, const uint8_t * value, const uint8_t * bda) {
    if (len > 0) {
        if ((offset + len) >= BCONFIGPROFILE_COMMAND_MAX_LEN) {
            ESP_LOGE(GATTS_TABLE_TAG, "GATT_WRITE_EVT, data exeed BCONFIGPROFILEChar1");
            return;
        }
        memcpy(&(ble_cfg->BCONFIGPROFILEChar1[offset]), value, len);
        ble_cfg->BCONFIGPROFILEChar1Len = offset + len;
        ble_cfg->BCONFIGPROFILEChar1[ble_cfg->BCONFIGPROFILEChar1Len] = 0;
        uint8_t *newCommand = ble_cfg->BCONFIGPROFILEChar1;
        int totallen = ble_cfg->BCONFIGPROFILEChar1Len;
        if ( totallen > 8 ) {
            char s1[MAX_CMD_STR_LEN], s2[MAX_JSON_STR_LEN];
            memcpy(s1, newCommand, 5);
            int e = totallen - 1;
            while ( (e > 5) && (newCommand[e] != '}') ) e--;
            memcpy(s2, &(newCommand[6]), e - 5);
            s1[5] = 0;
            s2[e - 5] = 0;
            if ((strcmp(s1, "*cmd*") == 0) && (strcmp(s2, "exit") == 0)){
                //GAPRole_TerminateConnection();
                esp_ble_gap_disconnect(bda);
            } else
            {
                if (strcmp(s1, "*get*") == 0)
                    restartCfg();
                else
                if (strcmp(s1, "*set*") == 0) {
                    int s2len = strlen(s2);
                    if (s2len >= 2) {
                        cJSON * resp = cJSON_Parse(s2);
                        if (resp) {
                            int8_t k = -1;

                            for (int8_t i = 0; i < CFG_IDS_CNT; i++) {
                                cJSON * value_item = cJSON_GetObjectItem(resp, get_cfg_id(i));

                                if ((value_item != NULL) && ((cfgOpts[i] & CFG_READ_ONLY) == 0)) {
                                    k = i;
                                    break;
                                }
                            }

                            if (k >= 0)
                                setConfigValue(k, resp);
                            else
                                cJSON_Delete(resp);
                        }
                    }
                }
            }
        }
    }
}

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_char(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        exec_write_char1(prepare_write_env->prepare_len, 0, prepare_write_env->prepare_buf, param->exec_write.bda);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: {
                esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
                if (set_dev_name_ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
                }
                esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(ble_cfg->raw_adv_data, (sizeof(const_raw_adv_data) + sizeof(DEVICE_NAME) - 1));
                if (raw_adv_ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
                }
                ble_cfg->adv_config_done |= ADV_CONFIG_FLAG;
                esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
                if (raw_scan_ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
                }
                ble_cfg->adv_config_done |= SCAN_RSP_CONFIG_FLAG;
                esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, CFG_IDX_NB, SVC_INST_ID);
                if (create_attr_ret){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
                }
            }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // the data length of gattc write  must be less than BCONFIGPROFILE_VALUE_MAX_LEN.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                #ifdef LOG_DEBUG
                esp_log_buffer_char(GATTS_TABLE_TAG, param->write.value, param->write.len);
                #endif

                if (ble_cfg->cfg_handle_table[IDX_CHAR_VAL_1] == param->write.handle) {
                    exec_write_char1(param->write.len, param->write.offset, param->write.value, param->write.bda);
                } else
                if (ble_cfg->cfg_handle_table[IDX_CHAR_CFG_2] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        restartCfg();
                        esp_timer_start_periodic(ble_cfg->tim_send_new_data_portion, NEXT_DATA_TIMER_DELTA);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                        esp_timer_stop(ble_cfg->tim_send_new_data_portion);
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        #ifdef LOG_DEBUG
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                        #endif
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            } else {
                /* handle prepare write */
                prepare_write_event_env(gatts_if, &(ble_cfg->prepare_write_env_cfg), param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than BCONFIGPROFILE_VALUE_MAX_LEN.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            exec_write_event_env(&(ble_cfg->prepare_write_env_cfg), param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ble_cfg->cfgReadyToNotify = true;
            if (param->conf.status != ESP_GATT_OK)
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            #ifdef LOG_DEBUG
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            #endif
            esp_timer_stop(ble_cfg->tim_stop_adv);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x40;     // max_int = 0x40*1.25ms  = 80ms
            conn_params.min_int = 0x20;     // min_int = 0x20*1.25ms  = 40ms
            conn_params.timeout = 400;      // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&(ble_cfg->adv_params));
            esp_timer_start_periodic(ble_cfg->tim_stop_adv, STOP_ADV_TIMER_SHORT_DELTA);
            esp_timer_stop(ble_cfg->tim_send_new_data_portion);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != CFG_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to CFG_IDX_NB(%d)", param->add_attr_tab.num_handle, CFG_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(ble_cfg->cfg_handle_table, param->add_attr_tab.handles, sizeof(uint16_t) * CFG_IDX_NB);
                esp_ble_gatts_start_service(ble_cfg->cfg_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
            ble_enabled = false;
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            ble_cfg->cfg_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == ble_cfg->cfg_profile_tab[idx].gatts_if) {
                if (ble_cfg->cfg_profile_tab[idx].gatts_cb) {
                    ble_cfg->cfg_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

int initialize_ble(const cJSON * cfg) {
    #ifdef LOG_DEBUG
    heap_caps_print_heap_info(MALLOC_CAP_DMA|MALLOC_CAP_EXEC);
    #endif

    ble_cfg = (ble_mode_config_t *)malloc(sizeof(ble_mode_config_t));

    ble_cfg->prepare_write_env_cfg.prepare_buf = NULL;
    ble_cfg->prepare_write_env_cfg.prepare_len = 0;
    ble_cfg->cfg_handle_table = (uint16_t *)malloc(sizeof(uint16_t) * CFG_IDX_NB);
    ble_cfg->raw_adv_data = (uint8_t *)malloc((sizeof(const_raw_adv_data) + sizeof(DEVICE_NAME) - 1) * sizeof(uint8_t));;
    memcpy(&(ble_cfg->raw_adv_data[0]), &(const_raw_adv_data[0]), sizeof(const_raw_adv_data));
    memcpy(&(ble_cfg->raw_adv_data[sizeof(const_raw_adv_data)]), DEVICE_NAME, sizeof(DEVICE_NAME)-1);
    memset(&(ble_cfg->cfg_profile_tab[PROFILE_APP_IDX]), 0x00, sizeof(struct gatts_profile_inst));
    memset(&(ble_cfg->adv_params), 0x00, sizeof(esp_ble_adv_params_t));

    ble_cfg->cfg_profile_tab[PROFILE_APP_IDX].gatts_cb = gatts_profile_event_handler;
    ble_cfg->cfg_profile_tab[PROFILE_APP_IDX].gatts_if = ESP_GATT_IF_NONE;
    ble_cfg->adv_params.adv_int_min         = 0x20;
    ble_cfg->adv_params.adv_int_max         = 0x40;
    ble_cfg->adv_params.adv_type            = ADV_TYPE_IND;
    ble_cfg->adv_params.own_addr_type       = BLE_ADDR_TYPE_PUBLIC;
    ble_cfg->adv_params.channel_map         = ADV_CHNL_ALL;
    ble_cfg->adv_params.adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    ble_cfg->BCONFIGPROFILEChar1 = (uint8_t *)malloc(BCONFIGPROFILE_COMMAND_MAX_LEN * sizeof(uint8_t));
    ble_cfg->BCONFIGPROFILEChar1Len = 0;
    ble_cfg->BCONFIGPROFILEChar2 = (uint8_t *)malloc(BCONFIGPROFILE_VALUE_MAX_LEN   * sizeof(uint8_t));
    ble_cfg->BCONFIGPROFILEChar2Len = 0;
    ble_cfg->cfgTotal = (char *)malloc(CFG_TOTAL_LEN * sizeof(char));
    ble_cfg->cfgTotalLen = 0;
    ble_cfg->cfgReadLoc = 0;
    ble_cfg->cfgNeedRebuild = true;
    ble_cfg->cfgReadyToNotify = true;
    ble_cfg->cfgRebuildFrom = 0;
    ble_cfg->adv_config_done = 0;

    ble_enabled = false;

    WC_CFG_VALUES = cJSON_CreateArray();
    for (int8_t i = 0; i < CFG_IDS_CNT; i++) {
        int8_t k = -1;
        for (int8_t j = 0; j < cJSON_GetArraySize(cfg); j++)  {
            cJSON * value_item = cJSON_GetArrayItem(cfg, j);
            cJSON * value = cJSON_GetObjectItem(value_item, get_cfg_id(i));
            if (value != NULL) {
                k = j;
                cJSON * valued_obj = cJSON_CreateObject();
                cJSON_AddStringToObject(valued_obj, get_cfg_id(i), cJSON_GetStringValue(value));
                setConfigValue(i, valued_obj);
                break;
            }
        }
        if (k < 0) {
            cJSON * empty_obj = cJSON_CreateObject();
            cJSON_AddStringToObject(empty_obj, get_cfg_id(i), "");
            setConfigValue(i, empty_obj);
        }
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }

    ble_enabled = true;

    return 0;
}

void stop_adv_cb(void* arg)
{
    ble_enabled = false;
}

void send_new_data_portion_cb(void* arg)
{
    if (writeValueToProfile()) {
        esp_ble_gatts_send_indicate(ble_cfg->cfg_profile_tab[PROFILE_APP_IDX].gatts_if, ble_cfg->cfg_profile_tab[PROFILE_APP_IDX].conn_id,
                                    ble_cfg->cfg_handle_table[IDX_CHAR_VAL_2], ble_cfg->BCONFIGPROFILEChar2Len, ble_cfg->BCONFIGPROFILEChar2, false);
        ESP_LOGI(GATTS_TABLE_TAG, "indicate: ");
        #ifdef LOG_DEBUG
        esp_log_buffer_char(GATTS_TABLE_TAG, ble_cfg->BCONFIGPROFILEChar2, ble_cfg->BCONFIGPROFILEChar2Len);
        #endif
    }
}

bool ble_config_proceed() {
    return ble_enabled;
}

void stop_ble_config_round() {
    esp_timer_stop(ble_cfg->tim_stop_adv);
    esp_timer_stop(ble_cfg->tim_send_new_data_portion);
    esp_timer_delete(ble_cfg->tim_stop_adv);
    esp_timer_delete(ble_cfg->tim_send_new_data_portion);

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    free(ble_cfg->cfgTotal);
    free(ble_cfg->BCONFIGPROFILEChar1);
    free(ble_cfg->BCONFIGPROFILEChar2);
    free(ble_cfg->raw_adv_data);
    free(ble_cfg->cfg_handle_table);

    if (ble_cfg->prepare_write_env_cfg.prepare_buf != NULL) {
        free(ble_cfg->prepare_write_env_cfg.prepare_buf);
        ble_cfg->prepare_write_env_cfg.prepare_buf = NULL;
    }

    free(ble_cfg);

    #ifdef LOG_DEBUG
    heap_caps_print_heap_info(MALLOC_CAP_DMA|MALLOC_CAP_EXEC);
    #endif
}

int start_ble_config_round() {
    esp_timer_create_args_t timer_args;

    esp_err_t ret;

    timer_args.callback = &stop_adv_cb;
    ret = esp_timer_create(&timer_args, &(ble_cfg->tim_stop_adv));
    if (ret){
        ESP_LOGE(GATTS_TAG, "cant create timer 1, error code = %x", ret);
        return ret;
    }

    timer_args.callback = &send_new_data_portion_cb;
    ret = esp_timer_create(&timer_args, &(ble_cfg->tim_send_new_data_portion));
    if (ret){
        ESP_LOGE(GATTS_TAG, "cant create timer 2, error code = %x", ret);
        return ret;
    }

    /* start timers */
    ret = esp_timer_start_periodic(ble_cfg->tim_stop_adv, STOP_ADV_TIMER_DELTA);
    if (ret){
        ESP_LOGE(GATTS_TAG, "cant start timer 1, error code = %x", ret);
        return ret;
    }

    return ret;
}