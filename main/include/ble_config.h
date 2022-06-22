// Copyright 2022 Medvedkov Ilya
//
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BLE_CONFIG
#define BLE_CONFIG

#include <cJSON.h>

#define JSON_CFG "json_cfg"

enum
{
    IDX_SVC,
    IDX_CHAR_1,
    IDX_CHAR_VAL_1,

    IDX_CHAR_2,
    IDX_CHAR_VAL_2,
    IDX_CHAR_CFG_2,

    CFG_IDX_NB,
};

enum WC_CFG_ENUM {
  CFG_USER_NAME,
  CFG_USER_PASSWORD,
  CFG_HOST_NAME,
  CFG_SSID_NAME,
  CFG_SSID_PASSWORD,
  CFG_DEVICE_NAME,

  CFG_IDS_CNT
};

extern cJSON * WC_CFG_VALUES;

int initialize_ble(const cJSON * cfg);
int start_ble_config_round();
bool ble_config_proceed();
void stop_ble_config_round();

const char * get_cfg_id(int id);
char * get_cfg_value(int id);


#endif