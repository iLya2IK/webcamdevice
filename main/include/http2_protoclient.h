// Copyright 2022 Medvedkov Ilya
//
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HTTP2_PROTO_CLIENT
#define HTTP2_PROTO_CLIENT

#include <cJSON.h>

#define INITIAL_RESP_BUFFER 0x8000
#define MAXIMUM_RESP_BUFFER 0x200000

void h2pc_initialize();
bool h2pc_connect_to_http2(char * aserver);
void h2pc_prepare_to_send(cJSON * tosend);
void h2pc_prepare_to_send_static(char * buf, int size);
void h2pc_do_post(char * aPath);
bool h2pc_wait_for_response();
cJSON * h2pc_consume_response_content();
void h2pc_disconnect_http2();

bool h2pc_locked_incoming_pool_waiting();
bool h2pc_lock_incoming_pool();
cJSON * h2pc_incoming_pool();
void h2pc_set_incoming_pool(cJSON * data);
void h2pc_clr_incoming_pool();
cJSON * h2pc_set_incoming_from_response();
void h2pc_unlock_incoming_pool();

bool h2pc_locked_outgoing_pool_waiting();
bool h2pc_lock_outgoing_pool();
cJSON * h2pc_outgoing_pool();
void h2pc_set_outgoing_pool(cJSON * data);
void h2pc_clr_outgoing_pool();
void h2pc_unlock_outgoing_pool();

void h2pc_reset_buffers();
void h2pc_finalize();


#endif