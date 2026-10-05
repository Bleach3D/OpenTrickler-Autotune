#include <pico/cyw43_arch.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>

// #include <lwip/apps/httpd.h>

#include "wireless.h"
#include "eeprom.h"
#include "access_point_mode.h"
#include "display.h"
#include "mini_12864_module.h"
#include "http_rest.h"
#include "rest_endpoints.h"
#include "common.h"
#include "lwip/apps/mdns.h"


#ifdef CYW43_HOST_NAME
#undef CYW43_HOST_NAME
#endif

// Overwrite the host name
#define CYW43_HOST_NAME "opentrickler"
#define LED_INTERFACE_MINIMUM_POLL_PERIOD_MS    20

// The hostname is used by STA mode to advertise mDNS. The value is also used for AP mode as the SSID
char host_name[18];

typedef enum {
    WIRELESS_STATE_NOT_INITIALIZED = 0,
    WIRELESS_STATE_IDLE,
    WIRELESS_STATE_AP_MODE_INIT,
    WIRELESS_STATE_AP_MODE_LISTEN,
    WIRELESS_STATE_STA_MODE_INIT,
    WIRELESS_STATE_STA_MODE_LISTEN,
} wireless_state_t;

typedef enum {
    WIRELESS_CTRL_NOP = 0,
    WIRELESS_CTRL_CYW43_INIT,
    WIRELESS_CTRL_CYW43_DEINIT,
    WIRELESS_CTRL_DISCONNECT,
    WRIELESS_CTRL_START_AP_MODE,
    WRIELESS_CTRL_START_STA_MODE,
    WIRELESS_CTRL_LED_ON,
    WIRELESS_CTRL_LED_OFF,
} wireless_ctrl_t;


typedef struct {
    eeprom_wireless_metadata_t eeprom_wireless_metadata;
    wireless_state_t current_wireless_state;
} wireless_config_t;


static wireless_config_t wireless_config;
const eeprom_wireless_metadata_t default_eeprom_wireless_metadata = {
    .wireless_data_rev = 0,
    .ssid = "",
    .pw = "",
    .auth = AUTH_WPA2_MIXED_PSK,
    .timeout_ms = 30000,    // 30s
    .enable = false,
};

static eeprom_wireless_extra_t wireless_extra;
static const eeprom_wireless_extra_t default_eeprom_wireless_extra = {0};

static QueueHandle_t wireless_ctrl_queue;

// Render task
const char * wireless_state_strings[] = {
    "Not Initialized",
    "Idling",
    "AP Mode Init",
    "AP Mode Listen",
    "STA Mode Init",
    "STA Mode Listen"
};


char first_line_buffer[35];
char second_line_buffer[35];


void wirelss_info_render_task(void *p) {
    u8g2_t * display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        u8g2_ClearBuffer(display_handler);

        // Draw state in the title
        const char * title_string = wireless_state_strings[wireless_config.current_wireless_state];
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);
        u8g2_DrawStr(display_handler, 5, 10, title_string);

        // Draw line
        u8g2_DrawHLine(display_handler, 0, 13, u8g2_GetDisplayWidth(display_handler));

        // Draw IP address
        char * ip_addr_string = ipaddr_ntoa(netif_ip4_addr(netif_default));
        if (strlen(ip_addr_string)) {
            u8g2_SetFont(display_handler, u8g2_font_6x12_tf);
            u8g2_DrawStr(display_handler, 5, 23, ip_addr_string);
        }

        // Draw first line
        if (strlen(first_line_buffer)) {
            u8g2_SetFont(display_handler, u8g2_font_6x12_tf);
            u8g2_DrawStr(display_handler, 5, 33, first_line_buffer);
        }

        // Draw second line
        if (strlen(second_line_buffer)) {
            u8g2_SetFont(display_handler, u8g2_font_6x12_tf);
            u8g2_DrawStr(display_handler, 5, 43, second_line_buffer);
        }

        // Draw link status
        if (wireless_config.current_wireless_state == WIRELESS_STATE_STA_MODE_INIT || 
            wireless_config.current_wireless_state == WIRELESS_STATE_STA_MODE_LISTEN) {
                int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
                char * link_status_string = NULL;

                if (link_status == CYW43_LINK_DOWN) {
                    link_status_string = "LINK_DOWN";
                }
                else if (link_status == CYW43_LINK_JOIN) {
                    link_status_string = "CYW43_LINK_JOIN";
                }
                else if (link_status == CYW43_LINK_NOIP) {
                    link_status_string = "CYW43_LINK_NOIP";
                }
                else if (link_status == CYW43_LINK_UP) {
                    link_status_string = "CYW43_LINK_UP";
                }
                else if (link_status == CYW43_LINK_FAIL) {
                    link_status_string = "CYW43_LINK_FAIL";
                }
                else if (link_status == CYW43_LINK_NONET) {
                    link_status_string = "CYW43_LINK_NONET";
                }
                else if (link_status == CYW43_LINK_BADAUTH) {
                    link_status_string = "CYW43_LINK_BADAUTH";
                }
                u8g2_SetFont(display_handler, u8g2_font_6x12_tf);
                u8g2_DrawStr(display_handler, 5, 53, link_status_string);
            }


        u8g2_SendBuffer(display_handler);

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(200));
    }
}



bool wireless_init() {
    bool is_ok = true;

    memset(&wireless_config, 0x00, sizeof(wireless_config_t));
    is_ok = load_config(EEPROM_WIRELESS_CONFIG_BASE_ADDR, &wireless_config.eeprom_wireless_metadata, &default_eeprom_wireless_metadata, sizeof(wireless_config.eeprom_wireless_metadata), EEPROM_WIRELESS_CONFIG_METADATA_REV);
    if (!is_ok) {
        printf("Unable to read WiFi configuration\n");
        return false;
    }

    // Additional networks (separate block, defaults to empty on first boot)
    memset(&wireless_extra, 0x00, sizeof(wireless_extra));
    if (!load_config(EEPROM_WIRELESS_EXTRA_BASE_ADDR, &wireless_extra, &default_eeprom_wireless_extra,
                     sizeof(wireless_extra), EEPROM_WIRELESS_EXTRA_REV)) {
        printf("Unable to read additional WiFi networks\n");
        memset(&wireless_extra, 0x00, sizeof(wireless_extra));
    }
    for (int i = 0; i < WIRELESS_EXTRA_NETWORK_CNT; i++) {
        wireless_extra.networks[i].ssid[sizeof(wireless_extra.networks[i].ssid) - 1] = '\0';
        wireless_extra.networks[i].pw[sizeof(wireless_extra.networks[i].pw) - 1] = '\0';
    }

    // Create Wireless handler task
    xTaskCreate(wireless_task, "Wireless Task", 1024, NULL, 3, NULL);

    // Register to eeprom save all
    eeprom_register_handler(wireless_config_save);

    // Generate the hostname
    char id[4];
    eeprom_get_board_id(id, sizeof(id));
    snprintf(host_name, sizeof(host_name), "opentrickler-%s", id);


    return is_ok;
}


bool wireless_config_save(void) {
    bool is_ok = save_config(EEPROM_WIRELESS_CONFIG_BASE_ADDR, &wireless_config.eeprom_wireless_metadata, sizeof(eeprom_wireless_metadata_t));
    is_ok = save_config(EEPROM_WIRELESS_EXTRA_BASE_ADDR, &wireless_extra, sizeof(wireless_extra)) && is_ok;
    return is_ok;
}


uint32_t get_cyw43_auth(cyw43_auth_t auth) {
    uint32_t cyw43_auth = 0;

    switch (auth) {
        case AUTH_OPEN:
            cyw43_auth = CYW43_AUTH_OPEN;
            break;
        case AUTH_WPA_TKIP_PSK:
            cyw43_auth = CYW43_AUTH_WPA_TKIP_PSK;
            break;
        case AUTH_WPA2_AES_PSK:
            cyw43_auth = CYW43_AUTH_WPA2_AES_PSK;
            break;
        case AUTH_WPA2_MIXED_PSK:
            cyw43_auth = CYW43_AUTH_WPA2_MIXED_PSK;
            break;
        default:
            break;
    }

    return cyw43_auth;
}


void led_interface_task(void *p) {
    wireless_ctrl_t wireless_ctrl;
    uint32_t blink_interval_ms = 0;
    bool prev_led_state = false;
    bool led_state = prev_led_state;

    while (true) {
        prev_led_state = led_state;

        switch (wireless_config.current_wireless_state) {
            case WIRELESS_STATE_NOT_INITIALIZED:
            case WIRELESS_STATE_IDLE:
                blink_interval_ms = 0;
                led_state = false;
                break;
            case WIRELESS_STATE_AP_MODE_INIT:
                led_state = !led_state;
                blink_interval_ms = 20;
                break;
            case WIRELESS_STATE_AP_MODE_LISTEN:
                led_state = !led_state;
                blink_interval_ms = 1000;
                break;
            case WIRELESS_STATE_STA_MODE_INIT:
                led_state = !led_state;
                blink_interval_ms = 20;
                break;
            case WIRELESS_STATE_STA_MODE_LISTEN:
                led_state = true;
                blink_interval_ms = 0;
                break;
            default:
                break;
        }

        if (led_state != prev_led_state) {
            if (led_state) {
                wireless_ctrl = WIRELESS_CTRL_LED_ON;
            }
            else {
                wireless_ctrl = WIRELESS_CTRL_LED_OFF;
            }
            xQueueSend(wireless_ctrl_queue, &wireless_ctrl, 0);
        }

        
        vTaskDelay(pdMS_TO_TICKS(MAX(blink_interval_ms, LED_INTERFACE_MINIMUM_POLL_PERIOD_MS)));
    }
}


static void srv_txt(struct mdns_service *service, void *txt_userdata)
{
  err_t res;
  LWIP_UNUSED_ARG(txt_userdata);

  res = mdns_resp_add_service_txtitem(service, "path=/", 6);
  LWIP_ERROR("mdns add service txt failed\n", (res == ERR_OK), return);
}

// ---------------------------------------------------------------------------
// Multiple known networks
// ---------------------------------------------------------------------------
#define WIRELESS_KNOWN_NETWORK_CNT  (1 + WIRELESS_EXTRA_NETWORK_CNT)
#define WIRELESS_SCAN_TIMEOUT_MS    8000
#define WIRELESS_RSSI_NOT_SEEN      (-1000)

typedef struct {
    const char *ssid;
    const char *pw;
    uint8_t auth;
    int8_t slot;            // 0 = network 1 (primary), 1..4 = additional
} wireless_candidate_t;

static int8_t wireless_connected_slot = -1;   // slot of the joined network, -1 if none

static int16_t scan_best_rssi[WIRELESS_KNOWN_NETWORK_CNT];
static wireless_candidate_t scan_candidates[WIRELESS_KNOWN_NETWORK_CNT];
static int scan_candidate_cnt;


// Network n (0 = the original/primary network, 1..4 = additional ones).
static bool wireless_get_known_network(int n, wireless_candidate_t *out) {
    if (n == 0) {
        out->ssid = wireless_config.eeprom_wireless_metadata.ssid;
        out->pw = wireless_config.eeprom_wireless_metadata.pw;
        out->auth = (uint8_t) wireless_config.eeprom_wireless_metadata.auth;
    }
    else if (n >= 1 && n <= WIRELESS_EXTRA_NETWORK_CNT) {
        out->ssid = wireless_extra.networks[n - 1].ssid;
        out->pw = wireless_extra.networks[n - 1].pw;
        out->auth = wireless_extra.networks[n - 1].auth;
    }
    else {
        return false;
    }
    return out->ssid[0] != '\0';
}


static int wireless_scan_result_cb(void *env, const cyw43_ev_scan_result_t *result) {
    (void) env;
    if (result == NULL || result->ssid_len == 0 || result->ssid_len > 32) {
        return 0;
    }
    for (int i = 0; i < scan_candidate_cnt; i++) {
        const char *ssid = scan_candidates[i].ssid;
        size_t len = strlen(ssid);
        if (len == result->ssid_len && memcmp(ssid, result->ssid, len) == 0) {
            if (result->rssi > scan_best_rssi[i]) {
                scan_best_rssi[i] = result->rssi;
            }
        }
    }
    return 0;
}


// Try every configured network: the ones seen in a scan first (strongest
// signal first), then the ones not seen (hidden SSIDs, or the scan failed) in
// list order. Each attempt uses the configured connect timeout.
static bool wireless_connect_known_networks(void) {
    scan_candidate_cnt = 0;
    for (int n = 0; n < WIRELESS_KNOWN_NETWORK_CNT; n++) {
        wireless_candidate_t c;
        if (wireless_get_known_network(n, &c)) {
            c.slot = (int8_t) n;
            scan_candidates[scan_candidate_cnt] = c;
            scan_best_rssi[scan_candidate_cnt] = WIRELESS_RSSI_NOT_SEEN;
            scan_candidate_cnt++;
        }
    }
    if (scan_candidate_cnt == 0) {
        return false;
    }

    // Scan only when there is a choice to make.
    bool scan_ok = false;
    if (scan_candidate_cnt > 1) {
        snprintf(first_line_buffer, sizeof(first_line_buffer), ">Scanning...");
        cyw43_wifi_scan_options_t scan_options = {0};
        if (cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, wireless_scan_result_cb) == 0) {
            TickType_t scan_stop = xTaskGetTickCount() + pdMS_TO_TICKS(WIRELESS_SCAN_TIMEOUT_MS);
            while (cyw43_wifi_scan_active(&cyw43_state) && xTaskGetTickCount() < scan_stop) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            scan_ok = !cyw43_wifi_scan_active(&cyw43_state);
        }
    }

    // Order: seen networks by RSSI (desc), then unseen in list order.
    int order[WIRELESS_KNOWN_NETWORK_CNT];
    for (int i = 0; i < scan_candidate_cnt; i++) {
        order[i] = i;
    }
    for (int i = 1; i < scan_candidate_cnt; i++) {   // stable insertion sort
        int k = order[i];
        int j = i - 1;
        while (j >= 0 && scan_best_rssi[order[j]] < scan_best_rssi[k]) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = k;
    }

    uint32_t timeout_ms = wireless_config.eeprom_wireless_metadata.timeout_ms;
    if (timeout_ms < 5000) {
        timeout_ms = 5000;
    }

    for (int i = 0; i < scan_candidate_cnt; i++) {
        const wireless_candidate_t *c = &scan_candidates[order[i]];

        snprintf(first_line_buffer, sizeof(first_line_buffer), ">%s", c->ssid);
        snprintf(second_line_buffer, sizeof(second_line_buffer), "Network %d/%d", i + 1, scan_candidate_cnt);

        // If the authentication method is open then the password is NULL
        const char *wifi_password = (c->auth != AUTH_OPEN) ? c->pw : NULL;

        // A network the (successful) scan did not see is most likely out of
        // range; give it one short window (it may still be a hidden SSID)
        // so a list of absent networks doesn't delay the AP fallback.
        uint32_t this_timeout_ms = timeout_ms;
        if (scan_ok && scan_best_rssi[order[i]] == WIRELESS_RSSI_NOT_SEEN && this_timeout_ms > 10000) {
            this_timeout_ms = 10000;
        }

        // Same as the original single-network behaviour: keep retrying this
        // network until its timeout window is used up (a join can fail fast,
        // e.g. while the AP is still coming up).
        TickType_t stop_tick = xTaskGetTickCount() + pdMS_TO_TICKS(this_timeout_ms);
        while (xTaskGetTickCount() < stop_tick) {
            uint32_t remaining_ms = (uint32_t) ((stop_tick - xTaskGetTickCount()) * portTICK_PERIOD_MS);
            if (remaining_ms < 1000) {
                break;
            }
            int resp = cyw43_arch_wifi_connect_timeout_ms(c->ssid,
                                                          wifi_password,
                                                          get_cyw43_auth((cyw43_auth_t) c->auth),
                                                          remaining_ms);
            if (resp == PICO_OK) {
                memset(second_line_buffer, 0x0, sizeof(second_line_buffer));
                wireless_connected_slot = c->slot;
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        // Make sure a half-finished join doesn't carry over into the next one
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    memset(second_line_buffer, 0x0, sizeof(second_line_buffer));
    return false;
}


void wireless_task(void *p) {
    static TaskHandle_t led_interface_task_handler = NULL;

    memset(first_line_buffer, 0x0, sizeof(first_line_buffer));
    memset(second_line_buffer, 0x0, sizeof(second_line_buffer));

    wireless_config.current_wireless_state = WIRELESS_STATE_NOT_INITIALIZED;
    wireless_ctrl_queue = xQueueCreate(5, sizeof(wireless_ctrl_t));

    if (cyw43_arch_init()) {
        exit(-1);
    }

    wireless_config.current_wireless_state = WIRELESS_STATE_IDLE;

    // Create LED task
    if (led_interface_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(led_interface_task, "LED Interface Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &led_interface_task_handler);
    }
    else {
        vTaskResume(led_interface_task_handler);
    }

    // Start default initialize pattern
    if (wireless_config.eeprom_wireless_metadata.enable) {
        wireless_config.current_wireless_state = WIRELESS_STATE_STA_MODE_INIT;
        cyw43_arch_enable_sta_mode();

        if (wireless_connect_known_networks()) {
            wireless_config.current_wireless_state = WIRELESS_STATE_STA_MODE_LISTEN;
        }
    }

    // If the state didn't change (connection failed) then we shall put it back to idle
    if (wireless_config.current_wireless_state == WIRELESS_STATE_STA_MODE_INIT) {
         wireless_config.current_wireless_state = WIRELESS_STATE_IDLE;
    }


    // If not configured, or failed to connect existing wifi then start the AP mode
    if (wireless_config.current_wireless_state == WIRELESS_STATE_IDLE) {
        // If previous configured, then start STA mode by default
        wireless_config.current_wireless_state = WIRELESS_STATE_AP_MODE_INIT;
        access_point_mode_start();
        wireless_config.current_wireless_state = WIRELESS_STATE_AP_MODE_LISTEN;
    }
    else {
        cyw43_arch_lwip_begin();
        // If the STA mode is successfully initialized, we will also start the mdns service
        mdns_resp_init();

        // The opentrickler can be accessed via 
        mdns_resp_add_netif(&cyw43_state.netif[CYW43_ITF_STA], host_name);

        // Add service HTTP, allowing user to access the web interface with hostname.local format
        mdns_resp_add_service(&cyw43_state.netif[CYW43_ITF_STA], "rest_httpd", "_http", DNSSD_PROTO_TCP, 80, srv_txt, NULL);

        // Add secondary service to allow client to discover with service _opentrickler._tcp.local
        mdns_resp_add_service(&cyw43_state.netif[CYW43_ITF_STA], "app_httpd", "_opentrickler", DNSSD_PROTO_TCP, 80, srv_txt, NULL);
        cyw43_arch_lwip_end();
    }

    // Initialize REST endpoints
    // If the current wireless state is AP mode then we will map / to the wifi configuration
    rest_endpoints_init(wireless_config.current_wireless_state == WIRELESS_STATE_AP_MODE_LISTEN);

    // Start the HTTP server
    cyw43_arch_lwip_begin();
    httpd_init();
    cyw43_arch_lwip_end();

    while (true) {
        wireless_ctrl_t wireless_ctrl;

        xQueueReceive(wireless_ctrl_queue, &wireless_ctrl, portMAX_DELAY);

        switch (wireless_ctrl) {
            case WIRELESS_CTRL_LED_ON:
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
                break;
            case WIRELESS_CTRL_LED_OFF:
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
                break;
            default:
                break;
        }
    }

    mdns_resp_remove_netif(&cyw43_state.netif[CYW43_ITF_STA]);
    cyw43_arch_deinit();

    vTaskSuspend(led_interface_task_handler);
}


uint8_t wireless_view_wifi_info(void) {
    static TaskHandle_t wirelss_info_render_task_handler = NULL;
    if (wirelss_info_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(wirelss_info_render_task, "Wireless Display Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &wirelss_info_render_task_handler);
    }
    else {
        vTaskResume(wirelss_info_render_task_handler);
    }

    bool quit = false;
    while (quit == false) {
        // Wait if quit is pressed
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(true);
        if (button_encoder_event == BUTTON_RST_PRESSED || button_encoder_event == BUTTON_ENCODER_PRESSED) {
            quit = true;
            break;
        }
    }

    vTaskSuspend(wirelss_info_render_task_handler);

    return 40;  // Returns to the Wireless menu (view 40)
}


// JSON-escape a string into dst (quotes, backslash, control characters).
static void wireless_json_escape(char *dst, size_t dst_len, const char *src) {
    size_t o = 0;
    for (const unsigned char *c = (const unsigned char *) src; *c && o + 7 < dst_len; c++) {
        if (*c == '"' || *c == '\\') {
            dst[o++] = '\\';
            dst[o++] = (char) *c;
        }
        else if (*c < 0x20) {
            o += (size_t) snprintf(dst + o, dst_len - o, "\\u%04x", *c);
        }
        else {
            dst[o++] = (char) *c;
        }
    }
    dst[o] = '\0';
}


// Parses "x<n><field>" (n = 1..WIRELESS_EXTRA_NETWORK_CNT, field = s/p/a).
static bool wireless_parse_extra_param(const char *param, int *slot, char *field) {
    if (param[0] != 'x' || param[1] < '1' || param[1] > '0' + WIRELESS_EXTRA_NETWORK_CNT ||
        param[2] == '\0' || param[3] != '\0') {
        return false;
    }
    *slot = param[1] - '1';
    *field = param[2];
    return true;
}


bool http_rest_wireless_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mapping
    // w0 (str): ssid             (network 1)
    // w1 (str): pw               (network 1, write only)
    // w2 (int): auth             (network 1)
    // w3 (int): timeout_ms       (per network)
    // w4 (bool): enable
    // x<n>s (str): ssid          (additional network n = 1..4)
    // x<n>p (str): pw            (additional network n, write only)
    // x<n>a (int): auth          (additional network n)
    // xc (int): forget network slot n (0 = network 1, 1..4 = additional)
    // ee (bool): save to eeprom

    static char wireless_config_json_buffer[1024];
    bool save_to_eeprom = false;

    // If the argument includes control, then update the settings
    for (int idx = 0; idx < num_params; idx += 1) {
        int slot;
        char field;

        if (strcmp(params[idx], "w0") == 0) {
            strncpy(wireless_config.eeprom_wireless_metadata.ssid, values[idx],
                    sizeof(wireless_config.eeprom_wireless_metadata.ssid) - 1);
            wireless_config.eeprom_wireless_metadata.ssid[sizeof(wireless_config.eeprom_wireless_metadata.ssid) - 1] = '\0';
        }
        else if (strcmp(params[idx], "w1") == 0) {
            strncpy(wireless_config.eeprom_wireless_metadata.pw, values[idx],
                    sizeof(wireless_config.eeprom_wireless_metadata.pw) - 1);
            wireless_config.eeprom_wireless_metadata.pw[sizeof(wireless_config.eeprom_wireless_metadata.pw) - 1] = '\0';
        }
        else if (strcmp(params[idx], "w2") == 0) {
            cyw43_auth_t auth = (cyw43_auth_t) atoi(values[idx]);
            wireless_config.eeprom_wireless_metadata.auth = auth;
        }
        else if (strcmp(params[idx], "w3") == 0) {
            long timeout_ms = strtol(values[idx], NULL, 10);
            if (timeout_ms < 5000) timeout_ms = 5000;
            if (timeout_ms > 120000) timeout_ms = 120000;
            wireless_config.eeprom_wireless_metadata.timeout_ms = (uint32_t) timeout_ms;
        }
        else if (strcmp(params[idx], "w4") == 0) {
            bool enable = string_to_boolean(values[idx]);
            wireless_config.eeprom_wireless_metadata.enable = enable;
        }
        else if (strcmp(params[idx], "xc") == 0) {
            int n = atoi(values[idx]);
            if (n == 0) {
                // Forget network 1 (the one the setup hotspot writes)
                memset(wireless_config.eeprom_wireless_metadata.ssid, 0x00, sizeof(wireless_config.eeprom_wireless_metadata.ssid));
                memset(wireless_config.eeprom_wireless_metadata.pw, 0x00, sizeof(wireless_config.eeprom_wireless_metadata.pw));
                wireless_config.eeprom_wireless_metadata.auth = AUTH_WPA2_MIXED_PSK;
            }
            else if (n >= 1 && n <= WIRELESS_EXTRA_NETWORK_CNT) {
                memset(&wireless_extra.networks[n - 1], 0x00, sizeof(wireless_extra.networks[n - 1]));
                wireless_extra.networks[n - 1].auth = AUTH_WPA2_MIXED_PSK;
            }
        }
        else if (wireless_parse_extra_param(params[idx], &slot, &field)) {
            wireless_network_t *net = &wireless_extra.networks[slot];
            if (field == 's') {
                strncpy(net->ssid, values[idx], sizeof(net->ssid) - 1);
                net->ssid[sizeof(net->ssid) - 1] = '\0';
            }
            else if (field == 'p') {
                strncpy(net->pw, values[idx], sizeof(net->pw) - 1);
                net->pw[sizeof(net->pw) - 1] = '\0';
            }
            else if (field == 'a') {
                int auth = atoi(values[idx]);
                if (auth >= AUTH_OPEN && auth <= AUTH_WPA2_MIXED_PSK) {
                    net->auth = (uint8_t) auth;
                }
            }
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    // Perform action
    if (save_to_eeprom) {
        if (!wireless_config_save()) {
            snprintf(wireless_config_json_buffer,
                     sizeof(wireless_config_json_buffer),
                     "%s{\"error\":\"WirelessConfigSaveFailed\"}",
                     http_json_header);

            size_t data_length = strlen(wireless_config_json_buffer);
            file->data = wireless_config_json_buffer;
            file->len = data_length;
            file->index = data_length;
            file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
            return true;
        }
    }

    // Response (passwords are never sent back)
    char escaped[2 * 32 + 8];
    wireless_json_escape(escaped, sizeof(escaped), wireless_config.eeprom_wireless_metadata.ssid);

    int len = snprintf(wireless_config_json_buffer,
             sizeof(wireless_config_json_buffer),
             "%s"
             "{\"w0\":\"%s\",\"w2\":%d,\"w3\":%"PRId32",\"w4\":%s,\"xn\":%d",
             http_json_header,
             escaped,
             wireless_config.eeprom_wireless_metadata.auth,
             wireless_config.eeprom_wireless_metadata.timeout_ms,
             boolean_to_string(wireless_config.eeprom_wireless_metadata.enable),
             WIRELESS_EXTRA_NETWORK_CNT);

    for (int n = 0; n < WIRELESS_EXTRA_NETWORK_CNT && len > 0 && (size_t) len < sizeof(wireless_config_json_buffer); n++) {
        const wireless_network_t *net = &wireless_extra.networks[n];
        wireless_json_escape(escaped, sizeof(escaped), net->ssid);
        len += snprintf(wireless_config_json_buffer + len, sizeof(wireless_config_json_buffer) - len,
                        ",\"x%ds\":\"%s\",\"x%da\":%d",
                        n + 1, escaped, n + 1, net->ssid[0] ? net->auth : AUTH_WPA2_MIXED_PSK);
    }
    // cs: slot of the network joined at boot (-1 = none / AP mode)
    // pk: per slot whether a password is stored (the password itself is never sent)
    if (len > 0 && (size_t) len < sizeof(wireless_config_json_buffer)) {
        len += snprintf(wireless_config_json_buffer + len, sizeof(wireless_config_json_buffer) - len,
                        ",\"cs\":%d,\"pk\":[%d",
                        (int) wireless_connected_slot,
                        wireless_config.eeprom_wireless_metadata.pw[0] != '\0');
        for (int n = 0; n < WIRELESS_EXTRA_NETWORK_CNT && len > 0 && (size_t) len < sizeof(wireless_config_json_buffer); n++) {
            len += snprintf(wireless_config_json_buffer + len, sizeof(wireless_config_json_buffer) - len,
                            ",%d", wireless_extra.networks[n].pw[0] != '\0');
        }
        if (len > 0 && (size_t) len < sizeof(wireless_config_json_buffer)) {
            len += snprintf(wireless_config_json_buffer + len, sizeof(wireless_config_json_buffer) - len, "]");
        }
    }
    if (len > 0 && (size_t) len < sizeof(wireless_config_json_buffer) - 2) {
        wireless_config_json_buffer[len++] = '}';
        wireless_config_json_buffer[len] = '\0';
    }

    size_t data_length = strlen(wireless_config_json_buffer);
    file->data = wireless_config_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
