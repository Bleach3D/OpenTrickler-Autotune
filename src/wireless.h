#ifndef WIRELESS_H_
#define WIRELESS_H_

#include <stdint.h>
#include "http_rest.h"
#include "eeprom.h"

#define EEPROM_WIRELESS_CONFIG_METADATA_REV                     2              // 16 byte 


typedef enum {
    AUTH_OPEN = 0,
    AUTH_WPA_TKIP_PSK = 1,
    AUTH_WPA2_AES_PSK = 2,
    AUTH_WPA2_MIXED_PSK = 3,
} cyw43_auth_t;


typedef struct {
    uint16_t wireless_data_rev;
    char ssid[32];
    char pw[64];
    cyw43_auth_t auth;
    uint32_t timeout_ms;
    bool enable;
} eeprom_wireless_metadata_t;


// Additional known networks (network 1 is eeprom_wireless_metadata_t above,
// kept unchanged so existing configurations and the AP-mode setup wizard keep
// working). Stored in its own CRC-checked block inside the 2K wireless region.
#define WIRELESS_EXTRA_NETWORK_CNT              4
#define EEPROM_WIRELESS_EXTRA_BASE_ADDR         (EEPROM_WIRELESS_CONFIG_BASE_ADDR + 512)
#define EEPROM_WIRELESS_EXTRA_REV               0x5A17

typedef struct {
    char ssid[32];
    char pw[64];
    uint8_t auth;          // cyw43_auth_t
    uint8_t _reserved[3];
} wireless_network_t;

typedef struct {
    uint16_t wireless_extra_rev;
    uint16_t _reserved;
    wireless_network_t networks[WIRELESS_EXTRA_NETWORK_CNT];
} eeprom_wireless_extra_t;





#ifdef __cplusplus
extern "C" {
#endif


void wireless_task(void *);
bool wireless_init(void);
bool wireless_config_save(void);
uint8_t wireless_view_wifi_info(void);

bool http_rest_wireless_config(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif


#endif  // WIRELESS_H_
