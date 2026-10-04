#ifndef OTA_CLIENT_H_
#define OTA_CLIENT_H_

#include <stdint.h>
#include <stdbool.h>

#include "http_rest.h"

// HTTPS update client: the device checks github.com's stable
// ".../releases/latest/download/<asset>" URLs directly (no manifest server
// of your own to run, no maintainer secrets to manage) for a newer release
// of this firmware, and downloads the matching app.bin straight into the
// existing OTA staging area (ota_update.c) when asked to. Both the check
// and the download/apply run in a background task; nothing here ever runs
// unattended -- "Check for Updates" and "Update Now" are both explicit
// actions taken from the web GUI. See ota_tls.c for the TLS client and
// ota_tls_roots.c for the trust anchors used to verify GitHub's cert chain.

#define EEPROM_UPDATE_CONFIG_REV    1

typedef struct {
    uint16_t update_config_rev;
    char owner[40];   // GitHub org/user, e.g. "Bleach3D"
    char repo[64];    // GitHub repository name, e.g. "OpenTrickler-Firmware"
} eeprom_update_config_t;

#ifdef __cplusplus
extern "C" {
#endif

bool ota_client_init(void);
bool ota_client_config_save(void);

bool http_rest_update_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_update_check(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_update_status(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_update_apply(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // OTA_CLIENT_H_
