/*
 * Copyright (c) 2019, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

#ifndef PYCOM_CONFIG_H_
#define PYCOM_CONFIG_H_

#include "py/mpconfig.h"


/******************************************************************************
 DEFINE CONSTANTS
 ******************************************************************************/

/******************************************************************************
 DEFINE TYPES
 ******************************************************************************/

typedef struct {
    uint8_t lpwan_mac[8];
    uint8_t sigfox_id[4];
    uint8_t sigfox_pac[8];
    uint8_t sigfox_private_key[16];
    uint8_t sigfox_public_key[16];
    uint8_t lora_region;
} pycom_lpwan_config_t;

typedef struct {
    uint8_t wifi_on_boot :1;
    uint8_t wifi_mode :2;
    uint8_t wifi_smart_config:1;
    uint8_t wifi_auth :3;
    uint8_t wifi_antenna :1;
} pycom_wifi_config_t;

typedef struct {
    uint8_t wifi_ssid[33];
    uint8_t wifi_pwd[65];
} pycom_wifi_sta_config_t;

typedef struct {
    uint8_t wifi_ssid[33];
    uint8_t wifi_pwd[65];
} pycom_wifi_ap_config_t;

typedef struct {
    uint8_t heartbeat_on_boot :1;
    uint8_t rgb_error_color[3];
    uint8_t rgb_safeboot_color[3];
    uint8_t rgb_heartbeat_color[3];
} pycom_rgbled_config_t;

typedef struct {
    uint8_t device_token[40];
    uint8_t mqttServiceAddress[40];
    uint8_t userId[100];
    uint8_t network_preferences[55];
    uint8_t extra_preferences[100];
    uint8_t force_update;
    uint8_t reserved[12];
} pycom_pybytes_config_t;

typedef struct {
    uint8_t sw_version[12];
    uint8_t boot_fs_type;
    uint8_t boot_partition;
    uint8_t hw_type;
} pycom_config_t;

typedef struct {
    uint8_t wdt_on_boot;
    uint32_t wdt_on_boot_timeout;
} pycom_wdt_config_t;

typedef struct {
    uint8_t lte_modem_en_on_boot;
} pycom_lte_config_t;

typedef struct {
	uint8_t carrier[129];
	uint8_t apn[129];
	uint8_t type[17];
	uint8_t cid;
	uint8_t band;
	uint8_t reset;
} pycom_pybytes_lte_config_t;

typedef struct {
    pycom_lpwan_config_t lpwan_config;
    pycom_wifi_config_t wifi_config;
    pycom_wifi_sta_config_t wifi_sta_config;
    pycom_rgbled_config_t rgbled_config;
    pycom_pybytes_config_t pybytes_config;
    pycom_wdt_config_t wdt_config;
    pycom_lte_config_t lte_config;
    pycom_config_t pycom_config;
    pycom_wifi_ap_config_t wifi_ap_config;
    pycom_pybytes_lte_config_t pycom_pybytes_lte_config;
    uint8_t pycom_reserved[112];
} pycom_config_block_t;

typedef enum
{
    PYCOM_WIFI_CONF_MODE_STA = 0,
    PYCOM_WIFI_CONF_MODE_AP,
    PYCOM_WIFI_CONF_MODE_APSTA,
    PYCOM_WIFI_CONF_MODE_NONE
}pycom_config_wifi_mode_t;

/******************************************************************************
 DECLARE PUBLIC FUNCTIONS
 ******************************************************************************/
void config_init0(void);

bool config_set_lpwan_mac(const uint8_t *mac);

void config_get_lpwan_mac(uint8_t *mac);

bool config_set_sigfox_id(uint8_t *id);

void config_get_sigfox_id(uint8_t *id);

bool config_set_sigfox_pac(uint8_t *pac);

void config_get_sigfox_pac(uint8_t *pac);

bool config_set_sigfox_public_key(uint8_t *public_key);

void config_get_sigfox_public_key(uint8_t *public_key);

bool config_set_sigfox_private_key(uint8_t *private_key);

void config_get_sigfox_private_key(uint8_t *private_key);

bool config_set_wifi_on_boot(uint8_t wifi_on_boot);

bool config_get_wifi_on_boot(void);

bool config_set_wifi_mode(uint8_t wifi_mode, bool update_flash);

uint8_t config_get_wifi_mode(void);

bool config_set_wifi_auth(uint8_t wifi_auth, bool update_flash);

uint8_t config_get_wifi_auth(void);

bool config_get_wifi_antenna(void);

bool config_set_wdt_on_boot (uint8_t wdt_on_boot);

bool config_set_wifi_smart_config (uint8_t smartConfig, bool update_flash);

bool config_get_wifi_smart_config(void);

bool config_get_wdt_on_boot (void);

bool config_set_wdt_on_boot_timeout (uint32_t wdt_on_boot_timeout);

uint32_t config_get_wdt_on_boot_timeout (void);

bool config_set_heartbeat_on_boot(uint8_t wifi_on_boot);

bool config_get_heartbeat_on_boot(void);

bool config_set_sta_wifi_ssid(const uint8_t *wifi_ssid, bool update_flash);

bool config_get_wifi_sta_ssid(uint8_t *wifi_ssid);

bool config_set_wifi_sta_pwd(const uint8_t *wifi_pwd, bool update_flash);

bool config_get_wifi_sta_pwd(uint8_t *wifi_pwd);

bool config_set_wifi_ap_ssid(const uint8_t *wifi_ssid);

bool config_get_wifi_ap_ssid(uint8_t *wifi_ssid);

bool config_set_wifi_ap_pwd(const uint8_t *wifi_pwd);

bool config_get_wifi_ap_pwd(uint8_t *wifi_pwd);

bool config_set_lora_region (uint8_t lora_region);

uint8_t config_get_lora_region (void);

void config_get_pybytes_device_token (uint8_t *pybytes_device_token);

void config_get_pybytes_mqttServiceAddress (uint8_t *pybytes_mqttServiceAddress);

#if (VARIANT == PYBYTES)
void config_get_pybytes_userId (uint8_t *pybytes_userId);

void config_get_pybytes_network_preferences (uint8_t *pybytes_userId);

void config_get_pybytes_extra_preferences (uint8_t *pybytes_userId);

bool config_set_pybytes_force_update (uint8_t force_update);

bool config_get_pybytes_force_update (void);
#endif

uint8_t config_get_boot_fs_type (void);

bool config_set_boot_fs_type (const uint8_t boot_fs_type);

pycom_pybytes_lte_config_t config_get_pybytes_lte_config (void);

bool config_set_pybytes_lte_config(const pycom_pybytes_lte_config_t pycom_pybytes_lte_config);

uint8_t config_get_boot_partition (void);

bool config_set_boot_partition (const uint8_t boot_partition);

bool config_set_lte_modem_enable_on_boot (bool lte_modem_en_on_boot);

bool config_get_lte_modem_enable_on_boot (void);

#endif /* PYCOM_CONFIG_H_ */
