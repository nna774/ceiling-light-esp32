/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <esp_matter.h>
#include <app_priv.h>
#include <common_macros.h>

#include <device.h>
#include <button_gpio.h>

#include <esp_mac.h>
#include <psa/crypto.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "host/ble_hs.h"
#include "host/ble_gap.h"

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

// ---------------------------------------------------------------------
// LampSmart Pro / FanLamp Pro raw BLE advertising protocol ("other/v3"
// FanLampEncoderV2 variant), ported from the standalone console firmware
// at ~/codes/misk/light/main/main.c (see that repo's NOTES.md for how this
// was reverse engineered from the physical remote).
//
// This only sends plain on/off -- pairing was already done with this same
// chip's BT MAC address by the earlier console firmware, and the lamp
// remembers pairings by that identifier, so no re-pairing is needed here.
//
// BLE lifecycle note: CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING is disabled for
// this build, so Matter's NimBLE stack stays initialized for the life of the
// device instead of being torn down after commissioning. We piggyback on
// that same already-running host (ble_gap_adv_set_data/start/stop) rather
// than bringing up our own -- re-initializing NimBLE after
// esp_bt_mem_release() (which USE_BLE_ONLY_FOR_COMMISSIONING triggers) reuses
// memory the allocator has already handed out elsewhere, which corrupted the
// BT controller's internal state ("HLI Magic mismatch") and crashed on real
// hardware.
// ---------------------------------------------------------------------

#define CMD_TURN_ON 0x10
#define CMD_TURN_OFF 0x11
#define CMD_DIM 0x21 // LIGHT_WCOLOR: args[2]=cold(0-255), args[3]=warm(0-255)
#define DEVICE_TYPE 0x0100
#define GROUP_INDEX 0x00
#define REPEAT_COUNT 15

// This lamp has no datasheet-specified color temperature range, so pick a
// conventional CCT bulb range (2700K-6500K) and interpolate cold/warm channel
// balance linearly in mireds -- matches how the two-channel LED driver in
// this lamp actually works (no true per-Kelvin curve, just a channel mix).
#define LAMP_MIREDS_MIN 154 // ~6500K, coolest (max cold channel)
#define LAMP_MIREDS_MAX 370 // ~2700K, warmest (max warm channel)

static const uint8_t PREFIX[3] = {0x10, 0x80, 0x00};
static const uint8_t AD_HEADER[2] = {0xF0, 0x08};

static uint32_t s_identifier;
static uint8_t s_tx_count;

// Brightness (Matter CurrentLevel, 0-254) and color temperature (Matter
// ColorTemperatureMireds) are independent attributes but map onto a single
// CMD_DIM command that sets both LED channels at once, so track the last
// value of each and recompute/resend on every change to either one.
static uint8_t s_current_level = DEFAULT_BRIGHTNESS;
static uint16_t s_current_mireds = 250;

static const uint16_t CRC_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

static const uint8_t XBOXES[128] = {
    0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC,
    0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A,
    0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85,
    0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
    0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
    0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
    0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C,
    0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
    0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9,
    0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94,
    0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68,
    0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16};

static uint16_t crc16_seeded(const uint8_t *data, int len, uint16_t init)
{
    uint32_t crc = init;
    for (int i = 0; i < len; i++) {
        crc = CRC_TABLE[((crc >> 8) ^ data[i]) & 255] ^ (crc << 8);
    }
    return (uint16_t)crc;
}

static void whiten_v2(uint8_t *buf, int size, uint8_t seed, uint8_t salt)
{
    for (int i = 0; i < size; i++) {
        buf[i] ^= XBOXES[((seed + i + 9) & 0x1f) + (salt & 0x3) * 0x20];
        buf[i] ^= seed;
    }
}

// AES-128-ECB single block encrypt via PSA Crypto (the mbedtls 4.x /
// tf-psa-crypto legacy mbedtls_aes_* API is not usable directly from
// application code any more -- its header is a "private" build-internal
// one). psa_crypto_init() is idempotent; Matter itself already calls it,
// but we call it defensively here too since this can run standalone.
static uint16_t aes_sign(const uint8_t *buf16, uint8_t tx_count, uint16_t seed)
{
    uint8_t sigkey[16] = {0, 0, 0, 0x0D, 0xBF, 0xE6, 0x42, 0x68,
                          0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16};
    sigkey[0] = seed & 0xFF;
    sigkey[1] = (seed >> 8) & 0xFF;
    sigkey[2] = tx_count;

    psa_crypto_init();

    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, 128);

    psa_key_id_t key_id;
    if (psa_import_key(&attr, sigkey, sizeof(sigkey), &key_id) != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_import_key failed for lamp sign key");
        return 0xFFFF;
    }

    uint8_t out[16];
    size_t out_len = 0;
    psa_status_t status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, buf16, 16, out, sizeof(out), &out_len);
    psa_destroy_key(key_id);
    if (status != PSA_SUCCESS || out_len < 2) {
        ESP_LOGE(TAG, "psa_cipher_encrypt failed: %d", (int)status);
        return 0xFFFF;
    }

    uint16_t sign_val = out[0] | (out[1] << 8);
    return sign_val == 0 ? 0xFFFF : sign_val;
}

static void build_packet(uint16_t command, const uint8_t args[4], uint8_t out_ad[31])
{
    uint16_t seed = (uint16_t)(esp_random() & 0xFFFF);

    uint8_t buf[24];
    memcpy(buf, PREFIX, 3);
    buf[3] = s_tx_count;
    buf[4] = DEVICE_TYPE & 0xFF;
    buf[5] = (DEVICE_TYPE >> 8) & 0xFF;
    memcpy(buf + 6, &s_identifier, 4);
    buf[10] = GROUP_INDEX;
    buf[11] = command & 0xFF;
    buf[12] = (command >> 8) & 0xFF;
    memcpy(buf + 13, args, 4);
    buf[20] = seed & 0xFF;
    buf[21] = (seed >> 8) & 0xFF;

    uint16_t sign_val = aes_sign(buf + 1, s_tx_count, seed);
    buf[17] = sign_val & 0xFF;
    buf[18] = (sign_val >> 8) & 0xFF;
    buf[19] = 0x00;

    whiten_v2(buf + 2, 18, (uint8_t)seed, 0);

    uint16_t crc = crc16_seeded(buf, 22, (uint16_t)(~seed));
    buf[22] = crc & 0xFF;
    buf[23] = (crc >> 8) & 0xFF;

    out_ad[0] = 0x02;
    out_ad[1] = 0x01;
    out_ad[2] = 0x02;
    out_ad[3] = 0x1B;
    out_ad[4] = 0x16;
    memcpy(out_ad + 5, AD_HEADER, 2);
    memcpy(out_ad + 7, buf, 24);
}

static int lamp_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    return 0;
}

static void send_packet_n(uint16_t command, const uint8_t args[4], int repeat_count)
{
    if (!ble_hs_synced()) {
        ESP_LOGW(TAG, "lamp BLE host not synced yet, dropping command 0x%04x", command);
        return;
    }

    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;
    adv_params.itvl_min = 0x0020;
    adv_params.itvl_max = 0x0020;

    for (int rep = 0; rep < repeat_count; rep++) {
        uint8_t ad[31];
        build_packet(command, args, ad);
        s_tx_count++;

        int rc = ble_gap_adv_set_data(ad, sizeof(ad));
        if (rc != 0) {
            ESP_LOGW(TAG, "ble_gap_adv_set_data failed: %d", rc);
            return;
        }
        rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, lamp_ble_gap_event, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "ble_gap_adv_start failed: %d", rc);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(80));
        ble_gap_adv_stop();
    }
}

static void send_packet(uint16_t command, const uint8_t args[4])
{
    send_packet_n(command, args, REPEAT_COUNT);
}

// Recomputes cold/warm channel levels from the last-known brightness and
// color temperature (whichever attribute just changed) and sends CMD_DIM.
static void lamp_send_dim(void)
{
    uint8_t level255 = REMAP_TO_RANGE(s_current_level, MATTER_BRIGHTNESS, 255);

    uint16_t mireds = s_current_mireds;
    if (mireds < LAMP_MIREDS_MIN) mireds = LAMP_MIREDS_MIN;
    if (mireds > LAMP_MIREDS_MAX) mireds = LAMP_MIREDS_MAX;

    // ratio 0 at LAMP_MIREDS_MAX (warmest) -> 1 at LAMP_MIREDS_MIN (coolest)
    uint32_t cold = (uint32_t)level255 * (LAMP_MIREDS_MAX - mireds) / (LAMP_MIREDS_MAX - LAMP_MIREDS_MIN);
    uint32_t warm = (uint32_t)level255 * (mireds - LAMP_MIREDS_MIN) / (LAMP_MIREDS_MAX - LAMP_MIREDS_MIN);

    uint8_t args[4] = {0, 0, (uint8_t)cold, (uint8_t)warm};
    send_packet(CMD_DIM, args);
}

// Reads this chip's own BT MAC into s_identifier, used by build_packet() as
// the lamp's per-remote identifier. Safe to call at any time (doesn't touch
// the BLE stack itself).
static void app_driver_lamp_identifier_init(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(&s_identifier, mac + 2, 4);
    ESP_LOGI(TAG, "lamp identifier: %08" PRIX32, s_identifier);
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    uint16_t endpoint_id = light_endpoint_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    attribute_t *attribute = attribute::get(endpoint_id, cluster_id, attribute_id);

    esp_matter_attr_val_t val;
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id != light_endpoint_id) {
        return ESP_OK;
    }
    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        const uint8_t zero_args[4] = {0, 0, 0, 0};
        send_packet(val->val.b ? CMD_TURN_ON : CMD_TURN_OFF, zero_args);
    } else if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
        s_current_level = val->val.u8;
        lamp_send_dim();
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
        s_current_mireds = val->val.u16;
        lamp_send_dim();
    }
    return ESP_OK;
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    esp_matter_attr_val_t val;

    attribute_t *attribute = attribute::get(endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_attribute_update(nullptr, endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id, &val);

    attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_attribute_update(nullptr, endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id, &val);

    attribute = attribute::get(endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_attribute_update(nullptr, endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val);

    return err;
}

app_driver_handle_t app_driver_light_init()
{
    /* No local LED to drive -- power state is relayed to the physical lamp
       over BLE instead of a GPIO. */
    app_driver_lamp_identifier_init();
    return (app_driver_handle_t)1;
}

app_driver_handle_t app_driver_button_init()
{
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = button_driver_get_config();

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }

    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
    return (app_driver_handle_t)handle;
}
