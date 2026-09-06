// LampSmart Pro / FanLamp Pro BLE raw-advertising protocol -- "other/v3"
// variant, ported from aronsky/esphome-components (components/ble_adv_controller,
// FanLampEncoderV2) to ESP-IDF Bluedroid raw advertising on ESP32-C3.
//
// This variant differs from the simpler MasterDevX/lampify protocol we
// tried first: no bit-reversal, whitening uses a 128-byte XBOXES lookup
// table (not an LFSR), and each packet carries a 16-bit AES-128-ECB
// "sign" computed over a hardcoded key mixed with a per-packet seed and
// tx_count. Confirmed to match this exact lamp's remote via BLE capture
// (fixed prefix 10 80 00, header F0 08, AD type 0x16 service data).

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "mbedtls/aes.h"

static const char *TAG = "lampsmart";
static SemaphoreHandle_t s_ble_sem;
static uint32_t s_identifier;
static uint8_t s_tx_count;

#define CMD_PAIR 0x28
#define CMD_UNPAIR 0x45
#define CMD_TURN_ON 0x10
#define CMD_TURN_OFF 0x11
#define CMD_DIM 0x21 // LIGHT_WCOLOR
// Night light (moon icon on the remote): captured directly from the real
// remote via BLE scan + our own decoder (whitening is self-inverse once
// the per-packet seed is known). It's a single toggle command, not a
// separate on/off pair -- 0x12/0x13 (LIGHT_SEC_ON/OFF in the reference
// CommandType enum) turned out not to be what this lamp actually uses.
#define CMD_NIGHT_TOGGLE 0x23

#define DEVICE_TYPE 0x0100
#define GROUP_INDEX 0x00

static const uint8_t BRIGHTNESS_LEVELS[10] = {
    0x1A, 0x33, 0x4C, 0x66, 0x7F,
    0x99, 0xB2, 0xCC, 0xE5, 0xFF};

// FanLampEncoderV2 "other/v3": prefix + AD service-data header, verbatim
// from the __init__.py config table matching our captures.
static const uint8_t PREFIX[3] = {0x10, 0x80, 0x00};
static const uint8_t AD_HEADER[2] = {0xF0, 0x08};

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

// CRC16/CCITT (poly 0x1021, MSB-first, no reflection) with a caller-chosen
// initial value -- matches esphome's crc16be() used by FanLampEncoderV2.
static uint16_t crc16_seeded(const uint8_t *data, int len, uint16_t init)
{
    uint32_t crc = init;
    for (int i = 0; i < len; i++)
    {
        crc = CRC_TABLE[((crc >> 8) ^ data[i]) & 255] ^ (crc << 8);
    }
    return (uint16_t)crc;
}

// FanLampEncoderV2::whiten(): XOR each byte with a table entry selected by
// (seed + position) and with the seed itself. Not an LFSR like lampify's
// protocol -- a fixed 128-byte substitution table instead.
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

static void whiten_v2(uint8_t *buf, int size, uint8_t seed, uint8_t salt)
{
    for (int i = 0; i < size; i++)
    {
        buf[i] ^= XBOXES[((seed + i + 9) & 0x1f) + (salt & 0x3) * 0x20];
        buf[i] ^= seed;
    }
}

// FanLampEncoderV2::sign(): AES-128-ECB over 16 bytes (buf[1..16] of the
// packet, i.e. 2 prefix bytes + tx_count..args, all still unwhitened at
// this point), keyed by a hardcoded constant mixed with seed+tx_count.
// The low 16 bits of the first AES output word become the "sign" field.
static uint16_t aes_sign(const uint8_t *buf16, uint8_t tx_count, uint16_t seed)
{
    uint8_t sigkey[16] = {0, 0, 0, 0x0D, 0xBF, 0xE6, 0x42, 0x68,
                           0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16};
    sigkey[0] = seed & 0xFF;
    sigkey[1] = (seed >> 8) & 0xFF;
    sigkey[2] = tx_count;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, sigkey, 128);
    uint8_t out[16];
    mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, buf16, out);
    mbedtls_aes_free(&ctx);

    uint16_t sign_val = out[0] | (out[1] << 8);
    return sign_val == 0 ? 0xFFFF : sign_val;
}

static void init_identifier(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(&s_identifier, mac + 2, 4);
    ESP_LOGI(TAG, "identifier: %08" PRIX32, s_identifier);
}

// Builds the 31-byte raw advertising payload (flags + service-data
// structure) for one command. `args` is the 4-byte command argument
// block used by FanLampEncoderV2 (brightness/temperature/etc, zero for
// plain on/off/pair/unpair).
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
    buf[19] = 0x00; // spare

    whiten_v2(buf + 2, 18, (uint8_t)seed, 0);

    uint16_t crc = crc16_seeded(buf, 22, (uint16_t)(~seed));
    buf[22] = crc & 0xFF;
    buf[23] = (crc >> 8) & 0xFF;

    out_ad[0] = 0x02;
    out_ad[1] = 0x01;
    out_ad[2] = 0x02; // flags
    out_ad[3] = 0x1B; // length of the following structure (27)
    out_ad[4] = 0x16; // AD type: Service Data - 16 bit UUID
    memcpy(out_ad + 5, AD_HEADER, 2);
    memcpy(out_ad + 7, buf, 24);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        xSemaphoreGive(s_ble_sem);
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            const uint8_t *adv = param->scan_rst.ble_adv;
            uint8_t len = param->scan_rst.adv_data_len;
            // Cheap signature filter: only print packets that contain the
            // Service Data (0x16) + UUID 08F0 bytes seen from the real
            // remote, to cut phone/laptop BLE noise out of the log.
            bool interesting = false;
            for (int i = 0; i + 2 < len; i++)
            {
                if (adv[i] == 0x16 && adv[i + 1] == 0xF0 && adv[i + 2] == 0x08)
                {
                    interesting = true;
                    break;
                }
            }
            if (interesting)
            {
                printf("SCAN %02X:%02X:%02X:%02X:%02X:%02X rssi=%d connectable=%d adv(%d):",
                       param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                       param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5],
                       param->scan_rst.rssi, param->scan_rst.ble_evt_type);
                for (int i = 0; i < len; i++)
                {
                    printf(" %02X", adv[i]);
                }
                printf("\n");
            }
        }
        break;
    default:
        break;
    }
}

// Same params as lampify's hciSetParams(): 20ms interval, all 3 primary
// adv channels. Non-connectable, matching the real remote's Connectable:No.
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x0020,
    .adv_int_max = 0x0020,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static bool wait_ble_event(const char *step)
{
    if (xSemaphoreTake(s_ble_sem, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        printf("timed out waiting for BLE event after %s\n", step);
        return false;
    }
    return true;
}

static void send_raw_adv(const uint8_t *raw_ad_data, uint8_t len)
{
    printf("adv payload (%d bytes):", len);
    for (int i = 0; i < len; i++)
    {
        printf(" %02X", raw_ad_data[i]);
    }
    printf("\n");

    esp_err_t err = esp_ble_gap_config_adv_data_raw((uint8_t *)raw_ad_data, len);
    if (err != ESP_OK)
    {
        printf("esp_ble_gap_config_adv_data_raw failed: %s\n", esp_err_to_name(err));
        return;
    }
    if (!wait_ble_event("config_adv_data_raw")) return;

    err = esp_ble_gap_start_advertising(&s_adv_params);
    if (err != ESP_OK)
    {
        printf("esp_ble_gap_start_advertising failed: %s\n", esp_err_to_name(err));
        return;
    }
    if (!wait_ble_event("start_advertising")) return;

    vTaskDelay(pdMS_TO_TICKS(100));

    err = esp_ble_gap_stop_advertising();
    if (err != ESP_OK)
    {
        printf("esp_ble_gap_stop_advertising failed: %s\n", esp_err_to_name(err));
        return;
    }
    wait_ble_event("stop_advertising");
}

// A single press on the real remote keeps transmitting for a while
// (observed via BLE scan: continuous stream of fresh packets, not one
// shot). Mimic that by repeating the command, each time with a freshly
// built packet (new seed/sign/CRC, incrementing tx_count).
//
// This only makes sense for idempotent commands (on/off/dim: repeating
// them doesn't change the outcome). CMD_NIGHT_TOGGLE is NOT idempotent --
// each accepted copy flips the state again, so it must be sent with a
// low repeat count or it visibly flickers through many toggles.
#define REPEAT_COUNT 15
#define REPEAT_COUNT_TOGGLE 1

static void send_packet_n(uint16_t command, const uint8_t args[4], int repeat_count)
{
    for (int rep = 0; rep < repeat_count; rep++)
    {
        uint8_t ad[31];
        build_packet(command, args, ad);
        s_tx_count++;

        if (rep == 0)
        {
            printf("adv payload (31 bytes):");
            for (int i = 0; i < 31; i++)
            {
                printf(" %02X", ad[i]);
            }
            printf(" (x%d)\n", repeat_count);
        }

        esp_err_t err = esp_ble_gap_config_adv_data_raw(ad, sizeof(ad));
        if (err != ESP_OK)
        {
            printf("esp_ble_gap_config_adv_data_raw failed: %s\n", esp_err_to_name(err));
            return;
        }
        if (!wait_ble_event("config_adv_data_raw")) return;

        err = esp_ble_gap_start_advertising(&s_adv_params);
        if (err != ESP_OK)
        {
            printf("esp_ble_gap_start_advertising failed: %s\n", esp_err_to_name(err));
            return;
        }
        if (!wait_ble_event("start_advertising")) return;

        vTaskDelay(pdMS_TO_TICKS(80));

        err = esp_ble_gap_stop_advertising();
        if (err != ESP_OK)
        {
            printf("esp_ble_gap_stop_advertising failed: %s\n", esp_err_to_name(err));
            return;
        }
        if (!wait_ble_event("stop_advertising")) return;
    }
}

static void send_packet(uint16_t command, const uint8_t args[4])
{
    send_packet_n(command, args, REPEAT_COUNT);
}

// A packet actually captured (via CoreBluetooth scan log) from the real
// physical remote controlling this exact lamp, byte-for-byte. Useful as a
// known-good reference to replay for comparison.
static const uint8_t CAPTURED_REAL_PACKET[] = {
    0x02, 0x01, 0x01,
    0x1B, 0x16, 0xF0, 0x08,
    0x10, 0x00, 0xD5, 0x8A, 0x70, 0x89, 0x5D, 0x0D, 0x9F, 0xC9, 0x64, 0xAE,
    0xA9, 0xB8, 0x7B, 0x9F, 0x7F, 0xB4, 0xB1, 0xB9, 0xBC, 0xB8, 0xA9, 0xD0};

static const esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

static void do_scan(int seconds)
{
    esp_err_t err = esp_ble_gap_set_scan_params((esp_ble_scan_params_t *)&s_scan_params);
    if (err != ESP_OK)
    {
        printf("esp_ble_gap_set_scan_params failed: %s\n", esp_err_to_name(err));
        return;
    }
    if (!wait_ble_event("set_scan_params")) return;

    printf("scanning for %d seconds, looking for 16 F0 08 signature...\n", seconds);
    err = esp_ble_gap_start_scanning(seconds);
    if (err != ESP_OK)
    {
        printf("esp_ble_gap_start_scanning failed: %s\n", esp_err_to_name(err));
        return;
    }
    if (!wait_ble_event("start_scanning")) return;

    vTaskDelay(pdMS_TO_TICKS((seconds + 1) * 1000));
    printf("scan done\n");
}

static void init_console_uart(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    if (!usb_serial_jtag_is_driver_installed())
    {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    }
    usb_serial_jtag_vfs_use_driver();
}

static void init_ble(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    s_ble_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    // Scan response is a separate packet slot from the raw adv data used
    // for the lamp protocol, so we can freely stick a local name here to
    // make the device easy to spot in a BLE scanner, without touching the
    // protocol-critical advertising payload at all.
    static const uint8_t scan_rsp[] = {
        13, 0x09, 'L', 'A', 'M', 'P', 'S', 'M', 'A', 'R', 'T', '-', 'C', '3'};
    ESP_ERROR_CHECK(esp_ble_gap_config_scan_rsp_data_raw((uint8_t *)scan_rsp, sizeof(scan_rsp)));
    xSemaphoreTake(s_ble_sem, portMAX_DELAY);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_ble();
    init_identifier();
    init_console_uart();

    printf("\nlampsmart-pro raw BLE console (FanLampEncoderV2 other/v3)\n");
    printf("commands: pair / unpair / on / off / cold <0-9> / warm <0-9> / dual <0-9> / full / half / night / scan [sec] / replay\n");
    printf("pair the bulb within 5s of powering it on.\n\n");

    char line[64];
    while (1)
    {
        printf("lamp> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        line[strcspn(line, "\r\n")] = 0;
        if (strlen(line) == 0)
        {
            continue;
        }

        char cmd[16] = {0};
        int arg = -1;
        sscanf(line, "%15s %d", cmd, &arg);
        const uint8_t zero_args[4] = {0, 0, 0, 0};

        if (!strcmp(cmd, "pair"))
        {
            send_packet(CMD_PAIR, zero_args);
            printf("sent: pair\n");
        }
        else if (!strcmp(cmd, "unpair"))
        {
            send_packet(CMD_UNPAIR, zero_args);
            printf("sent: unpair\n");
        }
        else if (!strcmp(cmd, "on"))
        {
            send_packet(CMD_TURN_ON, zero_args);
            printf("sent: on\n");
        }
        else if (!strcmp(cmd, "off"))
        {
            send_packet(CMD_TURN_OFF, zero_args);
            printf("sent: off\n");
        }
        else if (!strcmp(cmd, "night"))
        {
            send_packet_n(CMD_NIGHT_TOGGLE, zero_args, REPEAT_COUNT_TOGGLE);
            printf("sent: night (toggle)\n");
        }
        else if (!strcmp(cmd, "full"))
        {
            uint8_t level = BRIGHTNESS_LEVELS[9];
            uint8_t args[4] = {0, 0, level, level};
            send_packet(CMD_DIM, args);
            printf("sent: full\n");
        }
        else if (!strcmp(cmd, "half"))
        {
            uint8_t level = BRIGHTNESS_LEVELS[4];
            uint8_t args[4] = {0, 0, level, level};
            send_packet(CMD_DIM, args);
            printf("sent: half\n");
        }
        else if (!strcmp(cmd, "replay"))
        {
            send_raw_adv(CAPTURED_REAL_PACKET, sizeof(CAPTURED_REAL_PACKET));
            printf("sent: replay\n");
        }
        else if (!strcmp(cmd, "scan"))
        {
            do_scan(arg >= 0 ? arg : 20);
        }
        else if (!strcmp(cmd, "cold") && arg >= 0)
        {
            uint8_t level = BRIGHTNESS_LEVELS[arg % 10];
            uint8_t args[4] = {0, 0, level, 0};
            send_packet(CMD_DIM, args);
            printf("sent: cold %d\n", arg % 10);
        }
        else if (!strcmp(cmd, "warm") && arg >= 0)
        {
            uint8_t level = BRIGHTNESS_LEVELS[arg % 10];
            uint8_t args[4] = {0, 0, 0, level};
            send_packet(CMD_DIM, args);
            printf("sent: warm %d\n", arg % 10);
        }
        else if (!strcmp(cmd, "dual") && arg >= 0)
        {
            uint8_t level = BRIGHTNESS_LEVELS[arg % 10];
            uint8_t args[4] = {0, 0, level, level};
            send_packet(CMD_DIM, args);
            printf("sent: dual %d\n", arg % 10);
        }
        else
        {
            printf("unknown command: %s\n", line);
        }
    }
}
