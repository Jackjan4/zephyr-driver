#include <device.h>
#include <zephyr.h>

#ifndef __SL025B_H__
#define __SL025B_H__

// Preambles
#define SL025B_PREAMBLE_HOST 0xBA
#define SL025B_PREAMBLE_SL025B 0xBD

// Status Codes
#define SL025B_STATUS_OP_SUCCEED    0x00
#define SL025B_STATUS_NO_TAG        0x01
#define SL025B_STATUS_LOGIN_SUCCEED 0x02
// ...
#define SL025B_STATUS_CHECKSUM_ERROR 0xF0

// RFID Card Types
#define SL025B_CARD_TYPE_MIFARE_1K_UID4     0x01
#define SL025B_CARD_TYPE_MIFARE_1K_UID7     0x02
#define SL025B_CARD_TYPE_MIFARE_ULTRALIGHT  0x03
#define SL025B_CARD_TYPE_MIFARE_4K_UID4     0x04
#define SL025B_CARD_TYPE_MIFARE_4K_UID7     0x05
#define SL025B_CARD_TYPE_MIFARE_DESFIRE     0x06
#define SL025B_CARD_TYPE_OTHER              0x0A

// 
#define SL025B_KEY_MIFARE_DEFAULT 0xFFFFFFFFFFFF

// Commands
#define SL025B_COMMAND_SELECT_MIFARE_CARD   0x01
#define SL025B_COMMAND_LOGIN_TO_SECTOR      0x02
#define SL025B_COMMAND_READ_DATA_BLOCK      0x03
#define SL025B_COMMAND_WRITE_DATA_BLOCK     0x04
#define SL025B_COMMAND_READ_VALUE_BLOCK     0x05
#define SL025B_COMMAND_INIT_VALUE_BLOCK     0x06
#define SL025B_COMMAND_WRITE_MASTER_KEY_A   0x07
// ... There are more commands here but we don't need them
#define SL025B_COMMAND_MANAGE_RED_LED       0x40
#define SL025B_COMMAND_GET_FW_VERSION       0xF0



typedef int (*sl025b_select_mifare_card_t)(const struct device *dev, struct sl025b_response *response, uint8_t* data_buffer, uint8_t buffer_size, k_timeout_t timeout);

typedef int (*sl025b_login_to_sector_t)(const struct device *dev, uint8_t sector, uint8_t key_type, uint8_t key[6], struct sl025b_response *response, k_timeout_t timeout);

typedef int (*sl025b_read_data_block_t)(const struct device *dev, uint8_t block, struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout);

typedef int (*sl025b_write_data_block_t)(const struct device *dev, uint8_t block, uint8_t write_data[16], struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout);

// ...

typedef int (*sl025b_write_master_key_a_t)(const struct device *dev, uint8_t sector, uint8_t key[6], struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout);

// ...

typedef int (*sl025b_manage_red_led_t)(const struct device *dev, uint8_t value, struct sl025b_response *response, k_timeout_t timeout);

typedef int (*sl025b_get_fw_version_t)(const struct device *dev, struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout);



struct sl025b_api {
    sl025b_select_mifare_card_t select_mifare_card;
    sl025b_login_to_sector_t login_to_sector;
    sl025b_read_data_block_t read_data_block;
    sl025b_write_data_block_t write_data_block;
    sl025b_write_master_key_a_t write_master_key_a;
    sl025b_manage_red_led_t manage_red_led;
    sl025b_get_fw_version_t get_fw_version;
};

struct sl025b_response {
    uint8_t preamble;
    uint8_t len;
    uint8_t command;
    uint8_t status;
    uint8_t checksum;
};


static inline int sl025b_select_mifare_card(const struct device *dev, struct sl025b_response *response, uint8_t* data_buffer, uint8_t buffer_size, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->select_mifare_card(dev, response, data_buffer, buffer_size, timeout);
}

static inline int sl025b_login_to_sector(const struct device *dev, uint8_t sector, uint8_t key_type, uint8_t key[6], struct sl025b_response *response, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->login_to_sector(dev, sector, key_type, key, response, timeout);
}

static inline int sl025b_read_data_block(const struct device *dev, uint8_t block, struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->read_data_block(dev, block, response, data_buffer, buffer_size, timeout);
}

static inline int sl025b_write_data_block(const struct device *dev, uint8_t block, uint8_t write_data[16], struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->write_data_block(dev, block, write_data, response, data_buffer, buffer_size, timeout);
}

// ...

static inline int sl025b_write_master_key_a(const struct device *dev, uint8_t sector, uint8_t key[6], struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->write_master_key_a(dev, sector, key, response, data_buffer, buffer_size, timeout);
}

// ...

static inline int sl025b_manage_red_led(const struct device *dev, uint8_t value, struct sl025b_response *response, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->manage_red_led(dev, value, response, timeout);
}

static inline int sl025b_get_fw_version(const struct device *dev, struct sl025b_response *response, uint8_t *data_buffer, uint8_t buffer_size, k_timeout_t timeout) {
    struct sl025b_api *api = (struct sl025b_api *)dev->api;
    return api->get_fw_version(dev, response, data_buffer, buffer_size, timeout);
}
#endif