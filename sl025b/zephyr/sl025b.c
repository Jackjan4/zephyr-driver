#define DT_DRV_COMPAT stronglink_sl025b
#define MODULE sl025b

// Header
#include "sl025b.h"

#define SL025B_INIT_PRIORITY 41

// Zephyr Header
#ifdef CONFIG_LOG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE);  // TODO: Hier muss wieder CONFIG_UART_LOG_LEVEL rein oder noch besser SL025B_LOG_LEVEL
#endif
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

struct sl025b_config {
    const struct device *uart_dev;  // UART device.
};

// Enums
enum uart_cb_state_t {
    CB_STATE_START = 0x00,
    CB_STATE_RECEIVE_LEN = 0x01,
    CB_STATE_DATA = 0x02,
};

#define MSG_SIZE 25

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static uint8_t uart_rx_buf[MSG_SIZE] = {0};  // UART rx buffer
static uint8_t rx_buffer_pos;
static uint8_t msg_pending;
static enum uart_cb_state_t cb_state;

static int construct_sl025b_response(struct sl025b_response *response, uint8_t *msg_buffer) {
    int err = 0;

    if (msg_buffer[0] != SL025B_PREAMBLE_SL025B) {
        err = -1;
        return err;
    }

    uint8_t len = msg_buffer[1];

    // TODO: Check if len is valid

    response->preamble = msg_buffer[0],
    response->len = len,
    response->command = msg_buffer[2],
    response->status = msg_buffer[3];
    response->checksum = msg_buffer[1 + len];

    return err;
};

void serial_cb(const struct device *dev, void *user_data) {
    uint8_t c;

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uart_fifo_read(dev, &c, 1);

        switch (cb_state) {
            case (CB_STATE_START): {
                if (c == SL025B_PREAMBLE_SL025B) {
                    rx_buffer_pos = 0;
                    uart_rx_buf[rx_buffer_pos] = c;
                    rx_buffer_pos++;
                    cb_state = CB_STATE_RECEIVE_LEN;
                }
                break;
            }
            case (CB_STATE_RECEIVE_LEN): {
                msg_pending = c;
                uart_rx_buf[rx_buffer_pos] = c;
                rx_buffer_pos++;
                cb_state = CB_STATE_DATA;
                break;
            }
            case (CB_STATE_DATA): {
                if (msg_pending > 0) {
                    uart_rx_buf[rx_buffer_pos] = c;
                    rx_buffer_pos++;
                    msg_pending--;

                    if (msg_pending <= 0) {
                        k_msgq_put(&uart_msgq, &uart_rx_buf, K_NO_WAIT);
                        cb_state = CB_STATE_START; // Reset callback state machine
                    }
                }
            }
        }
    }
}

static void write_command(const struct device *dev, uint8_t command, uint8_t *data, uint8_t size) {
    struct sl025b_config *config = (struct sl025b_config *)dev->config;
    // Byte 1   :   Preamble    [0xBA]
    // Byte 2   :   Len         []
    // Byte 3   :   Command     [command]
    // Byte 4..X:   Data        [data0]
    // Byte X+1 :   Checksum    [1 XOR ... XOR X]

    // Construct UART packet...
    uint8_t packet_length = size + 4;
    uint8_t packet_buffer[packet_length];

    packet_buffer[0] = SL025B_PREAMBLE_HOST;  // Preamble
    packet_buffer[1] = packet_length - 2;     // Len
    packet_buffer[2] = command;               // Command

    // Data
    if (size > 0) {
        for (int i = 0; i < size; i++) {
            packet_buffer[3 + i] = data[i];
        }
    }

    // Checksum
    uint8_t checksum = packet_buffer[0];
    for (int j = 1; j < packet_length - 1; j++) {
        checksum ^= packet_buffer[j];
    }
    packet_buffer[packet_length - 1] = checksum;

    // Send packet over UART
    for (int k = 0; k < packet_length; k++) {
        uart_poll_out(config->uart_dev, packet_buffer[k]);
    }
}

static int receive_response(struct sl025b_response *response, uint8_t *data, k_timeout_t timeout) {
    int err = 0;

    uint8_t msg_buffer[MSG_SIZE];
    err = k_msgq_get(&uart_msgq, &msg_buffer, timeout);

    // If message queue has no message, abort
    if (err) {
        // k_msgq_get can throw: -EAGAIN : Timeout
        return err;
    }

    //
    err = construct_sl025b_response(response, &msg_buffer);

    if (err) {
        return err;
    }

    // Copy data
    if (response->len > 3) {
        for (int i = 0; i < response->len - 1; i++) {
            data[i] = msg_buffer[4 + i];
        }
    }
    return err;
}

static void select_mifare_card(const struct device *dev, struct sl025b_response *response, uint8_t *data, k_timeout_t timeout) {
    int err = 0;
    struct sl025b_config *config = (struct sl025b_config *)dev->config;

    write_command(dev, SL025B_COMMAND_SELECT_MIFARE_CARD, NULL, 0);
    err = receive_response(response, data, timeout);
    return err;
}

static int manage_red_led(const struct device *dev, uint8_t value, struct sl025b_response *response, uint8_t *data, k_timeout_t timeout) {
    int err = 0;
    struct sl025b_config *config = (struct sl025b_config *)dev->config;

    uint8_t input_data[1];
    input_data[0] = value;

    write_command(dev, SL025B_COMMAND_MANAGE_RED_LED, &input_data, 1);
    err = receive_response(response, data, timeout);
    return err;
}

static int get_fw_version(const struct device *dev, struct sl025b_response *response, uint8_t *data, k_timeout_t timeout) {
    int err = 0;
    struct sl025b_config *config = (struct sl025b_config *)dev->config;

    write_command(dev, SL025B_COMMAND_GET_FW_VERSION, NULL, 0);
    err = receive_response(response, data, timeout);
    return err;
}

/**
 * @brief Initializes Stronglink SL025B
 *
 * @param dev UART peripheral device.
 * @return 0 on success, negative error code otherwise.
 */
static int init_sl025b(const struct device *dev) {
    int err = 0;
    struct sl025b_config *config = (struct sl025b_config *)dev->config;

    if (!device_is_ready(config->uart_dev)) {
        __ASSERT(false, "UART device not ready");
        return -ENODEV;
    }

    cb_state = CB_STATE_START;
    /* configure interrupt and callback to receive data */
    uart_irq_callback_user_data_set(config->uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(config->uart_dev);

    LOG_DBG("SL025B initialized");
    return 0;
}

static struct sl025b_api sl025b_api = {
    .select_mifare_card = select_mifare_card,
    .manage_red_led = manage_red_led,
    .get_fw_version = get_fw_version};

#define INIT_SL025B(inst)                                  \
    /*    static struct sl025b_data sl025b_data_##inst = { \
        };  */                                             \
    static struct sl025b_config sl025b_config_##inst = {   \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),      \
    };                                                     \
    DEVICE_DT_INST_DEFINE(inst,                            \
                          init_sl025b,                     \
                          NULL,                            \
                          NULL,                            \
                          &sl025b_config_##inst,           \
                          POST_KERNEL,                     \
                          SL025B_INIT_PRIORITY,            \
                          &sl025b_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_SL025B);