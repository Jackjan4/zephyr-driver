
// Header
#include "shockburst.h"

// Zephyr Header
#ifdef CONFIG_LOG
#include <logging/log.h>
#endif
#include <errno.h>
#include <irq.h>

#ifdef CONFIG_LOG
LOG_MODULE_REGISTER(shockburst);
#endif

/* These are set to zero as ShockBurst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  (0UL)  /**< Packet length field size in bits. */

// Event handler function pointer
static void (*event_handler)(enum shockburst_event);

// TODO: No fixed max payload size
static uint8_t tx_payload_buffer[CONFIG_SHOCKBURST_MAX_PAYLOAD_LENGTH];
static uint8_t rx_payload_buffer[CONFIG_SHOCKBURST_MAX_PAYLOAD_LENGTH];

static uint8_t payload_length;



static void radio_irq_handler(void) {
    if (NRF_RADIO->EVENTS_END && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk)) {
		NRF_RADIO->EVENTS_END = 0; // Clear EVENTS_END
		
        if (event_handler) {
            event_handler(SHOCKBURST_EVENT_RX_RECEIVED);
        }
	}
}

ISR_DIRECT_DECLARE(RADIO_SHOCKBURST_IRQHandler) {
    radio_irq_handler();

    ISR_DIRECT_PM();

    return 1;
}



/**
 * @brief Function for swapping/mirroring bits in a byte.
 *
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 *
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31]
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp) {
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}

void shockburst_init() {
    /// Radio config
    /// Default settings
    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Neg4dBm << RADIO_TXPOWER_TXPOWER_Pos); // 0dBm
    NRF_RADIO->FREQUENCY = 7UL; // Frequency bin 7, 2407MHz
    NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos); // 1Mmbps

    // Radio address config
    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xE7) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xE7) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0xE7E7E7E7UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = (0x01UL << 1);  // Enable device address 1 to use to select which addresses to receive

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                       (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                       (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
                       (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |
                       (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    payload_length = PACKET_STATIC_LENGTH;

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Activate CRC and use two checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos)) {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos)) {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }

    // TODO: Priority
    IRQ_DIRECT_CONNECT(RADIO_IRQn, 1, RADIO_SHOCKBURST_IRQHandler, 0);
    irq_enable(RADIO_IRQn);
}

int shockburst_set_frequency(uint32_t freq) {
    if (freq > 100) {
		return -EINVAL;
	}

    NRF_RADIO->FREQUENCY = freq;

    return 0;
}

void shockburst_set_bitrate(enum shockburst_bitrate bitrate) {
    NRF_RADIO->MODE = (bitrate << RADIO_MODE_MODE_Pos);
}

void shockburst_set_tx_power(enum shockburst_tx_power power) {
    NRF_RADIO->TXPOWER = (power << RADIO_TXPOWER_TXPOWER_Pos);
}

void shockburst_set_base_adress0(uint32_t base0) {
    NRF_RADIO->BASE0 = bytewise_bitswap(base0);
}

void shockburst_set_base_adress1(uint32_t base1) {
    NRF_RADIO->BASE1 = bytewise_bitswap(base1);
}

void shockburst_set_prefixes(uint8_t prefix0, uint8_t prefix1, uint8_t prefix2, uint8_t prefix3, uint8_t prefix4, uint8_t prefix5, uint8_t prefix6, uint8_t prefix7) {
    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(prefix3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix0) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(prefix7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix5) << 8) // Prefix byte of address 4 converted to nRF24L series format
      | ((uint32_t)swap_bits(prefix4) << 0); // Prefix byte of address 4 converted to nRF24L series format
}

void shockburst_set_crc(enum shockburst_crc crc) {
    NRF_RADIO->CRCCNF = (crc << RADIO_CRCCNF_LEN_Pos);

    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
}

enum shockburst_radio_state shockburst_get_radio_state() {
    enum shockburst_radio_state result = SHOCKBURST_RADIO_STATE_DISABLED;

    return result;
}

int shockburst_set_tx_address(uint32_t addr) {
    if (addr > 7) {
        return -EINVAL;
    }

    NRF_RADIO->TXADDRESS = addr;

    return 0;
}

void shockburst_set_rx_addresses(uint8_t activations) {
    NRF_RADIO->RXADDRESSES = activations;
}

int shockburst_clear_interrupt_end() {
    int err = 0;
    
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;

    return err;
}

int shockburst_write_tx_payload(uint8_t* payload, uint8_t size) {
    memcpy(&tx_payload_buffer[0], payload, size);
    NRF_RADIO->PACKETPTR = (uint32_t)tx_payload_buffer;

    NRF_RADIO->EVENTS_READY = 0U; // Clear EVENTS_READY register

    // If radio is disabled then start it up
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle) {
        NRF_RADIO->TASKS_TXEN = 1; // Enable Radio in TX mode

        while (NRF_RADIO->EVENTS_READY == 0U) {
            // wait for EVENTS_READY a.k.a. Radio has ramped up in TX mode and is ready to be started
        }
    }

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_TxIdle) {
        return -EPERWS;
    }

    NRF_RADIO->EVENTS_END  = 0U; // Clear EVENTS_END
    NRF_RADIO->TASKS_START = 1U; // Start Radio a.k.a. send packet

    while (NRF_RADIO->EVENTS_END == 0U) {
        // wait for EVENTS_END a.k.a. packet to be finished send
    }

    NRF_RADIO->EVENTS_DISABLED = 0U; // Clear EVENTS_DISABLED register
    NRF_RADIO->TASKS_DISABLE = 1U; // Disable Radio

    while (NRF_RADIO->EVENTS_DISABLED == 0U){
        // wait for Radio to be disabled
    }

    return 0;
}



int shockburst_write_tx_payload_pause(uint8_t* payload, uint8_t size) {
    int err = 0;

    memcpy(&tx_payload_buffer[0], payload, size);
    NRF_RADIO->PACKETPTR = (uint32_t)tx_payload_buffer;

    NRF_RADIO->EVENTS_READY = 0U; // Clear EVENTS_READY register

    // If radio is disabled then start it up
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle) {
        NRF_RADIO->TASKS_TXEN = 1; // Enable Radio in TX mode

        while (NRF_RADIO->EVENTS_READY == 0U) {
            // wait for EVENTS_READY a.k.a. Radio has ramped up in TX mode and is ready to be started
        }
    }

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_TxIdle) {
         return -EPERWS;
    }

    NRF_RADIO->EVENTS_END  = 0U; // Clear EVENTS_END
    NRF_RADIO->TASKS_START = 1U; // Start Radio a.k.a. send packet

    while (NRF_RADIO->EVENTS_END == 0U) {
        // wait for EVENTS_END a.k.a. packet to be finished se
    }

    return err;
}

int shockburst_read_rx_payload(uint8_t* payload) {
    int result = 0;

    NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;

    NRF_RADIO->EVENTS_READY = 0U; // Clear EVENTS_READY register

    // If radio is disabled then start it up
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) {
        NRF_RADIO->TASKS_RXEN = 1U; // Enable Radio in RX mode

        while (NRF_RADIO->EVENTS_READY == 0U) {
            // wait for EVENTS_READY a.k.a. Radio has ramped up in RX mode and is ready to be started
        }
    }

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        return -EPERWS;
    }

    NRF_RADIO->EVENTS_END = 0U; // Clear EVENTS_END
    NRF_RADIO->TASKS_START = 1U; // Start Radio a.k.a. listening for receiving packets

    while (NRF_RADIO->EVENTS_END == 0U) {
        // wait for EVENTS_END a.k.a. packet received
    }

    // Check if CRC of received packet is okay
    if (NRF_RADIO->CRCSTATUS == 0U) {
        // Error
        result = -ECHK;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U; // Clear EVENTS_DISABLED register
    NRF_RADIO->TASKS_DISABLE = 1U; // Disable Radio

    while (NRF_RADIO->EVENTS_DISABLED == 0U) {
        // wait for Radio to be disabled
    }

    // Copy rx_payload_buffer to payload argument
    memcpy(payload, rx_payload_buffer, payload_length);

    return result;
}

int shockburst_set_payload_length(uint32_t length) {
    uint32_t reg = NRF_RADIO->PCNF1;
    reg &= 0xFFFF0000UL;
    reg |= (length << RADIO_PCNF1_STATLEN_Pos) |
           (length << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PCNF1 = reg;

    payload_length = length;

    return 0;
}

int shockburst_set_address_base_length(uint32_t addr_length) {
    uint32_t reg = NRF_RADIO->PCNF1;
    reg &= 0xFFF8FFFFUL;
    reg |= (addr_length << RADIO_PCNF1_BALEN_Pos);
    NRF_RADIO->PCNF1 = reg;

    return 0;
}

int shockburst_activate_fast_rampup() {
    uint32_t reg = NRF_RADIO->MODECNF0;
    reg |= 1UL;
    NRF_RADIO->MODECNF0 = reg;

    return 0;
}

int shockburst_deactivate_fast_rampup() {
    uint32_t reg = NRF_RADIO->MODECNF0;
    reg &= 0xFFFFFFFEUL;
    NRF_RADIO->MODECNF0 = reg;

    return 0;
}

int shockburst_rx_start_listening() {
    int result = 0;

    NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;

    // Disable Interrupt for EVENTS_END (Packet send or received)
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;
    
    // If radio is disabled then start it up
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) {
        NRF_RADIO->EVENTS_READY = 0U; // Clear EVENTS_READY register
        NRF_RADIO->TASKS_RXEN = 1U; // Enable Radio in RX mode

        while (NRF_RADIO->EVENTS_READY == 0U) {
            // wait for EVENTS_READY a.k.a. Radio has ramped up in RX mode and is ready to be started
        }
    }

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        result = -EPERWS;
        return result;
    }

    NRF_RADIO->EVENTS_END = 0U; // Clear EVENTS_END
    NRF_RADIO->TASKS_START = 1U; // Start Radio a.k.a. listening for receiving packets

    return result;
}


int shockburst_rx_stop_listening() {
    int result = 0;
    //
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx) {
        NRF_RADIO->TASKS_STOP = 1U;

        while(NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
            // Wait until ready is in RXIDLE
        }
    }

    NRF_RADIO->EVENTS_DISABLED = 0U; // Clear EVENTS_DISABLED register
    NRF_RADIO->TASKS_DISABLE = 1U; // Disable Radio

    while (NRF_RADIO->EVENTS_DISABLED == 0U) {
        // wait for Radio to be disabled
    }

    return result;
}

int shockburst_rx_available() {
    uint8_t result = NRF_RADIO->EVENTS_END;

    if (result != 0) {
        return 1;
    }

    return 0;
}

int shockburst_rx_read(uint8_t* payload) {
    int result = 0;

    NRF_RADIO->EVENTS_END = 0U; // Clear END

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        result = -EBUSY;
#ifdef CONFIG_LOG
        LOG_ERR("Radio war im falschen State: %d", NRF_RADIO->STATE);
#endif
        return result;
    }

    // Copy rx_payload_buffer to payload argument
    memcpy(payload, rx_payload_buffer, payload_length);

    // Check if CRC of received packet is okay
    if (NRF_RADIO->CRCSTATUS == 0U) {
        // Error
        result = -ECHK;
    }

    // THE RECEIVED MESSAGE IS STILL COPIED INTO PAYLOAD, EVEN IF THE CRC FAILED!
    // THE USER CAN THUS DECIDE IF HE WANTS TO DISCARD IT

    // Start listening to packets again
    NRF_RADIO->TASKS_START = 1U;

    return result;
}

int shockburst_rx_read_idle(uint8_t* payload) {
    int result = 0;

    NRF_RADIO->EVENTS_END = 0U; // Clear END

    // We do not start listening to packets again; thus the radio stays in RX_IDLE
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        result = -EPERWS;
#ifdef CONFIG_LOG
        LOG_ERR("Radio war im falschen State: %d", NRF_RADIO->STATE);
#endif
        return result;
    }

    // Copy rx_payload_buffer to payload argument
    memcpy(payload, rx_payload_buffer, payload_length);

    // Check if CRC of received packet is okay
    if (NRF_RADIO->CRCSTATUS == 0U) {
        // Error
        result = -ECHK;
        return result;
    }

    // THE RECEIVED MESSAGE IS STILL COPIED INTO PAYLOAD, EVEN IF THE CRC FAILED!
    // THE USER CAN THUS DECIDE IF HE WANTS TO DISCARD IT

    return result;
}

int shockburst_rx_start_listening_interrupt(void (*handler)(enum shockburst_event)) {
    int result = 0;

    // Check if radio is already listening
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx) {
        return result;
    }

    event_handler = handler;

    NRF_RADIO->PACKETPTR = (uint32_t)rx_payload_buffer;
    
    // If radio is disabled then start it up
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) {
        NRF_RADIO->EVENTS_READY = 0U; // Clear EVENTS_READY register
        NRF_RADIO->TASKS_RXEN = 1U; // Enable Radio in RX mode

        while (NRF_RADIO->EVENTS_READY == 0U) {
            // wait for EVENTS_READY a.k.a. Radio has ramped up in RX mode and is ready to be started
        }
    }

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        result = -EPERWS;
        return result;
    }

    // Enable Interrupt for EVENTS_END (Packet send or received)
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

    NRF_RADIO->EVENTS_END = 0U; // Clear EVENTS_END
    NRF_RADIO->TASKS_START = 1U; // Start Radio a.k.a. listening for receiving packets

    return result;
}

int shockburst_rx_stop_listening_interrupt() {
    int result = 0;

    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled) {
        return result;
    }

    //
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx) {
        NRF_RADIO->TASKS_STOP = 1U;

        while(NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        // Wait until ready is in RXIDLE
        }
    }

    // Disable Interrupt for EVENTS_END (Packet send or received)
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;

    NRF_RADIO->EVENTS_DISABLED = 0U; // Clear EVENTS_DISABLED register
    NRF_RADIO->TASKS_DISABLE = 1U; // Disable Radio

    while (NRF_RADIO->EVENTS_DISABLED == 0U) {
        // wait for Radio to be disabled
    }

    return result;
}

int shockburst_rx_pause_listening_interrupt() {
    int result = 0;

    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled) {
        return result;
    }

    //
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx) {
        NRF_RADIO->TASKS_STOP = 1U;

        while(NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        // Wait until ready is in RXIDLE
        }
    }

    // Disable Interrupt for EVENTS_END (Packet send or received)
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;

    return result;
}