#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lora_driver.h"
#include "config.h"       // All config comes from here - ONE source of truth!
#include "lwip/def.h"     // For htonl, ntohl - Network byte order!

static const char *TAG = "LORA_DRIVER";

// Pins and LoRa parameters now come from config.h - ORGANIZED!

// ============================================================
// SX1276 Register Addresses - The chip's memory map!
// ============================================================
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_SYMB_TIMEOUT_LSB     0x1F
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// ============================================================
// Operating Modes - What the radio should be doing!
// ============================================================
#define MODE_LONG_RANGE_MODE     0x80   // LoRa mode - Long range is BEAUTIFUL!
#define MODE_SLEEP               0x00   // Sleep mode - Low power
#define MODE_STDBY               0x01   // Standby - Ready to work
#define MODE_TX                  0x03   // Transmit mode - SENDING!
#define MODE_RX_CONTINUOUS       0x05   // Continuous receive - Always listening!

spi_device_handle_t spi;

// ============================================================
// SPI Basic Operations - Talking to the chip!
// ============================================================

/**
 * Write a value to a register - Simple and effective!
 */
void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t out[2] = { reg | 0x80, val };
    uint8_t in[2];
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * 2,
        .tx_buffer = out,
        .rx_buffer = in
    };
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
}

/**
 * Read a value from a register - Get that data!
 */
uint8_t lora_read_reg(uint8_t reg) {
    uint8_t out[2] = { reg & 0x7f, 0xFF };
    uint8_t in[2];
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * 2,
        .tx_buffer = out,
        .rx_buffer = in
    };
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
    return in[1];
}

/**
 * User-friendly frequency setter
 * Formula: Frf = (Freq_MHz * 2^19) / 32
 * Result is a 24-bit value split into 3 registers. MATH!
 */
void lora_set_frequency(uint32_t freq_mhz) {
    uint64_t frf = ((uint64_t)freq_mhz << 19) / 32;
    
    uint8_t msb = (frf >> 16) & 0xFF;
    uint8_t mid = (frf >> 8) & 0xFF;
    uint8_t lsb = frf & 0xFF;
    
    lora_write_reg(REG_FRF_MSB, msb);
    lora_write_reg(0x07, mid);
    lora_write_reg(0x08, lsb);
    
    ESP_LOGI(TAG, "LoRa Frequency Set: %lu MHz (Regs: 0x%02X%02X%02X) - TREMENDOUS!", 
             freq_mhz, msb, mid, lsb);
}

/**
 * Initialize LoRa radio - Full configuration. THE WORKS!
 * All parameters MUST match your teammates. No exceptions!
 */
void lora_init(void) {
    // Step 1: GPIO initialization - Set up the pins!
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);

    // Step 2: Hardware reset - Fresh start!
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 3: SPI bus initialization - High speed communication!
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz - Fast enough!
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    // Step 4: Check chip version (SX1276 should be 0x12)
    uint8_t version = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "LoRa Chip Version: 0x%02x", version);
    if (version != 0x12) {
        ESP_LOGE(TAG, "Unrecognized LoRa chip! Check your wiring - SAD!");
    }

    // Step 5: Enter sleep mode for configuration
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 6: Enter standby mode
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 7: Set frequency (868.2 MHz for Europe - Legal and POWERFUL!)
    uint64_t frf = (uint64_t)(LORA_FREQ_MHZ * 1000000.0) * (1 << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    lora_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    lora_write_reg(REG_FRF_LSB, frf & 0xFF);
    ESP_LOGI(TAG, "Frequency: %.1f MHz (Frf=0x%06llX) - PERFECT!", LORA_FREQ_MHZ, frf);

    // Step 8: Configure modulation - MUST match teammates!
    // Modem Config 1: BW[7:4] + CR[3:1] + ImplicitHeader[0]
    uint8_t bw_reg = 0x80;  // 250 kHz bandwidth - Nice and wide!
    uint8_t cr_reg = ((LORA_CR - 4) & 0x07) << 1;  // Coding rate
    lora_write_reg(REG_MODEM_CONFIG_1, bw_reg | cr_reg);
    ESP_LOGI(TAG, "BW=250kHz, CR=4/%d, Config1=0x%02X", LORA_CR, bw_reg | cr_reg);

    // Modem Config 2: SF[7:4] + CRC settings
    uint8_t sf_reg = (LORA_SF << 4);
    lora_write_reg(REG_MODEM_CONFIG_2, sf_reg | 0x04);  // CRC enabled!
    ESP_LOGI(TAG, "SF=%d, CRC=ON, Config2=0x%02X - SECURE!", LORA_SF, sf_reg | 0x04);

    // Modem Config 3: AGC auto on
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);

    // Step 9: Set Sync Word - Like a secret handshake!
    lora_write_reg(REG_SYNC_WORD, LORA_SYNCWORD);
    ESP_LOGI(TAG, "SyncWord=0x%02X - EXCLUSIVE!", LORA_SYNCWORD);

    // Step 10: Set Preamble length
    lora_write_reg(REG_PREAMBLE_MSB, (LORA_PREAMBLE >> 8) & 0xFF);
    lora_write_reg(REG_PREAMBLE_LSB, LORA_PREAMBLE & 0xFF);
    ESP_LOGI(TAG, "Preamble=%d symbols", LORA_PREAMBLE);

    // Step 11: Configure PA (Power Amplifier) - MAXIMUM POWER!
    lora_write_reg(REG_PA_CONFIG, 0x8F);  // PA_BOOST + max power
    
    // Step 12: Configure OCP (Overcurrent Protection)
    lora_write_reg(REG_OCP, 0x2B);  // 100mA limit - Safe!

    // Step 13: Configure LNA (Low Noise Amplifier) - Hear EVERYTHING!
    lora_write_reg(REG_LNA, 0x23);  // Max gain + LNA boost

    // Step 14: Set FIFO base addresses
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Step 15: Enter continuous receive mode - Always ready!
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "LoRa READY! Freq=%.1fMHz, BW=250kHz, SF=%d, CR=4/%d", 
             LORA_FREQ_MHZ, LORA_SF, LORA_CR);
    ESP_LOGI(TAG, "SyncWord=0x%02X, Preamble=%d, CRC=ON - TREMENDOUS!", 
             LORA_SYNCWORD, LORA_PREAMBLE);
    ESP_LOGI(TAG, "========================================");
}

/**
 * Send a LoRa packet - Broadcast to the world!
 * Waits for TxDone flag instead of fixed delay.
 */
void lora_send_packet(lora_packet_t *packet) {
    // Step 1: Go to standby mode for FIFO access
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    
    // Step 2: Reset FIFO pointer
    lora_write_reg(REG_FIFO_ADDR_PTR, 0); 
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);

    // Step 3: Write packet data to FIFO
    uint8_t *data = (uint8_t*)packet;
    int len = sizeof(lora_packet_t);
    
    for(int i=0; i<len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
    lora_write_reg(REG_PAYLOAD_LENGTH, len);

    // Step 4: Start transmission
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    // Step 5: Wait for TX complete by polling TxDone flag
    // OLD METHOD (commented out):
    // vTaskDelay(pdMS_TO_TICKS(50));  // Fixed 50ms wait - may be too short!
    
    // NEW METHOD: Poll TxDone flag (bit 3 = 0x08)
    uint32_t timeout = 0;
    while (timeout < 400) {  // 400ms max timeout
        uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);
        if (irq & 0x08) {  // TxDone flag set
            ESP_LOGI(TAG, "TX Done after %lums", timeout);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout += 10;
    }
    
    if (timeout >= 400) {
        ESP_LOGW(TAG, "TX timeout! Forcing RX mode");
    }
    
    // Step 6: Clear all IRQ flags (important for clean RX state!)
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    
    // Step 7: Return to continuous RX mode
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    ESP_LOGI(TAG, "Packet Sent! Seq: %d", ntohs(packet->seq_number));
}

/**
 * Receive a LoRa packet - Check if someone's talking!
 * Non-blocking - returns 0 if no packet, 1 if packet received.
 */
int lora_receive_packet(lora_packet_t *packet) {
    // Check IRQ flags for RxDone
    uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);
    
    // Debug: Print IRQ status periodically (avoid spam!)
    static int rx_check_count = 0;
    if (++rx_check_count >= 100) {
        rx_check_count = 0;
        if (irq != 0) {
            ESP_LOGI(TAG, "RX Check: IRQ=0x%02X", irq);
        }
    }
    
    // No RxDone flag? No packet. Exit fast!
    if ((irq & 0x40) == 0) return 0;

    // Clear interrupt flags
    lora_write_reg(REG_IRQ_FLAGS, irq);

    // Get payload length
    uint8_t len = lora_read_reg(REG_RX_NB_BYTES);
    ESP_LOGI(TAG, "RX: Got packet! IRQ=0x%02X, len=%d (expected=%d)", irq, len, sizeof(lora_packet_t));
    
    // Size check - Must match our packet structure!
    if (len != sizeof(lora_packet_t)) {
        ESP_LOGW(TAG, "Packet size mismatch: %d bytes - REJECTED!", len);
        return 0;
    }

    // Set read pointer to packet start
    uint8_t current_addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR, current_addr);

    // Read packet data from FIFO
    uint8_t *data = (uint8_t*)packet;
    for(int i=0; i<len; i++) {
        data[i] = lora_read_reg(REG_FIFO);
    }
    
    return 1;  // SUCCESS! Beautiful packet received!
}