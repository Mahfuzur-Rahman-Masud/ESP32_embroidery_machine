#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "spi_m.h"



/**
 * @brief Manages the SPI master transmission loop, handling standard sequential 
 * data packets, flow control tracking, and queue discard commands.
 */
void spi_master_task(void *pvParameters) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(spi_payload_t)
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY
    };

    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    spi_payload_t tx_msg = {0};
    spi_payload_t rx_msg = {0};
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = sizeof(spi_payload_t) * 8;
    t.tx_buffer = &tx_msg;
    t.rx_buffer = &rx_msg;

    uint32_t expected_ack = 1;
    uint8_t slave_free_slots = 0;
    bool fetch_new_line = true;

    while (1) {
        if (check_flush_condition()) {
            tx_msg.id = 0;
            tx_msg.type = 2;
            memset(tx_msg.data, 0, sizeof(tx_msg.data));
            
            if (spi_device_transmit(spi, &t) == ESP_OK) {
                expected_ack = 1;
                fetch_new_line = true;
            }
            continue;
        }

        if (fetch_new_line) {
            tx_msg.id = expected_ack;
            tx_msg.type = 1;
            nextLine(tx_msg.data);
        } else {
            line(tx_msg.data);
        }

        if (slave_free_slots == 0 && expected_ack > 1) {
            tx_msg.type = 0; 
            memset(tx_msg.data, 0, sizeof(tx_msg.data));
        }

        esp_err_t ret = spi_device_transmit(spi, &t);
        
        if (ret == ESP_OK) {
            slave_free_slots = rx_msg.rx_free;
            
            if (tx_msg.type == 0) {
                continue;
            }

            if (rx_msg.packet_ack == expected_ack) {
                expected_ack++;
                fetch_new_line = true;
            } else {
                fetch_new_line = false;
            }
        } else {
            fetch_new_line = false;
        }
    }
}