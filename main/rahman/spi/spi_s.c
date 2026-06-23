#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "spi_s.h"


#define BUFFER_POOL_SIZE (QUEUE_CAPACITY + 2)


static QueueHandle_t work_queue;
static QueueHandle_t free_queue;

static WORD_ALIGNED_ATTR spi_payload_t tx_msg = {0};
static WORD_ALIGNED_ATTR spi_payload_t rx_pool[BUFFER_POOL_SIZE];

static volatile uint32_t last_received_id = 0;
static volatile uint32_t last_completed_id = 0;

/**
 * @brief Flushes all pending transactions within the work queue and returns 
 * the memory buffers back to the tracking allocation pool.
 */
static void flush_work_queue(void) {
    spi_payload_t *payload_ptr;
    while (xQueueReceive(work_queue, &payload_ptr, 0) == pdTRUE) {
        xQueueSend(free_queue, &payload_ptr, 0);
    }
    last_received_id = 0;
    last_completed_id = 0;
}

/**
 * @brief Fetches a single completed payload pointer from the inbound work queue.
 */
bool spi_slave_receive_command(spi_payload_t **payload) {
    if (xQueueReceive(work_queue, payload, 0) == pdTRUE) {
        return true;
    }
    return false;
}

/**
 * @brief Recycles a processed data buffer back into the allocation pool and 
 * stores its completion sequence context.
 */
void spi_slave_acknowledge_done(spi_payload_t *payload) {
    if (payload != NULL) {
        last_completed_id = payload->id;
        xQueueSend(free_queue, &payload, portMAX_DELAY);
    }
}

/**
 * @brief Controls the hardware slave communication interface, handles packet ordering, 
 * validates incoming buffers, and responds to master-initiated buffer purges.
 */
void spi_slave_task(void *pvParameters) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    work_queue = xQueueCreate(QUEUE_CAPACITY, sizeof(spi_payload_t*));
    free_queue = xQueueCreate(BUFFER_POOL_SIZE, sizeof(spi_payload_t*));

    for (int i = 0; i < BUFFER_POOL_SIZE; i++) {
        spi_payload_t *ptr = &rx_pool[i];
        xQueueSend(free_queue, &ptr, 0);
    }

    spi_slave_transaction_t t;

    while (1) {
        UBaseType_t items_in_queue = uxQueueMessagesWaiting(work_queue);
        tx_msg.rx_free = (items_in_queue >= QUEUE_CAPACITY) ? 0 : (QUEUE_CAPACITY - items_in_queue);
        tx_msg.packet_ack = last_received_id;
        tx_msg.packet_done = last_completed_id;

        spi_payload_t *current_rx_buf = NULL;
        if (xQueueReceive(free_queue, &current_rx_buf, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        memset(&t, 0, sizeof(t));
        t.length = sizeof(spi_payload_t) * 8;
        t.tx_buffer = &tx_msg;
        t.rx_buffer = current_rx_buf;

        esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        if (ret == ESP_OK) {
            if (current_rx_buf->type == 2) {
                flush_work_queue();
                xQueueSend(free_queue, &current_rx_buf, 0);
            } else if (current_rx_buf->type == 1 && current_rx_buf->id == (last_received_id + 1)) {
                if (xQueueSend(work_queue, &current_rx_buf, 0) == pdTRUE) {
                    last_received_id = current_rx_buf->id;
                } else {
                    xQueueSend(free_queue, &current_rx_buf, 0);
                }
            } else {
                xQueueSend(free_queue, &current_rx_buf, 0);
            }
        } else {
            xQueueSend(free_queue, &current_rx_buf, 0);
        }
    }
}