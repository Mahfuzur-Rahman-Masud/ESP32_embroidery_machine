#ifndef SPI_C_H_
#define SPI_C_H_

#include <stdint.h>

#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define QUEUE_CAPACITY 10

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint8_t type;
    uint32_t packet_ack;
    uint32_t packet_done;
    uint8_t rx_free;
    uint8_t data[64];
} spi_payload_t;


void nextLine(uint8_t *buf);
void line(uint8_t *buf);
bool check_flush_condition(void);


#endif //SPI_C_H_
