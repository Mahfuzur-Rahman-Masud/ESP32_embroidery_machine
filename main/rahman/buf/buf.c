#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t id;
    char* data[32];
} m_t;

#define Q_MAX_SIZE 8

m_t* q[Q_MAX_SIZE];
int8_t head = 0;
int8_t tail = 0;
int8_t size = 0;

// static portMUX_TYPE q_mux = portMUX_INITIALIZER_UNLOCKED;

void clear() {
    // portENTER_CRITICAL(&q_mux);

    head = 0;
    tail = 0;
    size = 0;

    for (int8_t i = 0; i < Q_MAX_SIZE; i++) {
        q[i] = NULL;
    }

    // portEXIT_CRITICAL(&q_mux);
}

void add(m_t* data) {
    // portENTER_CRITICAL(&q_mux);

    q[tail] = data;
    tail = (tail + 1) % Q_MAX_SIZE;
    
    if (size < Q_MAX_SIZE) {
        size++;
    } else {
        head = (head + 1) % Q_MAX_SIZE;
    }

    // portEXIT_CRITICAL(&q_mux);
}

bool poll(m_t* data) {
    bool success = false;
    // portENTER_CRITICAL(&q_mux);

    if (size > 0) {
        if (data != NULL && q[head] != NULL) {
            *data = *q[head];
        }
    
        q[head] = NULL;
        head = (head + 1) % Q_MAX_SIZE;
        size--;
        success = true;
    }

    // portEXIT_CRITICAL(&q_mux);
    return success;
}


bool poll() {
    bool success = false;
    // portENTER_CRITICAL(&q_mux);

    if (size > 0) {  
        q[head] = NULL;
        head = (head + 1) % Q_MAX_SIZE;
        size--;
        success = true;
    }

    // portEXIT_CRITICAL(&q_mux);
    return success;
}

bool peek(m_t* data) {
    bool success = false;
    // portENTER_CRITICAL(&q_mux);

    if (size > 0) {
        if (data != NULL && q[head] != NULL) {
            *data = *q[head];
        }
        success = true;
    }

    // portEXIT_CRITICAL(&q_mux);
    return success;
}
