//
// Created by Felix on 03.10.2025.
//

#include "ir_raw.h"

#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/util/queue.h"

#define IDLE_THRESHOLD_US 30000
#define FIND_IDLE_TIMEOUT_US 1000000
#define EDGE_POOL_SIZE 128

typedef struct {
    uint64_t timestamp;
    bool edge;
} edge_t;

typedef enum {
    IDLE_LOW = 0,
    IDLE_HIGH = 1,
    IDLE_AUTO = 2
} idle_mode_t;

static uint8_t ir_input_gpio;
static uint8_t pulse_idle;

static edge_t edge_pool[EDGE_POOL_SIZE];
static queue_t edge_queue;
static queue_t free_pool_queue; // enthält edge_t*

static int find_idle_ir(uint8_t gpio) {
    const uint64_t start = time_us_64();
    bool cur = gpio_get(gpio);
    uint64_t last_edge = start;
    while (time_us_64() < start + FIND_IDLE_TIMEOUT_US) {
        if (cur != gpio_get(gpio)) {
            cur = gpio_get(gpio);
            last_edge = time_us_64();
        }
        else if (time_us_64() > last_edge + 30000) {
            return cur;
        }
    }
    return -1;
}

void __not_in_flash_func(pulse_handler)(uint gpio, uint32_t events) {
    uint64_t now = time_us_64();

    edge_t *edge;

    if (!queue_try_remove(&free_pool_queue, &edge)) {
        return; // TODO: Maybe dump/reset entire queues?
    }

    edge->timestamp = now;
    edge->edge = (events & GPIO_IRQ_EDGE_RISE) != 0;

    queue_try_add(&edge_queue, edge);
}

void process_edges() {
    // temporary
    edge_t *edge;
    while (queue_try_remove(&edge_queue, &edge)) {
        // TODO: process the edges
        queue_add_blocking(&free_pool_queue, &edge); // this should never block, so if it does I deserve it lol
    }
}

bool init_ir_raw(uint8_t pin, idle_mode_t idle) {
    ir_input_gpio = pin;

    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    if (idle == IDLE_LOW) {
        pulse_idle = 0;
    } else if (idle == IDLE_HIGH) {
        pulse_idle = 1;
    } else if (idle == IDLE_AUTO) {
        const int found = find_idle_ir(pin);
        if (found == -1) {
            return false;
        }
        pulse_idle = found;
    } else {
        return false;
    }

    queue_init(&edge_queue, sizeof(edge_t *), EDGE_POOL_SIZE);
    queue_init(&free_pool_queue, sizeof(queue_t *), EDGE_POOL_SIZE);
    for (int i = 0; i < EDGE_POOL_SIZE; i++) {
        edge_t *ptr = &edge_pool[i];
        queue_add_blocking(&free_pool_queue, &ptr);
    }

    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_handler);
    return true;
}


