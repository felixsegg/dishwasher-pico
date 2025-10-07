//
// Created by Felix on 07.10.2025.
//

#include "ir/gpio_edge_receiver.h"

#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/util/queue.h"

#define IDLE_THRESHOLD_US 30000
#define FIND_IDLE_TIMEOUT_US 1000000
#define MAX_EDGES 128

static queue_t* edge_queue;

static int find_idle_ir(uint8_t gpio) {
    const uint64_t start = time_us_64();
    bool cur = gpio_get(gpio);
    uint64_t last_edge = start;
    while (time_us_64() < start + FIND_IDLE_TIMEOUT_US) {
        if (cur != gpio_get(gpio)) {
            cur = gpio_get(gpio);
            last_edge = time_us_64();
        } else if (time_us_64() > last_edge + 30000) {
            return cur;
        }
    }
    return -1;
}

static void __not_in_flash_func(edge_handler)(uint gpio, uint32_t events) {
    uint64_t now = time_us_64();

    edge_t *edge = malloc(sizeof(edge_t)); // NOLINT - ownership goes to the receiving end of the queue
    edge->timestamp = now;
    edge->edge = (events & GPIO_IRQ_EDGE_RISE) != 0 ? RISE : FALL;

    queue_try_add(edge_queue, edge);
}


bool init_gpio_edge_receiver(uint8_t pin, idle_mode_t idle, queue_t *output, signal_state_t *idle_state) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    if (idle == IDLE_MODE_LOW) {
        *idle_state = STATE_LOW;
    } else if (idle == IDLE_MODE_HIGH) {
        *idle_state = STATE_HIGH;
    } else if (idle == IDLE_MODE_AUTO) {
        const int found = find_idle_ir(pin);
        *idle_state = found;
        if (found == -1) {
            return false;
        }
    } else {
        return false;
    }

    queue_init(output, sizeof(edge_t *), MAX_EDGES);
    edge_queue = output;

    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &edge_handler);
    return true;
}