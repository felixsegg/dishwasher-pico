//
// Created by Felix on 07.10.2025.
//

#ifndef DISHWASHER_PICO_GPIO_EDGE_RECEIVER_H
#define DISHWASHER_PICO_GPIO_EDGE_RECEIVER_H

#include "hardware/timer.h"
#include "pico/util/queue.h"

typedef enum {
    FALL = 0, // Edge is falling
    RISE = 1 // Edge is rising
} edge_type_t;

typedef struct {
    uint64_t timestamp; // Timestamp of when the edge occurred
    edge_type_t edge; // Type of the edge
} edge_t;

typedef enum {
    IDLE_MODE_LOW = 0, // Idle is supposed to be LOW
    IDLE_MODE_HIGH = 1, // Idle is supposed to be HIGH
    IDLE_MODE_AUTO = 2 // Idle is supposed to be automatically determined
} idle_mode_t;

typedef enum {
    STATE_LOW = 0, // Signal is LOW
    STATE_HIGH = 1 // Signal is HIGH
} signal_state_t;

bool init_gpio_edge_receiver(uint8_t pin, idle_mode_t idle, queue_t *output, signal_state_t *idle_state);

#endif //DISHWASHER_PICO_GPIO_EDGE_RECEIVER_H