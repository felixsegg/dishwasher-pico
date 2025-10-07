//
// Created by Felix on 07.10.2025.
//

#ifndef DISHWASHER_PICO_PULSE_DETECTOR_H
#define DISHWASHER_PICO_PULSE_DETECTOR_H

#include "gpio_edge_receiver.h"
#include "pico/util/queue.h"

typedef struct {
    uint64_t timestamp_us; // Start of the pulse
    uint32_t duration_us; // Duration of the pulse
} pulse_t;

void process_edges(queue_t *input, queue_t *output, signal_state_t idle_state);
bool init_pulse_detector(queue_t *pulse_queue);

#endif //DISHWASHER_PICO_PULSE_DETECTOR_H