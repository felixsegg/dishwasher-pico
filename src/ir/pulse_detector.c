//
// Created by Felix on 07.10.2025.
//

#include "ir/pulse_detector.h"

#include <stdlib.h>
#include <string.h>

#include "../../include/ir/gpio_edge_receiver.h"
#include "pico/util/queue.h"

#define MAX_PULSES 64

void process_edges(queue_t *input, queue_t *output, signal_state_t idle_state) {
    edge_t *edge1;
    edge_t *edge2;
    while (true) {
        uint32_t flags = save_and_disable_interrupts(); // Interrupt would make reading the queue unsafe
        if (queue_get_level_unsafe(input) < 2) {
            // unsafe is faster and disabled interrupts already guarantee safety
            restore_interrupts(flags);
            break;
        }

        queue_try_remove(input, &edge1);

        if ((int) edge1->edge == (int) idle_state) {
            restore_interrupts(flags);
            continue; // started measuring during a pulse, not processable
        }

        queue_try_remove(input, &edge2);
        restore_interrupts(flags);

        pulse_t *pulse = malloc(sizeof(pulse_t)); // NOLINT - It's not leaked
        pulse->timestamp_us = edge1->timestamp;
        pulse->duration_us = edge2->timestamp - edge1->timestamp;
        queue_try_add(output, pulse);

        free(edge1);
    }
}

bool init_pulse_detector(queue_t *pulse_queue) {
    queue_init(pulse_queue, sizeof(pulse_t *), MAX_PULSES / 2);
    return true;
}
