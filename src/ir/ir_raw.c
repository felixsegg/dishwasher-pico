//
// Created by Felix on 03.10.2025.
//

#include "ir/ir_raw.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/util/queue.h"

#define IDLE_THRESHOLD_US 30000
#define FIND_IDLE_TIMEOUT_US 1000000
#define EDGE_POOL_SIZE 128

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

typedef struct {
    uint64_t timestamp_us; // Start of the pulse
    uint32_t duration_us; // Duration of the pulse
} pulse_t;

typedef enum {
    INVALID, // Error type
    ZERO, // Representing a logical 0 from the nec signal
    ONE, // Representing a logical 1 from the nec signal
    START, // Representing a start signal from the nec signal
    REPEAT, // Representing a repeat signal from the nec signal
    END // Representing the final pulse after either a message or repeat code transmission
} nec_code_t;

typedef struct {
    uint32_t data; // The data read.
    uint step; // The counter in the building process. The first 31 steps are reading the 32 bits, then repeats.
    bool implicit_end; // A flag to memorize if an end signal was already anticipated in the previous iteration.
} raw_nec_msg_builder_t;

typedef struct {
    uint8_t adress;
    uint8_t command;
    bool initial;
} nec_msg_t;

static uint8_t ir_input_gpio;
static signal_state_t idle_state;

static edge_t edge_pool[EDGE_POOL_SIZE];
static queue_t edge_queue;
static queue_t free_pool_queue;
static queue_t pulse_queue;

static raw_nec_msg_builder_t *cur_nec_msg_ptr = NULL;

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

void __not_in_flash_func(pulse_handler)(uint gpio, uint32_t events) {
    uint64_t now = time_us_64();

    edge_t *edge;

    if (!queue_try_remove(&free_pool_queue, &edge)) {
        return; // TODO: Maybe dump/reset entire queues?
    }

    edge->timestamp = now;
    edge->edge = (events & GPIO_IRQ_EDGE_RISE) != 0 ? RISE : FALL;

    queue_try_add(&edge_queue, edge);
}

void process_edges() {
    edge_t *edge1;
    edge_t *edge2;
    while (true) {
        uint32_t flags = save_and_disable_interrupts(); // Interrupt would make reading the queue unsafe
        if (queue_get_level_unsafe(&edge_queue) < 2) {
            // unsafe is faster and disabled interrupts already guarantee safety
            restore_interrupts(flags);
            break;
        }

        queue_try_remove(&edge_queue, &edge1);

        if ((int) edge1->edge == (int) idle_state) {
            restore_interrupts(flags);
            continue; // started measuring during a pulse, not processable
        }

        queue_try_remove(&edge_queue, &edge2);
        restore_interrupts(flags);

        pulse_t *pulse = malloc(sizeof(pulse_t));
        pulse->timestamp_us = edge1->timestamp;
        pulse->duration_us = edge2->timestamp - edge1->timestamp;
        queue_try_add(&pulse_queue, pulse);

        queue_add_blocking(&free_pool_queue, &edge1); // this should never block, so if it does, I deserve it lol
    }
}

static nec_code_t get_next_nec_code(const pulse_t *pulse1, const pulse_t *pulse2) {
    // Sorry for the following magic numbers, just look up the nec transmission protocol in order to understand them.
    // The margins are arbitrary.
    const uint32_t pulse_dur = pulse1->duration_us;
    const uint32_t space_dur = pulse2->timestamp_us - (pulse1->timestamp_us + pulse1->duration_us);
    if (pulse_dur > 500 && pulse_dur < 650) {
        // ~562.5 us
        if (space_dur > 500 && space_dur < 650) // ~562.5 us
            return ZERO;
        if (space_dur > 1600 && space_dur < 1750) // ~1687.5 us
            return ONE;
        if (space_dur >= 1750)
            return END;
    } else if (pulse_dur > 8000 && pulse_dur < 10000) {
        // 9 ms ± 1
        if (space_dur > 2000 && space_dur < 2500)
            return REPEAT;
        if (space_dur > 4000 && space_dur < 5000) // 4.5 ms ± 0.5
            return START;
    }
    return INVALID;
}

static int checkout_nec_msg(const uint32_t data, const bool initial) {
    const uint8_t adr = (data >>  0) & 0xFF;
    const uint8_t adr_inv = (data >>  8) & 0xFF;
    const uint8_t cmd = (data >> 16) & 0xFF;
    const uint8_t cmd_inv = (data >> 24) & 0xFF;

    if (adr != ~adr_inv || cmd != ~cmd_inv)
        return -1;

    nec_msg_t *msg = malloc(sizeof(nec_msg_t));
    msg->adress = adr;
    msg->command = cmd;
    msg->initial = initial;

    // TODO: Send msg

    return 0;
}

static int64_t auto_end_nec_signal() {
    // TODO: Send end of signal message

    return 0;
}

void process_nec_pulses() {
    while (true) {
        if (queue_get_level(&pulse_queue) < 2)
            break;

        pulse_t *pulse1;
        pulse_t *pulse2;

        queue_try_remove(&pulse_queue, &pulse1);
        queue_try_peek(&pulse_queue, &pulse2);

        nec_code_t cur_nec_code = get_next_nec_code(pulse1, pulse2);

        if (cur_nec_msg_ptr == NULL) {
            ///////////////////////////////////////////////
            /// NO CURRENT BUILD OF MESSAGE IN PROGRESS ///
            ///////////////////////////////////////////////

            if (cur_nec_code == START) {
                // START signal -> A new message follows
                cur_nec_msg_ptr = calloc(1, sizeof(raw_nec_msg_builder_t));
            }
            continue;
        }

        ////////////////////////////////////////////
        /// CURRENT BUILD OF MESSAGE IN PROGRESS ///
        ////////////////////////////////////////////

        if (cur_nec_code == START) {
            // New one
            memset(cur_nec_msg_ptr, 0, sizeof(raw_nec_msg_builder_t));
        } else if (cur_nec_code == END && cur_nec_msg_ptr->implicit_end) {
            // END signals should always be anticipated. Abort message if occurs unexpectedly
            cur_nec_msg_ptr->implicit_end = false;
        } else if (cur_nec_msg_ptr->step < 32 && (cur_nec_code == ZERO || cur_nec_code == ONE)) {
            // information bit was read
            if (cur_nec_code == ONE) // ZERO leaves it just at 0
                cur_nec_msg_ptr->data |= (UINT32_C(1) << cur_nec_msg_ptr->step);
            if (cur_nec_msg_ptr->step == 31) {
                // Anticipate next signal (END signal)
                cur_nec_msg_ptr->implicit_end = true;
                // Send
                checkout_nec_msg(cur_nec_msg_ptr->data, true);
                add_alarm_in_us(50000, auto_end_nec_signal, NULL, true); // TODO: Implement part in this function as well.
            }
        } else if (cur_nec_msg_ptr->step > 32 && !(cur_nec_msg_ptr->step % 2) && cur_nec_code == REPEAT) {
            // From here on only repeats are valid. If pos is an even number, it can only be END if valid and the
            // program never would've reached this point. If even and no END -> Exception
            // Anticipate next signal (END signal)
            cur_nec_msg_ptr->implicit_end = true;
            // Send repeat
            checkout_nec_msg(cur_nec_msg_ptr->data, false);
            add_alarm_in_us(105000, auto_end_nec_signal, NULL, true); // TODO
        } else {
            // Exception, abort
            free(cur_nec_msg_ptr);
            cur_nec_msg_ptr = NULL;
            continue;
        }

        // Always increment the counter
        cur_nec_msg_ptr->step++;
    }
}

bool init_ir_raw(uint8_t pin, idle_mode_t idle) {
    ir_input_gpio = pin;

    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    if (idle == IDLE_MODE_LOW) {
        idle_state = STATE_LOW;
    } else if (idle == IDLE_MODE_HIGH) {
        idle_state = STATE_HIGH;
    } else if (idle == IDLE_MODE_AUTO) {
        const int found = find_idle_ir(pin);
        if (found == -1) {
            return false;
        }
        idle_state = found;
    } else {
        return false;
    }

    queue_init(&edge_queue, sizeof(edge_t *), EDGE_POOL_SIZE);
    queue_init(&free_pool_queue, sizeof(queue_t *), EDGE_POOL_SIZE);
    queue_init(&pulse_queue, sizeof(pulse_t *), EDGE_POOL_SIZE / 2);
    for (int i = 0; i < EDGE_POOL_SIZE; i++) {
        edge_t *ptr = &edge_pool[i];
        queue_add_blocking(&free_pool_queue, &ptr);
    }

    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_handler);
    return true;
}
