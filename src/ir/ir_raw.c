//
// Created by Felix on 03.10.2025.
//

#include "ir/ir_raw.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/util/queue.h"
#include "cmd_listener.c" // TODO: replace by .h
#include "pico/mutex.h"

#define IDLE_THRESHOLD_US 30000
#define FIND_IDLE_TIMEOUT_US 1000000
#define EDGE_POOL_SIZE 128
#define AUTO_END_AFTER_US 125000

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
    uint step; // The counter in the building process. The first 31 steps are reading the 32 bits, then repeat signals.
    bool implicit_end; // A flag to memorize if an end signal was already anticipated in the previous iteration.
    mutex_t mutex; // To asynchronously check EoS
    uint64_t spared_until; // Timestamp in us marking the signal up to that point as not cancellable.
    bool is_cancelled;
} raw_nec_msg_builder_t;

static uint8_t ir_input_gpio;
static signal_state_t idle_state;

static edge_t edge_pool[EDGE_POOL_SIZE];
static queue_t edge_queue;
static queue_t free_pool_queue;
static queue_t pulse_queue;

static raw_nec_msg_builder_t cur_nec_msg = {0};

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

        pulse_t *pulse = malloc(sizeof(pulse_t)); // NOLINT - It's not leaked
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

static int checkout_nec_msg(const uint32_t *data) {
    if (data == NULL) {
        end_last_command();
    } else {
        const uint8_t adr = (*data >> 0) & 0xFF;
        const uint8_t adr_inv = (*data >> 8) & 0xFF;
        const uint8_t cmd = (*data >> 16) & 0xFF;
        const uint8_t cmd_inv = (*data >> 24) & 0xFF;

        if (adr != ~adr_inv || cmd != ~cmd_inv)
            return -1;

        new_command(adr, cmd);
    }

    return 0;
}

static void reset_raw_nec_msg_builder(raw_nec_msg_builder_t *msg_builder) {
    if (msg_builder == NULL)
        return;

    msg_builder->data = 0; // The data read.
    msg_builder->step = 0; // The counter in the building process. The first 31 steps are reading the 32 bits, then repeat signals.
    msg_builder->implicit_end = false; // A flag to memorize if an end signal was already anticipated in the previous iteration.
    msg_builder->spared_until = 0; // Timestamp in us marking the signal up to that point as not cancellable.
    msg_builder->is_cancelled = false;

}

static int64_t end_nec_signal(alarm_id_t id, void *safety_guaranteed) {
    // Either the caller already guarantees safety (in other words, already has the mutex locked) or we have to lock it
    // ourselves.
    if ((safety_guaranteed != NULL && *(bool *) safety_guaranteed || mutex_try_enter(&cur_nec_msg.mutex, NULL))
        && !cur_nec_msg.is_cancelled && cur_nec_msg.spared_until < time_us_64()) {
        end_last_command();
        if (!(safety_guaranteed != NULL && *(bool *) safety_guaranteed))
            mutex_exit(&cur_nec_msg.mutex);
        cur_nec_msg.is_cancelled = true;
    }
    free(safety_guaranteed);
    return 0;
}

static void add_or_delay_end_check() {
    cur_nec_msg.spared_until = time_us_64() + AUTO_END_AFTER_US - 5000; // A 5 ms margin
    bool *safety_guaranteed = malloc(sizeof(bool)); // NOLINT - No leak, ownership transferred to end_nec_signal
    *safety_guaranteed = false;
    add_alarm_in_us(AUTO_END_AFTER_US, end_nec_signal, safety_guaranteed, true);
}

void process_nec_pulses() {
    while (true) {
        if (!mutex_try_enter(&cur_nec_msg.mutex, NULL))
            break; // An end check is already in progress, let it happen. Due to timing, this should rarely occur.

        if (queue_get_level(&pulse_queue) < 2)
            break;

        pulse_t *pulse1;
        pulse_t *pulse2;

        queue_try_remove(&pulse_queue, &pulse1);
        queue_try_peek(&pulse_queue, &pulse2);

        const nec_code_t cur_nec_code = get_next_nec_code(pulse1, pulse2);

        free(pulse1);

        if (cur_nec_msg.is_cancelled) {
            // NO CURRENT BUILD OF MESSAGE IN PROGRESS
            if (cur_nec_code == START) {
                // START signal -> A new message follows
                reset_raw_nec_msg_builder(&cur_nec_msg);
            }
        } else switch (cur_nec_code) {
            case START: {
                // CURRENT BUILD OF MESSAGE INTERRUPTED BY NEW TRANSMISSION
                // Cancel the old signal
                bool *safety_guaranteed = malloc(sizeof(bool));
                *safety_guaranteed = true;
                end_nec_signal(0, safety_guaranteed);
                // New one
                reset_raw_nec_msg_builder(&cur_nec_msg);
                // Set up a timer to check for the end after the first repeat signal was supposed to happen
                add_or_delay_end_check();
            }
            case END: {
                if (cur_nec_msg.implicit_end) {
                    // END signals should always be anticipated. Abort message if occurs unexpectedly
                    cur_nec_msg.implicit_end = false;
                } else goto exception;
                cur_nec_msg.step++;
                break;
            }
            case ONE: {
                if (cur_nec_msg.step > 31) goto exception;
                cur_nec_msg.data |= (UINT32_C(1) << cur_nec_msg.step);
            }
            case ZERO: {
                if (cur_nec_msg.step > 31) goto exception;
                if (cur_nec_msg.step == 31) {
                    // For both, ONE and ZERO.
                    // Anticipate next signal (END signal)
                    cur_nec_msg.implicit_end = true;
                    // Send
                    checkout_nec_msg(&cur_nec_msg.data);
                }
                cur_nec_msg.step++;
                break;
            }
            case REPEAT: {
                if (cur_nec_msg.step > 32 && !(cur_nec_msg.step % 2)) {
                    // From here on only repeats are valid. If step is an even number, it can only be END if valid and the
                    // program never would've reached this point. If even and no END -> Exception
                    // Anticipate next signal (END signal)
                    cur_nec_msg.implicit_end = true;
                    add_or_delay_end_check(); // Repeats only delay the end check
                    cur_nec_msg.step++;
                } else goto exception;
                break;
            }
            default: { // If this is reached something unexpected has happened, abort
            exception:;
                bool *safety_guaranteed = malloc(sizeof(bool));
                *safety_guaranteed = true;
                end_nec_signal(0, safety_guaranteed);
            }
        }

        mutex_exit(&cur_nec_msg.mutex);
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
    queue_init(&free_pool_queue, sizeof(edge_t *), EDGE_POOL_SIZE);
    queue_init(&pulse_queue, sizeof(pulse_t *), EDGE_POOL_SIZE / 2);
    for (int i = 0; i < EDGE_POOL_SIZE; i++) {
        edge_t *ptr = &edge_pool[i];
        queue_add_blocking(&free_pool_queue, &ptr);
    }

    mutex_init(&cur_nec_msg.mutex);
    cur_nec_msg.is_cancelled = true; // Initially it's basically cancelled since there's nothing being built yet

    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_handler);
    return true;
}
