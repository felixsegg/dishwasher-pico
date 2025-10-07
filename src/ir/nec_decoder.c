//
// Created by Felix on 07.10.2025.
//

#include "ir/nec_decoder.h"

#include <stdlib.h>
#include <string.h>
#include "../../include/ir/pulse_detector.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define AUTO_END_AFTER_US 125000


static nec_code_t get_next_nec_code(const pulse_t *pulse1, const pulse_t *pulse2);
static int checkout_nec_msg(const nec_decoder_t *msg_builder);
static void reset_msg_builder(nec_decoder_t *msg_builder);
static int64_t end_nec_signal(alarm_id_t id, void *end_nec_params);
static end_nec_params_t *generate_end_nec_params(nec_decoder_t *msg_builder, bool safety_guaranteed);
static void add_or_delay_end_check(nec_decoder_t *msg_builder);


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

static int checkout_nec_msg(const nec_decoder_t *msg_builder) {
    if (msg_builder != NULL) {
        const uint8_t adr = (msg_builder->data >> 0) & 0xFF;
        const uint8_t adr_inv = (msg_builder->data >> 8) & 0xFF;
        const uint8_t cmd = (msg_builder->data >> 16) & 0xFF;
        const uint8_t cmd_inv = (msg_builder->data >> 24) & 0xFF;

        if (adr != ~adr_inv || cmd != ~cmd_inv)
            return -1;

        msg_builder->callback_new_cmd(adr, cmd);
    }

    return 0;
}

static void reset_msg_builder(nec_decoder_t *msg_builder) {
    if (msg_builder == NULL)
        return;

    msg_builder->data = 0;
    msg_builder->step = 0;
    msg_builder->implicit_end = false;
    msg_builder->spared_until = 0;
    msg_builder->is_cancelled = false;
    // No need to change the mutex
}

static int64_t end_nec_signal(alarm_id_t id, void *end_nec_params) {
    if (end_nec_params == NULL) {
        return -1; // This isn't even an error code we can catch, but why not
    }
    // Either the caller already guarantees safety (in other words, already has the mutex locked) or we have to lock it
    // ourselves.
    bool safety_guaranteed = ((end_nec_params_t *) end_nec_params)->safety_guaranteed;
    nec_decoder_t *msg_builder = ((end_nec_params_t *) end_nec_params)->msg_builder;

    if ((safety_guaranteed || mutex_try_enter(&msg_builder->mutex, NULL))
        && !msg_builder->is_cancelled && msg_builder->spared_until < time_us_64()) {
        msg_builder->callback_end_last_cmd();
        if (!safety_guaranteed)
            mutex_exit(&msg_builder->mutex);
        msg_builder->is_cancelled = true;
    }
    free(end_nec_params);
    return 0;
}

static end_nec_params_t *generate_end_nec_params(nec_decoder_t *msg_builder, bool safety_guaranteed) {
    end_nec_params_t *params = malloc(sizeof(end_nec_params_t)); // NOLINT - ownership transferred to end_nec_signal
    if (params != NULL) {
        msg_builder->spared_until = time_us_64() + AUTO_END_AFTER_US - 5000; // A 5 ms margin
        params->msg_builder = msg_builder;
        params->safety_guaranteed = false;
    }
    return params;
}

static void add_or_delay_end_check(nec_decoder_t *msg_builder) {
    end_nec_params_t *params = generate_end_nec_params(msg_builder, false);
    if (params != NULL) {
        msg_builder->spared_until = time_us_64() + AUTO_END_AFTER_US - 5000; // A 5 ms margin
        add_alarm_in_us(AUTO_END_AFTER_US, end_nec_signal, params, true);
    }
}


void process_nec_pulses(nec_decoder_t *msg_builder) {
    while (true) {
        if (!mutex_try_enter(&msg_builder->mutex, NULL))
            break; // An end check is already in progress, let it happen. Due to timing, this should rarely occur.

        if (queue_get_level(msg_builder->pulse_source) < 2)
            break;

        pulse_t *pulse1;
        pulse_t *pulse2;

        queue_try_remove(msg_builder->pulse_source, &pulse1);
        queue_try_peek(msg_builder->pulse_source, &pulse2);

        const nec_code_t cur_nec_code = get_next_nec_code(pulse1, pulse2);

        free(pulse1);

        if (msg_builder->is_cancelled) {
            // NO CURRENT BUILD OF MESSAGE IN PROGRESS
            if (cur_nec_code == START) {
                // START signal -> A new message follows
                reset_msg_builder(msg_builder);
            }
        } else
            switch (cur_nec_code) {
                case START: {
                    // CURRENT BUILD OF MESSAGE INTERRUPTED BY NEW TRANSMISSION
                    // Cancel the old signal
                    end_nec_params_t *params = generate_end_nec_params(msg_builder, true);
                    end_nec_signal(0, params);
                    // New one
                    reset_msg_builder(msg_builder);
                    // Set up a timer to check for the end after the first repeat signal was supposed to happen
                    add_or_delay_end_check(msg_builder);
                    break;
                }
                case END: {
                    if (msg_builder->implicit_end) {
                        // END signals should always be anticipated. Abort message if occurs unexpectedly
                        msg_builder->implicit_end = false;
                        msg_builder->step++;
                    } else goto exception;
                    break;
                }
                case ONE: {
                    if (msg_builder->step > 31) goto exception;
                    msg_builder->data |= (UINT32_C(1) << msg_builder->step);
                    // intentional fallthrough
                }
                case ZERO: {
                    if (msg_builder->step > 31) goto exception;
                    if (msg_builder->step == 31) {
                        // For both, ONE and ZERO.
                        // Anticipate next signal (END signal)
                        msg_builder->implicit_end = true;
                        // Send
                        checkout_nec_msg(msg_builder);
                    }
                    msg_builder->step++;
                    break;
                }
                case REPEAT: {
                    if (msg_builder->step > 32 && !(msg_builder->step % 2)) {
                        // From here on only repeats are valid. If step is an even number, it can only be END if valid and the
                        // program never would've reached this point. If even and no END -> Exception
                        // Anticipate next signal (END signal)
                        msg_builder->implicit_end = true;
                        add_or_delay_end_check(msg_builder); // Repeats only delay the end check
                        msg_builder->step++;
                    } else goto exception;
                    break;
                }
                default: {
                    // If this is reached something unexpected has happened, abort
                exception:;
                    bool *safety_guaranteed = malloc(sizeof(bool));
                    *safety_guaranteed = true;
                    end_nec_signal(0, safety_guaranteed);
                }
            }

        mutex_exit(&msg_builder->mutex);
    }
}

bool init_nec_decoder(nec_decoder_t *msg_builder, queue_t *pulse_source, void (*callback_new_cmd)(uint8_t adr, uint8_t cmd),
                 void (*callback_end_last_cmd)()) {
    if (msg_builder == NULL) {
        return false;
    }

    reset_msg_builder(msg_builder);
    mutex_init(&msg_builder->mutex);
    msg_builder->pulse_source = pulse_source;
    msg_builder->is_cancelled = true; // Initially it's basically cancelled since there's nothing being built yet
    msg_builder->callback_new_cmd = callback_new_cmd;
    msg_builder->callback_end_last_cmd = callback_end_last_cmd;

    return true;
}
