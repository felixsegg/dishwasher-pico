//
// Created by Felix on 07.10.2025.
//

#ifndef DISHWASHER_PICO_NEC_DECODER_H
#define DISHWASHER_PICO_NEC_DECODER_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/util/queue.h"
#include "pico/mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NEC signal code types.
 */
typedef enum {
    INVALID,  ///< Invalid or unrecognized signal pattern.
    ZERO,     ///< Logical 0 from the NEC signal.
    ONE,      ///< Logical 1 from the NEC signal.
    START,    ///< Start signal from the NEC protocol.
    REPEAT,   ///< Repeat signal in NEC protocol.
    END       ///< End signal (after message or repeat transmission).
} nec_code_t;

/**
 * @brief Represents a NEC decoder instance that builds messages from pulse input.
 */
typedef struct {
    uint32_t data;                       ///< The currently built data word.
    uint step;                           ///< Current step in the building process (0–31 = bits, >31 = repeats).
    bool implicit_end;                   ///< Indicates if an END signal was anticipated in previous iteration.
    mutex_t mutex;                       ///< Mutex to synchronize end-of-signal handling.
    uint64_t spared_until;               ///< Timestamp (us) marking the signal as non-cancellable until this moment.
    bool is_cancelled;                   ///< Whether the current message build is cancelled.
    queue_t *pulse_source;               ///< Pointer to the source queue containing pulse_t* elements.

    void (*callback_new_cmd)(uint8_t adr, uint8_t cmd); ///< Called when a new NEC command is completed.
    void (*callback_end_last_cmd)(void);                ///< Called when the previous command officially ends.
} nec_decoder_t;

/**
 * @brief Internal helper structure passed to delayed end-of-signal alarm callbacks.
 */
typedef struct {
    nec_decoder_t *msg_builder; ///< Decoder instance to finalize.
    bool safety_guaranteed;     ///< If true, caller already holds mutex.
} end_nec_params_t;

/**
 * @brief Initializes a NEC decoder instance.
 *
 * @param msg_builder Pointer to the decoder structure to initialize.
 * @param pulse_source Queue that provides pulse_t* input elements.
 * @param callback_new_cmd Callback called when a new valid command is detected.
 * @param callback_end_last_cmd Callback called when the command sequence ends.
 * @return true if initialization succeeded, false otherwise.
 */
bool init_nec_decoder(
    nec_decoder_t *msg_builder,
    queue_t *pulse_source,
    void (*callback_new_cmd)(uint8_t adr, uint8_t cmd),
    void (*callback_end_last_cmd)(void)
);

/**
 * @brief Processes pulses from the decoder’s pulse queue to build NEC messages.
 *
 * This function should be called regularly, e.g., from a main loop or interrupt context,
 * to process new pulses, recognize NEC signal sequences, and trigger callbacks accordingly.
 *
 * @param msg_builder Pointer to the initialized decoder instance.
 */
void process_nec_pulses(nec_decoder_t *msg_builder);

#ifdef __cplusplus
}
#endif

#endif //DISHWASHER_PICO_NEC_DECODER_H