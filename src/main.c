/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "ir/cmd_listener.h"

#include "ir/gpio_edge_receiver.h"
#include "ir/nec_decoder.h"
#include "ir/pulse_detector.h"

/// \tag::hello_uart[]

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define IR_INPUT_PIN 12 // ?

int main() {
    // IR setup
    queue_t edge_queue;
    signal_state_t ir_idle_state;
    init_gpio_edge_receiver(IR_INPUT_PIN, IDLE_MODE_HIGH, &edge_queue, &ir_idle_state);

    queue_t pulse_queue;
    init_pulse_detector(&pulse_queue);

    nec_decoder_t nec_decoder;
    init_nec_decoder(&nec_decoder, &pulse_queue, new_cmd, end_cmd);

    while (true) {
        // Happy decoding!
        process_edges(&edge_queue, &pulse_queue, ir_idle_state);
        process_nec_pulses(&nec_decoder);
    }



    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // Send out a character without any conversions
    uart_putc_raw(UART_ID, 'A');

    // Send out a character but do CR/LF conversions
    uart_putc(UART_ID, 'B');

    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");
    return 0;
}

/// \end::hello_uart[]
