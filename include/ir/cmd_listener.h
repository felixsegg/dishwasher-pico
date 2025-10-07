//
// Created by Felix on 06.10.2025.
//

#ifndef DISHWASHER_PICO_CMD_LISTENER_H
#define DISHWASHER_PICO_CMD_LISTENER_H

#include <stdint.h>

void new_cmd(uint8_t adr, uint8_t cmd);

void end_cmd();

#endif //DISHWASHER_PICO_CMD_LISTENER_H