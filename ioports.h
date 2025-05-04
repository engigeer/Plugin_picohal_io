/*
  ioports.h - connect to picoHAL (RP2040) ioexpander via modbus RTU

  Part of grblHAL

  Copyright (c) 2023 Expatria Technologies Inc.
  Copyright (c) 2025 Terje Io
  Copyright (c) 2025 Mitchell Grams

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl/hal.h"
#include "grbl/modbus.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/task.h"

#ifndef PICOHAL_ADDRESS
#define PICOHAL_ADDRESS               10
#endif

#define PICOHAL_RETRIES               5
#define PICOHAL_RETRY_DELAY           250
#define PICOHAL_KEEPALIVE_INTERVAL    1000

#ifndef PICOHAL_ADDR_KEEPALIVE
#define PICOHAL_ADDR_KEEPALIVE   0x0100
#endif

#ifndef PICOHAL_ADDR_DOUT
#define PICOHAL_ADDR_DOUT        0x0110
#endif

#ifndef PICOHAL_ADDR_AOUT
#define PICOHAL_ADDR_AOUT        0x0120
#endif

typedef enum {
    PICOHAL_MSG_KEEPALIVE = 0
} picohal_response_t;

typedef struct {
    uint16_t index;
    modbus_message_t picohal_packet;
} QueueItem;

typedef struct {
    uint16_t addr;
    xbar_t aux;
} picohal_aux_t;

bool picohal_send_message_now(modbus_message_t *data);
void picospindle_init(void);
