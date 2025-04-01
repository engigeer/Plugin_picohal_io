/*

  picohal.h

  Part of grblHAL
  grblHAL is
  Copyright (c) 2022-2023 Terje Io

  picoHAL design and plugin code are copyright (c) 2023 Expatria Technologies Inc.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#include "../grbl/modbus.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#include "grbl/modbus.h"
#include "driver.h"
#endif

#define PICOHAL_ADDRESS 10
#define QUEUE_SIZE 8

#define RETRY_DELAY         250
#define POLLING_INTERVAL    100
#define PICOHAL_RETRIES     5

typedef enum {
    TOOLCHANGE_ACK = 0,
    PROBE_START = 1,
    PROBE_COMPLETED = 2,
    PROBE_FIXTURE = 3,
    PROGRAM_COMPLETED = 30,
    HOMING_COMPLETED = 31,
    INVALID_EVENT = 255,
} picohal_events;

// typedef enum {
//     SPINDLE_Idle = 0,
//     SPINDLE_SetSpeed,
//     SPINDLE_GetSpeed,
//     SPINDLE_GetStatus,
//     SPINDLE_SetStatus,
// } picohal_response_t;

// void picohal_init (void);

/**/
