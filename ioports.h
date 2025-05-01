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

static enumerate_pins_ptr on_enumerate_pins;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);

// static void keepalive_rx_packet (modbus_message_t *msg);
// static void keepalive_rx_exception (uint8_t code, void *context);

typedef enum {
    PICOHAL_MSG_KEEPALIVE = 0
} picohal_response_t;

static const modbus_callbacks_t callbacks = {
    .retries = PICOHAL_RETRIES,
    .retry_delay = PICOHAL_RETRY_DELAY,    
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

typedef struct {
    uint16_t index;
    modbus_message_t picohal_packet;
} QueueItem;

modbus_message_t keepalive_msg = {
    .context = PICOHAL_MSG_KEEPALIVE,
    .crc_check = false,
    .adu[0] = PICOHAL_ADDRESS,
    .adu[1] = ModBus_WriteRegister,
    .adu[2] = (uint8_t)(PICOHAL_ADDR_KEEPALIVE >> 8),
    .adu[3] = (uint8_t)(PICOHAL_ADDR_KEEPALIVE & 0xFF),
    .adu[4] = 0,
    .adu[5] = 0x01,
    .tx_length = 8,
    .rx_length = 8
};

typedef struct {
    uint16_t addr;
    xbar_t aux;
} picohal_aux_t;

static uint16_t picohal_d_out[1]; // 16 BIT NUMBER IS GOOD FOR UP TO 16 DIGITAL OUTPUTS
static uint16_t picohal_a_out[2]; // NEEDS TO BE EQUAL TO NUMBER OF ANALOG OUTPUTS? (ALSO NEED TO TEST. . .)
static pin_function_t aux_dout_base = Output_Aux0, aux_aout_base = Output_Analog_Aux0;
static io_ports_data_t analog;
static io_ports_data_t digital;

static picohal_aux_t aux_dout[8] = {};
static picohal_aux_t aux_aout[2] = {};

static bool picohal_send_message_now(modbus_message_t *data);
