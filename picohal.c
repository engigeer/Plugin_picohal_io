/*

  picohal.c

  Part of grblHAL picohal plugin

  Copyright (c) 2022-2025 Terje Io
  Copyright (c) 2023 Andrew Marles
  Copyright (c) 2024-2025 Mitchell Grams

  picoHAL design is copyright (c) 2023 Expatria Technologies Inc.

  GrblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  GrblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "picohal.h"

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static on_program_completed_ptr on_program_completed;
static coolant_set_state_ptr on_coolant_changed; // For real time loop insertion
static driver_reset_ptr driver_reset;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_realtime_report_ptr on_realtime_report;

static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_data_t spindle_data = {0};
static spindle_state_t spindle_state = {0};

//static uint16_t retry_counter = 0;

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = PICOHAL_RETRIES,
    .retry_delay = PICOHAL_RETRY_DELAY,    
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

//static uint16_t current_BLC_flowrate = (10 | (10 << 8)); //Initialize both powder flow rates to 1.0RPM MOVE TO PICOHAL FW

static coolant_state_t current_coolant_state;
static sys_state_t current_state; 

typedef struct {
    uint16_t index;
    modbus_message_t picohal_packet;
} QueueItem;

QueueItem message_queue[QUEUE_SIZE];
int front = 0;
int rear = -1;
int item_count = 0;
modbus_message_t current_message;
modbus_message_t * current_msg_ptr = &current_message;
uint16_t current_index;

static bool enqueue_message(modbus_message_t data) {
    static uint16_t message_index;
    if (item_count == QUEUE_SIZE) {
        report_message("Warning: PicoHAL queue is full.", Message_Warning);
        return 0;
    }
    rear = (rear + 1) % QUEUE_SIZE;
    message_queue[rear].picohal_packet = data;
    message_queue[rear].index = message_index;
    message_queue[rear].picohal_packet.context = &message_queue[rear].index;
    message_index++;
    item_count++;
        return 1;
}

static bool dequeue_message() {
    if (item_count == 0) {
        //report_message("Error: queue is empty", Message_Info);
        return 0;
    }
    current_message = (message_queue[front].picohal_packet);
    front = (front + 1) % QUEUE_SIZE;
    item_count--;
    return 1;
}

static bool peek_message() {
    if (item_count == 0) {
        return 0;
    }
    current_message = (message_queue[front].picohal_packet);
    //sprintf(buf, "peek_context f: %p",*picohal_packet->context);
    //sprintf(buf, "peek_context: %d",19535); 

    return 1;
}

static void picohal_send (){

    //uint32_t ms = hal.get_elapsed_ticks();

    //can only send if there is something in the queue.
    //if (ms<1000)
    //    return;

    if(peek_message()){
        modbus_send(current_msg_ptr, &callbacks, true);
    }
}

static void picohal_rx_packet (modbus_message_t *msg)
{
    //check the context/index and pop it off the queue if it matches.
    // sprintf(buf, "recv_context:%d current_context: %d",*((uint16_t*)msg->context), *((uint16_t*)current_msg_ptr->context));
    // report_message(buf, Message_Plain);
    if(*((uint16_t*)msg->context) == *((uint16_t*)current_msg_ptr->context)){
        dequeue_message();
    }
    //else it should stay on the queue to be re-transmitted.
    
}

static void picohal_set_state (sys_state_t state)
{   
    uint16_t data;
    uint16_t alarm_code;

    data = ffs(state);

    if(state & (STATE_ESTOP|STATE_ALARM)) {
        //char *alarm;

        data = SystemState_Alarm; // requires latest core

        modbus_message_t cmd = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x00,
            .adu[3] = 0x01, //status register
            .adu[4] = data >> 8,
            .adu[5] = data & 0xFF,
            .tx_length = 8,
            .rx_length = 8
        };
        enqueue_message(cmd);

        //if in alarm state, write the alarm code to the alarm register.
        if (data == STATE_ALARM){

            alarm_code = (uint16_t) sys.alarm;

            modbus_message_t code_cmd = {
                .context = NULL,
                .crc_check = false,
                .adu[0] = PICOHAL_ADDRESS,
                .adu[1] = ModBus_WriteRegister,
                .adu[2] = 0x00,
                .adu[3] = 0x02, //alarm code register.
                .adu[4] = alarm_code >> 8,
                .adu[5] = alarm_code & 0xFF,
                .tx_length = 8,
                .rx_length = 8
            };
            enqueue_message(code_cmd); 
        };
    }
}

static void picohal_set_coolant ()
{       
    //set coolant state in register 0x100
    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x01,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = current_coolant_state.value & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };
    enqueue_message(cmd);
}

static void picohal_create_event (picohal_events event){

    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x00,
        .adu[3] = 0x05,
        .adu[4] = event >> 8,
        .adu[5] = event & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };
    enqueue_message(cmd);
}
static void spindleSetRPM (float rpm)
{
    uint16_t rpm_value = (uint16_t)rpm; // convert float to integer

    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x02,
        .adu[3] = 0x01,
        .adu[4] = (uint8_t)(rpm_value >> 8), // High byte
        .adu[5] = (uint8_t)(rpm_value & 0xFF), // Low byte
        .tx_length = 8,
        .rx_length = 8
    };

    enqueue_message(cmd);
}

static void spindleSetSpeed (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    spindleSetRPM(rpm);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x02,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = (!state.on || rpm == 0.0f) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 8,
        .rx_length = 8
    };

    spindle_state.on = state.on;
    spindle_state.ccw = state.ccw;

    if(enqueue_message(cmd))
        spindleSetRPM(rpm);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle_state;
}

static void raise_alarm (void *data)
{
    system_raise_alarm(Alarm_Spindle);
}

static void picohal_rx_exception (uint8_t code, void *context)
{
    if(sys.cold_start) // is this necessary? Copied from vfd
        protocol_enqueue_foreground_task(raise_alarm, NULL);
    else
        system_raise_alarm(Alarm_Spindle);

    // uint8_t value = *((uint8_t*)context);
    // char buf[16];

    // report_message("picohal_rx_exception", Message_Warning);
    // sprintf(buf, "CODE: %d", code);
    // report_message(buf, Message_Plain);   
    // sprintf(buf, "CONT: %d", value);
    // report_message(buf, Message_Plain);             
    //if RX exceptions during one of the messages, need to retry?
}

static void picohal_poll (void)
{
    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    //control the rate at which the queue is emptied to avoid filling the modbus queue
    if(ms < last_ms + POLLING_INTERVAL)
        return;    

    // stop sending messages if spindle alarm?
    // if there is a message try to send it.
    if(item_count){
        picohal_send();
        last_ms = ms;
    }
}

static void picohal_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);
    picohal_poll();
}

static void picohal_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);
    picohal_poll();
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt){
        hal.stream.write("[PLUGIN:PICOHAL v0.3]"  ASCII_EOL);
    }
}

static void onCoolantChanged (coolant_state_t state){

    current_coolant_state = state;
    picohal_set_coolant();

    if (on_coolant_changed)         // Call previous function in the chain.
        on_coolant_changed(state);    
}

static void onStateChanged (sys_state_t state)
{
    current_state = state;
    picohal_set_state(state);
    if (on_state_change)           // Call previous function in the chain.
        on_state_change(state);    
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    //write the program flow value to the event register.
    //enqueue(PROGRAM_COMPLETED);
    picohal_create_event(PROGRAM_COMPLETED);
    
    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void picohal_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(on_realtime_report)
        on_realtime_report(stream_write, report);

    //picohal_get_update() should this be above or here?
}

/* static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
} */

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {

        spindle_hal = spindle;
        spindle_data.rpm_programmed = -1.0f;

        //modbus_set_silence(NULL);

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

// DRIVER RESET
static void driverReset (void)
{
    picohal_set_state(current_state);
    driver_reset();
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup();
    //return true;
}

static const spindle_ptrs_t spindle = {
    .type = SpindleType_Stepper, //TODO ADD CUSTOM SPINDLE TYPE
    .ref_id = SPINDLE_MY_SPINDLE,
    .cap = {
        .variable = On,
        .at_speed = Off,
        .direction = Off,
        .cmd_controlled = On,
        .laser = On //TODO: TEST LASER CAPABILITY
    },
    .config = spindleConfig,
    .set_state = spindleSetState,
    .get_state = spindleGetState,
    .update_rpm = spindleSetSpeed
};

/*** ioports stuff  ***/

typedef struct {
    uint16_t addr;
    xbar_t aux;
} picohal_aux_t;

static uint16_t a_out[1];
static uint16_t d_out[1];
static pin_function_t aux_dout_base = Output_Aux0, aux_aout_base = Output_Analog_Aux0;
static io_ports_data_t analog;
static io_ports_data_t digital;

picohal_aux_t aux_dout[] = {
    {
        .addr = 0x2101,
        .aux = {
            .pin = 0,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    },
    {
        .addr = 0x2101,
        .aux = {
            .pin = 1,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    },
    {
        .addr = 0x2101,
        .aux = {
            .pin = 2,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    }
};

picohal_aux_t aux_aout[] = {
    {
        .addr = 0x2100,
        .aux = {
            .port = &a_out[0],
            .group = PinGroup_AuxOutputAnalog,
            .cap = {
                .output = On,
                .analog = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    }
};

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        uint16_t *val = (uint16_t *)aux_aout[port].aux.port; //get current value?

        *val = (uint16_t)value;

        modbus_message_t cmd = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(aux_aout[port].addr >> 8),
            .adu[3] = (uint8_t)(aux_aout[port].addr & 0xFF),
            .adu[4] = (uint8_t)(*val >> 8),
            .adu[5] = (uint8_t)*val,
            .tx_length = 8,
            .rx_length = 8
        };

        enqueue_message(cmd);
    }

    return true;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports) {

        uint16_t *val = (uint16_t *)aux_dout[port].aux.port; //get current value?

        if(on)
            *val |= (1 << aux_dout[port].aux.pin);
        else
            *val &= ~(1 << aux_dout[port].aux.pin);

        modbus_message_t cmd = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(aux_dout[port].addr >> 8),
            .adu[3] = (uint8_t)(aux_dout[port].addr & 0xFF),
            .adu[4] = 0,
            .adu[5] = (uint8_t)*val,
            .tx_length = 8,
            .rx_length = 8
        };

        enqueue_message(cmd);
    }
}

static float analog_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < analog.out.n_ports)
        value = (float)*(uint16_t *)output->port;

    return value;
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)(!!(*(uint16_t *)output->port & (1 << output->pin)));

    return value;
}

static xbar_t *a_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < analog.in.n_ports) {
//...
        info = &pin;
    }

    if(dir == Port_Output && port < analog.out.n_ports) {
        memcpy(&pin, &aux_aout[port].aux, sizeof(xbar_t));
        pin.get_value = analog_out_state;
        info = &pin;
    }

    return info;
}

static xbar_t *d_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < digital.in.n_ports) {
//...
        info = &pin;
    }

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_dout[port].aux, sizeof(xbar_t));
        pin.get_value = digital_out_state;
//        pin.set_value = digital_out_state;
        info = &pin;
    }

    return info;
}

static void a_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
//    if(dir == Port_Input && port < analog.in.n_ports)
//        aux_ain[port].description = description;

    if(dir == Port_Output && port < analog.out.n_ports)
        aux_aout[port].aux.description = description;
}

static void d_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
//    if(dir == Port_Input && port < digital.in.n_ports)
//        aux_din[port].description = description;

    if(dir == Port_Output && port < digital.out.n_ports)
        aux_dout[port].aux.description = description;
}

static enumerate_pins_ptr on_enumerate_pins;

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

    for(idx = 0; idx < sizeof(aux_dout) / sizeof(picohal_aux_t); idx ++) {

        memcpy(&pin, &aux_dout[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoHAL:";

        pin_info(&pin, data);
    };

    for(idx = 0; idx < sizeof(aux_aout) / sizeof(picohal_aux_t); idx ++) {

        memcpy(&pin, &aux_aout[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoHAL:";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *data)
{
    if(pin->group == PinGroup_AuxOutput)
        aux_dout_base = max(aux_dout_base, pin->function + 1);
    else if(pin->group == PinGroup_AuxOutputAnalog)
        aux_aout_base = max(aux_aout_base, pin->function + 1);
}

/**********************/

void my_plugin_init (void)
{

/*** ioports stuff  ***/

    uint_fast8_t idx;

    hal.enumerate_pins(false, get_aux_max, NULL);

    digital.out.n_ports = sizeof(aux_dout) / sizeof(picohal_aux_t);

    for(idx = 0; idx < digital.out.n_ports; idx ++) {
        aux_dout[idx].aux.id = idx;
        aux_dout[idx].aux.function = aux_dout_base + idx;
    }

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = d_get_pin_info,
        .set_pin_description = d_set_pin_description,
    };

    ioports_add_digital(&dports);

    analog.out.n_ports = sizeof(aux_aout) / sizeof(picohal_aux_t);

    for(idx = 0; idx < analog.out.n_ports; idx ++) {
        aux_aout[idx].aux.id = idx;
        aux_aout[idx].aux.function = aux_aout_base + idx;
    }

    io_analog_t aports = {
        .ports = &analog,
        .analog_out = analog_out,
        .get_pin_info = a_get_pin_info,
        .set_pin_description = a_set_pin_description,
    };

    ioports_add_analog(&aports);

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;

/**********************/

    // INIT PICOHAL SPINDLE IF CONFIGURED
    #if SPINDLE_ENABLE & (1<<SPINDLE_MY_SPINDLE)

        if((spindle_id = spindle_register(&spindle, "PicoHAL")) != -1) {
            // spindleSetState(NULL, spindle_state, 0.0f);

            on_spindle_selected = grbl.on_spindle_selected;
            grbl.on_spindle_selected = onSpindleSelected;
        } else {
            protocol_enqueue_foreground_task(report_warning, "PicoHAL spindle failed to initialize!");
        }
    #endif

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_state_change = grbl.on_state_change;             // Subscribe to the state changed event by saving away the original
    grbl.on_state_change = onStateChanged;              // function pointer and adding ours to the chain.

    on_coolant_changed = hal.coolant.set_state;         //subscribe to coolant events
    hal.coolant.set_state = onCoolantChanged;

    on_realtime_report = grbl.on_realtime_report;       //keepalive (IMPLEMENTATION NEEDED)
    grbl.on_realtime_report = picohal_realtime_report;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = picohal_poll_realtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = picohal_poll_delay;         

    on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events (lightshow on complete?)
    grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here

    driver_reset = hal.driver_reset;                    // Subscribe to driver reset event
    hal.driver_reset = driverReset;
}