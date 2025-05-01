/*

  picohal.c

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

#if 0

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "ioports.h"

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static on_program_completed_ptr on_program_completed;
static coolant_set_state_ptr on_coolant_changed; // For real time loop insertion
static driver_reset_ptr driver_reset;
static on_realtime_report_ptr on_realtime_report;

static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_data_t spindle_data = {0};
static spindle_state_t spindle_state = {0};

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

static coolant_state_t current_coolant_state;
static sys_state_t current_state; 

static void picohal_rx_packet (modbus_message_t *msg)
{
    
}

static void raise_alarm (void *data)
{
    system_raise_alarm(Alarm_Spindle);
}

static void picohal_rx_exception (uint8_t code, void *context)
{
    // if(sys.cold_start) // is this necessary? Copied from vfd
    //     protocol_enqueue_foreground_task(raise_alarm, NULL);
    // else
    //     system_raise_alarm(Alarm_Spindle);

    // uint8_t value = *((uint8_t*)context);
    // char buf[16];

    // report_message("picohal_rx_exception", Message_Warning);
    // sprintf(buf, "CODE: %d", code);
    // report_message(buf, Message_Plain);   
    // sprintf(buf, "CONT: %d", value);
    // report_message(buf, Message_Plain);             
    //if RX exceptions during one of the messages, need to retry?
}

static void picohal_set_state ()
{   
    uint16_t data;
    uint16_t alarm_code;

        switch (current_state){
        case STATE_ALARM:
            data = 1;
            break;
        case STATE_ESTOP:
            data = 1;
            break;            
        case STATE_CYCLE:
            data = 2;
            break;
        case STATE_HOLD:
            data = 3;
            break;
        case STATE_TOOL_CHANGE:
            data = 4;
            break;
        case STATE_IDLE:
            data = 5;
            break;
        case STATE_HOMING:
            data = 6;
            break;   
        case STATE_JOG:
            data = 7;
            break;                                    
        default :
            data = 254;
            break;                                                        
    }

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
    //enqueue_message(cmd);

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
        //enqueue_message(code_cmd); 
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
    //enqueue_message(cmd);
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
    //enqueue_message(cmd);
}
static void spindleSetRPM (float rpm, bool block)
{
    modbus_message_t mode_cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x02,
        .adu[3] = 0x01,
        .adu[4] = 0x00,
        .adu[5] = (rpm == 0.0f) ? 0x00 : 0x01, //NEED TO CONFIGURE USING RPM
        .tx_length = 8,
        .rx_length = 8
    };

    picohal_send_message_now(&mode_cmd);
}

static void spindleSetSpeed (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    modbus_message_t mode_cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x00,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = (!state.on || rpm == 0.0f) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 8,
        .rx_length = 8
    };

    spindle_state.on = state.on;
    spindle_state.ccw = state.ccw;

    //if(modbus_send(&mode_cmd, &callbacks, false))
    //    spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle_state;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt){
        hal.stream.write("[PLUGIN:PICOHAL v0.3]"  ASCII_EOL);
    }
}

static void onCoolantChanged (coolant_state_t state){

    //current_coolant_state = state;
    //picohal_set_coolant();

    if (on_coolant_changed)         // Call previous function in the chain.
        on_coolant_changed(state);    
}

static void onStateChanged (sys_state_t state)
{
    //current_state = state;
    //picohal_set_state();
    if (on_state_change)         // Call previous function in the chain.
        on_state_change(state);    
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    //write the program flow value to the event register.
    //enqueue(PROGRAM_COMPLETED);
    //picohal_create_event(PROGRAM_COMPLETED);
    
    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void picohal_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(on_realtime_report)
        on_realtime_report(stream_write, report);

    //picohal_get_update() should this be above or here?
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

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
    picohal_set_state();
    driver_reset();
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    //return modbus_isup();
}

static const spindle_ptrs_t spindle = {
    .type = SpindleType_PWM, //TODO ADD CUSTOM SPINDLE TYPE
    .ref_id = SPINDLE_PICOHAL,
    .cap = {
        .variable = On,
        .at_speed = Off,
        .direction = Off,
        .cmd_controlled = On,
        .laser = On
    },
    .config = spindleConfig,
    .set_state = spindleSetState,
    .get_state = spindleGetState,
    .update_rpm = spindleSetSpeed
};


void picohal_init (void)
{
    // mcodes_init(); // CUSTOM PCODES

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

    on_realtime_report = grbl.on_realtime_report;       //keepalive
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

#endif