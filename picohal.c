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

#include "picohal.h"

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static on_program_completed_ptr on_program_completed;
static coolant_set_state_ptr on_coolant_changed; // For real time loop insertion
static driver_reset_ptr driver_reset;
static user_mcode_ptrs_t user_mcode;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_realtime_report_ptr on_realtime_report;

static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_data_t spindle_data = {0};
static spindle_state_t spindle_state = {0};

static uint16_t retry_counter = 0;

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

//important variables for retries
static coolant_state_t current_coolant_state;
static IPG_state_t current_IPG_state;
static BLC_state_t current_BLC_state;

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

    uint32_t ms = hal.get_elapsed_ticks();

    //can only send if there is something in the queue.
    //if (ms<1000)
    //    return;

    if(peek_message()){
        modbus_send(current_msg_ptr, &callbacks, false);
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

static void picohal_set_IPG_output (IPG_state_t IPG_state)
{       
    //set IPG state in register 0x110
    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x01,
        .adu[3] = 0x10,
        .adu[4] = 0x00,
        .adu[5] = IPG_state.value & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };
    enqueue_message(cmd);
}

static void picohal_set_BLC_output (BLC_state_t BLC_state)
{       
    //set BLC state in register 0x120
    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x01,
        .adu[3] = 0x20,
        .adu[4] = 0x00,
        .adu[5] = BLC_state.value & 0xFF,
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

    modbus_send(&mode_cmd, &callbacks, false);
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
        .adu[2] = 0x02,
        .adu[3] = 0x00,
        .adu[4] = 0x00,
        .adu[5] = (!state.on || rpm == 0.0f) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 8,
        .rx_length = 8
    };

    spindle_state.on = state.on;
    spindle_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, false))
        spindleSetRPM(rpm, true);
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
    // if(sys.cold_start) // is this necessary? Copied from vfd
    //     protocol_enqueue_foreground_task(raise_alarm, NULL);
    // else
    //     system_raise_alarm(Alarm_Spindle);

    uint8_t value = *((uint8_t*)context);
    char buf[16];

    report_message("picohal_rx_exception", Message_Warning);
    sprintf(buf, "CODE: %d", code);
    report_message(buf, Message_Plain);   
    sprintf(buf, "CONT: %d", value);
    report_message(buf, Message_Plain);             
    //if RX exceptions during one of the messages, need to retry?
}

static void picohal_poll (void)
{
    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    //control the rate at which the queue is emptied to avoid filling the modbus queue
    if(ms < last_ms + POLLING_INTERVAL)
        return;    

    //if there is a message try to send it.
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

// check - check if M-code is handled here.
static user_mcode_type_t check (user_mcode_t mcode)
{
    return (mcode == LaserReady_On || mcode == LaserReady_Off ||
            mcode == LaserMains_On || mcode == LaserMains_Off ||
            mcode == Argon_On || mcode == Argon_Off ||
            mcode == Powder1_On || mcode == Powder1_Off
            )
                     ? UserMCode_Normal //  Handled by us. Set to UserMCode_NoValueWords if there are any parameter words (letters) without an accompanying value.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);	// If another handler present then call it or return ignore.
}

// validate - validate parameters
static status_code_t validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case LaserReady_On:
            break;
        case LaserReady_Off:
            break;
        case LaserMains_On:
            break;
        case LaserMains_Off:
            break;
        case Argon_On:
            break;
        case Argon_Off:
            break;
        case Powder1_On:
            break;
        case Powder1_Off:
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    // If not handled by us and another handler present then call it.
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

// execute - execute M-code
static void execute (sys_state_t state, parser_block_t *gc_block)
{
    bool handled = true;

    switch(gc_block->user_mcode) {

        case LaserReady_On:
            current_IPG_state.ready = 1;
            picohal_set_IPG_output(current_IPG_state);
            break;
        case LaserReady_Off:
            current_IPG_state.ready = 0;
            picohal_set_IPG_output(current_IPG_state);
            break;
        case LaserMains_On:
            current_IPG_state.mains = 1;
            picohal_set_IPG_output(current_IPG_state);
            break;
        case LaserMains_Off:
            current_IPG_state.mains = 0;
            picohal_set_IPG_output(current_IPG_state);
            break;
        case Argon_On:
            current_BLC_state.argon = 1;
            picohal_set_BLC_output(current_BLC_state);
            break;
        case Argon_Off:
            current_BLC_state.argon = 0;
            picohal_set_BLC_output(current_BLC_state);
            break;
        case Powder1_On:
            current_BLC_state.powder1 = 1;
            picohal_set_BLC_output(current_BLC_state);
            break;
        case Powder1_Off:
            current_BLC_state.powder1 = 0;
            picohal_set_BLC_output(current_BLC_state);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)          // If not handled by us and another handler present
        user_mcode.execute(state, gc_block);    // then call it.
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt){
        hal.stream.write("[PLUGIN:PICOHAL v0.2]"  ASCII_EOL);
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
    picohal_set_state();
    if (on_state_change)         // Call previous function in the chain.
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
    .type = SpindleType_Stepper, //TODO ADD CUSTOM SPINDLE TYPE
    .ref_id = SPINDLE_PICOHAL,
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

// Set up HAL pointers for handling additional M-codes.
// Call this function on driver setup.
void mcodes_init (void)
{
    // Save away current HAL pointers so that we can use them to keep
    // any chain of M-code handlers intact.
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    // Redirect HAL pointers to our code.
    grbl.user_mcode.check = check;
    grbl.user_mcode.validate = validate;
    grbl.user_mcode.execute = execute;
}

void picohal_init (void)
{
    mcodes_init(); // MCDOES FOR LASER AND POWDER COMMANDS

    // INIT PICOHAL SPINDLE IF CONFIGURED
    #if SPINDLE_ENABLE & (1<<SPINDLE_PICOHAL)

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