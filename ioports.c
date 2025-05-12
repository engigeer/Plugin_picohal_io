/*
  ioports.c - connect to picoHAL (RP2040) ioexpander via modbus RTU

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

#include "ioports.h"
#include "stdio.h"
#include "math.h"

#if PICOHAL_IO_ENABLE == 1

static enumerate_pins_ptr on_enumerate_pins;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = PICOHAL_RETRIES,
    .retry_delay = PICOHAL_RETRY_DELAY,    
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

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

static uint16_t picohal_d_out[1]; // 16 BIT NUMBER IS GOOD FOR UP TO 16 DIGITAL OUTPUTS
static uint16_t picohal_d_in[1]; // 16 BIT NUMBER IS GOOD FOR UP TO 16 DIGITAL OUTPUTS
static uint16_t picohal_a_out[2]; // NEEDS TO BE EQUAL TO NUMBER OF ANALOG OUTPUTS? (ALSO NEED TO TEST. . .)
static pin_function_t aux_dout_base = Output_Aux0, aux_din_base = Input_Aux0, aux_aout_base = Output_Analog_Aux0;
static io_ports_data_t analog;
static io_ports_data_t digital;

static picohal_aux_t aux_dout[12] = {};
static picohal_aux_t aux_din[4] = {};
static picohal_aux_t aux_aout[2] = {};

static bool picohal_is_online = false;

static void picohal_rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((picohal_response_t)msg->context) {
            
            case PICOHAL_MSG_KEEPALIVE:

                picohal_is_online = true;
                break;

            default:
                break;
        }
    }
}

static void raise_alarm (void *data)
{
    system_raise_alarm(Alarm_AbortCycle);
    report_message("PicoHAL communication error.", Message_Warning);
}

static void picohal_rx_exception (uint8_t code, void *context)
{
    sys_state_t state = state_get();

    if(sys.cold_start){
        task_add_immediate(raise_alarm, NULL);
    }
    else{
        if(picohal_is_online){ //only raise alarm if comms issue is new??
            system_raise_alarm(Alarm_AbortCycle);
            report_message("PicoHAL communication error.", Message_Warning);
            picohal_is_online = false;
        }
    }
}

static void picohal_send_keepalive (void *data){

    modbus_send(&keepalive_msg, &callbacks, false); // should this be non-blocking 

    task_add_delayed(picohal_send_keepalive, NULL, PICOHAL_KEEPALIVE_INTERVAL);
}

bool picohal_send_message_now (modbus_message_t *data){
    bool okay;
    
    if(!(okay = modbus_send(data, &callbacks, true))) {
        if(state_get() != STATE_IDLE)
            system_raise_alarm(Alarm_AbortCycle);
        report_message("PicoHAL communication error.", Message_Warning);
        // will this cause a double error report? (since an exception also occurs
        // ! no because keepalive will have already failed and set picohal_is_online = false)
        // hmm this may not yet be as desired . . .
    }
    return okay;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        uint16_t *val = (uint16_t *)aux_aout[port].aux.port; //get current value

        *val = (uint16_t)value;

        modbus_message_t data = {
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

        picohal_send_message_now(&data);
    }

    return true;
}

static void digital_out_ll (xbar_t *output, float value)
{
    bool on = value !=0.0f;

    if(aux_dout[output->id].aux.mode.inverted)
        on = !on;

    uint16_t *val = (uint16_t *)aux_dout[output->id].aux.port; //get current value

    if(on)
        *val |= (1 << aux_dout[output->id].aux.pin);
    else
        *val &= ~(1 << aux_dout[output->id].aux.pin);

    modbus_message_t data = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = (uint8_t)(aux_dout[output->id].addr >> 8),
        .adu[3] = (uint8_t)(aux_dout[output->id].addr & 0xFF),
        .adu[4] = 0,
        .adu[5] = (uint8_t)*val,
        .tx_length = 8,
        .rx_length = 8
    };

    picohal_send_message_now(&data);
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports)
        digital_out_ll(&aux_dout[port].aux, (float)on);
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports) {

        if(config->inverted != aux_dout[output->id].aux.mode.inverted) {
            aux_dout[output->id].aux.mode.inverted = config->inverted;
            digital_out(output->pin, (((*(uint16_t *)output->port) >> output->pin) & 1) ^ config->inverted);
        }
        // Open drain not supported

        if(persistent)
            ioport_save_output_settings(output, config);
    }

    return output->id < digital.out.n_ports;
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

static bool digital_in (uint8_t port)
{
    bool on = false;

    //if(port < digital.in.n_ports)

    return on;
}

static bool digital_in_cfg (xbar_t *input, gpio_in_config_t *config, bool persistent)
{
    if(input->id < digital.in.n_ports) {

        if(config->inverted != aux_din[input->id].aux.mode.inverted) {
            aux_din[input->id].aux.mode.inverted = config->inverted;
            //digital_out(output->pin, (((*(uint16_t *)output->port) >> output->pin) & 1) ^ config->inverted);
        }

        if(persistent)
            ioport_save_input_settings(input, config);
    }

    return input->id < digital.in.n_ports;
}

static float digital_in_state (xbar_t *input)
{
    float value = -1.0f;

    if(input->id < digital.in.n_ports)
        value = (float)(!!(*(uint16_t *)input->port & (1 << input->pin)));

    return value;
}

inline static __attribute__((always_inline)) int32_t get_input (const xbar_t *input, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return digital_in(input->pin);

    int32_t value = -1;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t irq_mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & irq_mode) {

            //event_bits &= ~(1UL << input->pin);
            //pinEnableIRQ(input, irq_mode);

            // do {
            //     if(event_bits & (1UL << input->pin)) {
            //         value = digital_in(input->pin);
            //         break;
            //     }
            //     if(delay) {
            //         protocol_execute_realtime();
            //         hal.delay_ms(50, NULL); //DOES THIS NEED TO INCREASE? DONT FORGET TO CHANGE CALC OF DELAY IF SO
            //     } else
            //         break;
            // } while(--delay && !sys.abort);

            //pinEnableIRQ(input, input->mode.irq_mode);    // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if(digital_in(input->pin) == wait_for) {
                value = digital_in(input->pin);
                break;
            }
            if(delay) {
                protocol_execute_realtime();
                hal.delay_ms(50, NULL); //DOES THIS NEED TO INCREASE? DONT FORGET TO CHANGE CALC OF DELAY IF SO
            } else
                break;
        } while(--delay && !sys.abort);
    }

    return value;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(port < digital.in.n_ports)
        value = get_input(&aux_din[port].aux, wait_mode, timeout);

    return value;
}

// void ioports_event (input_signal_t *input)
// {
//     event_bits |= (1UL << input->pin);

//     if(input->interrupt_callback)
//         input->interrupt_callback(input->user_port, digital_in(input->pin) ^ input->mode.inverted);
// }

static bool set_function (xbar_t *port, pin_function_t function)
{
    bool ok = false;

    if(port->mode.input & port->id < digital.in.n_ports)
        aux_din[port->id].aux.function = function;
        ok = true;

    if(port->mode.output & port->id < digital.out.n_ports)
        aux_dout[port->id].aux.function = function;
        ok = true;

    return ok;
}

static xbar_t *a_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < analog.in.n_ports) {
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
        memcpy(&pin, &aux_din[port].aux, sizeof(xbar_t));
        pin.get_value = digital_in_state;
        pin.config = digital_in_cfg;
        info = &pin;
    }

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_dout[port].aux, sizeof(xbar_t));
        pin.get_value = digital_out_state;
        pin.set_function = set_function;
        pin.config = digital_out_cfg;
        pin.set_value = digital_out_ll;
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
   if(dir == Port_Input && port < digital.in.n_ports)
       aux_din[port].aux.description = description;

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
            pin.port = "PicoHAL_DO";

        pin_info(&pin, data);
    };

    for(idx = 0; idx < sizeof(aux_aout) / sizeof(picohal_aux_t); idx ++) {

        memcpy(&pin, &aux_aout[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoHAL_AO";

        pin_info(&pin, data);
    };

    for(idx = 0; idx < sizeof(aux_din) / sizeof(picohal_aux_t); idx ++) {

        memcpy(&pin, &aux_din[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoHAL_DI";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *data)
{
    if(pin->group == PinGroup_AuxOutput)
        aux_dout_base = max(aux_dout_base, pin->function + 1);
    else if(pin->group == PinGroup_AuxInput)
        aux_din_base = max(aux_din_base, pin->function + 1);        
    else if(pin->group == PinGroup_AuxOutputAnalog)
        aux_aout_base = max(aux_aout_base, pin->function + 1);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("PicoHAL IOExpansion", picohal_is_online ? "0.03" : "0.03 : (not connected)");
}

static void OnReset (void)
{
    //picohal_is_online = true; // necessary to ensure that keepalive will raise new error if comms still down. . .
    // TODO: SET ALL OUTPUTS TO NULL?
    driver_reset();
}

static void complete_setup (void *data)
{

    if(modbus_isup().rtu) {

        picospindle_init();
        
        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = OnReset;

        task_add_delayed(picohal_send_keepalive, NULL, PICOHAL_KEEPALIVE_INTERVAL);
    } else {
        task_add_immediate(report_warning, "PICOHAL failed to initialize");
    }

}

void picohal_io_init (void) {

    uint_fast8_t idx;

    hal.enumerate_pins(false, get_aux_max, NULL);

    digital.in.n_ports = sizeof(aux_din) / sizeof(picohal_aux_t);
    digital.out.n_ports = sizeof(aux_dout) / sizeof(picohal_aux_t);

    for(idx = 0; idx < digital.out.n_ports; idx ++) {
        aux_dout[idx].addr = PICOHAL_ADDR_DOUT;
        aux_dout[idx].aux.id = idx;
        aux_dout[idx].aux.pin = idx;
        aux_dout[idx].aux.port = &picohal_d_out;
        aux_dout[idx].aux.function = aux_dout_base + idx;
        aux_dout[idx].aux.group = PinGroup_AuxOutput;
        aux_dout[idx].aux.cap.output = On;
        aux_dout[idx].aux.cap.invert = On;
        aux_dout[idx].aux.cap.external = On;
        aux_dout[idx].aux.cap.async = Off; //TODO: make configurable via xbar_config call
        aux_dout[idx].aux.cap.claimable = On;
        aux_dout[idx].aux.mode.output = On;
    }

    for(idx = 0; idx < digital.in.n_ports; idx ++) {
        aux_din[idx].addr = PICOHAL_ADDR_DOUT; //MAYBE WANT TO CHANGE THIS . . .
        aux_din[idx].aux.id = idx;
        aux_din[idx].aux.pin = idx;
        aux_din[idx].aux.port = &picohal_d_in;
        aux_din[idx].aux.function = aux_din_base + idx;
        aux_din[idx].aux.group = PinGroup_AuxInput;
        aux_din[idx].aux.cap.input = On;
        aux_din[idx].aux.cap.invert = On;
        aux_din[idx].aux.cap.external = On;
        aux_din[idx].aux.cap.irq_mode = IRQ_Mode_None;
        aux_din[idx].aux.mode.input = On;
    }

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = d_get_pin_info,
        .wait_on_input = wait_on_input,
        .set_pin_description = d_set_pin_description,
    };

    ioports_add_digital(&dports);

    analog.out.n_ports = sizeof(aux_aout) / sizeof(picohal_aux_t);

    for(idx = 0; idx < analog.out.n_ports; idx ++) {
        aux_aout[idx].addr = PICOHAL_ADDR_AOUT + idx;
        aux_aout[idx].aux.id = idx; 
        aux_aout[idx].aux.pin = idx;
        aux_aout[idx].aux.port = &picohal_a_out[idx];
        aux_aout[idx].aux.function = aux_aout_base + idx;
        aux_aout[idx].aux.group = PinGroup_AuxOutputAnalog;
        aux_aout[idx].aux.cap.output = On;
        aux_aout[idx].aux.cap.analog = On;
        aux_aout[idx].aux.cap.resolution = Resolution_16bit;
        aux_aout[idx].aux.cap.external = On;
        aux_aout[idx].aux.cap.async = On;
        aux_aout[idx].aux.cap.claimable = On;
        aux_aout[idx].aux.mode.output = On;
        aux_aout[idx].aux.mode.analog = On;
    }

    io_analog_t aports = {
        .ports = &analog,
        .analog_out = analog_out,
        .get_pin_info = a_get_pin_info,
        .set_pin_description = a_set_pin_description,
    };

    ioports_add_analog(&aports);

    // delay final setup until startup is complete
    task_run_on_startup(complete_setup, NULL);

}

#endif // PICOHAL_IO_ENABLE