/**
 * Copyright (c) 2014-2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup nrf_radio_test_example_main main.c
 * @{
 * @ingroup nrf_radio_test_example
 * @brief Radio Test Example application main file.
 *
 * This file contains the source code for a sample application that uses the NRF_RADIO and is controlled through the serial port.
 *
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "bsp.h"
#include "nrf.h"
#include "radio_cmd.h"
#include "app_uart.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "radio_config.h"
#if defined(NRF21540_DRIVER_ENABLE) && (NRF21540_DRIVER_ENABLE == 1)
#include "nrf21540.h"
#endif

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39
#define TIMER_PERIOD    (0xffff>>1)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define SAMPLE_MAXCNT       (0xA0)
#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)

#define ENABLE_DF  1
#define IOSEA  1

const static uint8_t ble_device_addr[6] = { 
    0x04, 0x04, 0x04, 0x04, 0x04, 0xc4
};

//Quuppa Tag_Id
const static uint8_t Quuppa_tag_ID[6] = { 
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00
};

const static uint8_t Quuppa_DF_data[14] = { 
    0x67, 0xF7, 0xDB, 0x34, 0xC4, 0x03, 0x8E, 0x5C, 0x0B, 0xAA, 0x97, 0x30, 0x56, 0xE6
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                uint8_t         flags;
                app_state_t     state;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
} app_vars_t;

app_vars_t app_vars;

NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);


/**@brief Function for starting a command line interface that works on the UART transport layer.
 */
static void cli_start(void)
{
    ret_code_t ret;

    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for configuring UART for CLI.
 */
static void cli_init(void)
{
    ret_code_t ret;

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;

    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret                 = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);

    APP_ERROR_CHECK(err_code);
}


/** @brief Function for configuring all peripherals used in this example.
 */
static void clock_init(void)
{
    // Start 64 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

static uint32_t                   packet;                    /**< Packet to transmit. */
/** @brief Function for the main application entry.
 */

//=========================== private =========================================

void assemble_IOSEA_packet(void) {

    uint8_t i;
    i=0;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x46;               // BLE ADV_SCAN_IND 
    app_vars.packet[i++]  = 0x06;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5
}

void assemble_QUUPPA_packet(void) {

    uint8_t i;
    i=0;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x46;               // BLE ADV_SCAN_IND 
    app_vars.packet[i++]  = 0x25;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

    app_vars.packet[i++]  = 0x02; // AD data length
    app_vars.packet[i++]  = 0x01; // AD:Type Flag
    app_vars.packet[i++]  = 0x00; // Flags Data
    app_vars.packet[i++]  = 0x1B; // Advertisement payload/data length
    app_vars.packet[i++]  = 0xFF; // Type - Manufacturer specific data
    app_vars.packet[i++]  = 0xC7; // Company ID
    app_vars.packet[i++]  = 0x00; // Company ID
    app_vars.packet[i++]  = 0x01; // Quuppa packet ID

//---- 
// Device type, header and tag ID should be unique
    app_vars.packet[i++]  = 0x20; // Device type identifier of a Quuppa Tag 
    app_vars.packet[i++]  = 0x1E; // Header - Carries information of Tag’s TX rate -> (7-14hz), TX power -> 6 dbm and ID type -> Developer (10) 

    app_vars.packet[i++]  = Quuppa_tag_ID[0]; // Quuppa Tag ID byte 0
    app_vars.packet[i++]  = Quuppa_tag_ID[1]; // Quuppa Tag ID byte 1
    app_vars.packet[i++]  = Quuppa_tag_ID[2]; // Quuppa Tag ID byte 2
    app_vars.packet[i++]  = Quuppa_tag_ID[3]; // Quuppa Tag ID byte 3
    app_vars.packet[i++]  = Quuppa_tag_ID[4]; // Quuppa Tag ID byte 4
    app_vars.packet[i++]  = Quuppa_tag_ID[5]; // Quuppa Tag ID byte 5
    app_vars.packet[i++]  = 0x94; // Checksum (CRC-8, polynomial 0x97) for byte 21-28
//----
    app_vars.packet[i++]  = Quuppa_DF_data[0]; // Quuppa DF_Field byte 0
    app_vars.packet[i++]  = Quuppa_DF_data[1]; // Quuppa DF_Field byte 1
    app_vars.packet[i++]  = Quuppa_DF_data[2]; // Quuppa DF_Field byte 2
    app_vars.packet[i++]  = Quuppa_DF_data[3]; // Quuppa DF_Field byte 3
    app_vars.packet[i++]  = Quuppa_DF_data[4]; // Quuppa DF_Field byte 4
    app_vars.packet[i++]  = Quuppa_DF_data[5]; // Quuppa DF_Field byte 5
    app_vars.packet[i++]  = Quuppa_DF_data[6]; // Quuppa DF_Field byte 6
    app_vars.packet[i++]  = Quuppa_DF_data[7]; // Quuppa DF_Field byte 7
    app_vars.packet[i++]  = Quuppa_DF_data[8]; // Quuppa DF_Field byte 8
    app_vars.packet[i++]  = Quuppa_DF_data[9]; // Quuppa DF_Field byte 9
    app_vars.packet[i++]  = Quuppa_DF_data[10]; // Quuppa DF_Field byte 10
    app_vars.packet[i++]  = Quuppa_DF_data[11]; // Quuppa DF_Field byte 11
    app_vars.packet[i++]  = Quuppa_DF_data[12]; // Quuppa DF_Field byte 11
    app_vars.packet[i++]  = Quuppa_DF_data[13]; // Quuppa DF_Field byte 11

}

uint32_t ble_channel_to_frequency(uint8_t channel) {

    uint32_t frequency;
    
    if (channel<=10) {

        frequency = 4+2*channel;
    } else {
        if (channel >=11 && channel <=36) {
            
            frequency = 28+2*(channel-11);
        } else {
            switch(channel) {
                case 37:
                    frequency = 2;
                break;
                case 38:
                    frequency = 26;
                break;
                case 39:
                    frequency = 80;
                break;
                default:
                    // something goes wrong
                    frequency = 2;

            }
        }
    }

    return frequency;
}

typedef enum {
   FREQ_TX                        = 0x01,
   FREQ_RX                        = 0x02,
} radio_freq_t;


void radio_setFrequency(uint8_t channel, radio_freq_t tx_or_rx) {

    NRF_RADIO->FREQUENCY     = ble_channel_to_frequency(channel);
    NRF_RADIO->DATAWHITEIV   = channel; 
}

void send_packet(uint8_t channel)
{
    radio_setFrequency(channel, FREQ_TX);
    // send the packet:
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN   = 1;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;

    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    NRF_LOG_INFO("The packet was sent");           

    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
}


int main(void)
{
    uint32_t err_code;
    uint32_t delay;

    log_init();

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    clock_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

#if UI
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
#endif
    // Set radio configuration parameters
    radio_configure();

    // enable the DF inline configuration
    radio_configure_direction_finding_inline();

#if IOSEA
    // assemble the IOSEA DF packet
    assemble_IOSEA_packet(); 
#else
    // assemble the Quuppa DF packet
    assemble_QUUPPA_packet(); 
#endif

    // set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)(&app_vars.packet[0]);

    NRF_LOG_INFO("Radio transmitter example started.");
    APP_ERROR_CHECK(err_code);

while(1)
{   
    // Send the packet
    send_packet(37);
    send_packet(38);
    NRF_LOG_FLUSH();

#if PACKET_DELAY
    for (delay=0;delay<0xfffff;delay++);
#endif

}

}

/** @} */
