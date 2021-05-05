/**
 * Copyright (c) 2009 - 2020, Nordic Semiconductor ASA
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
* @addtogroup nrf_dev_radio_rx_example_main nrf_dev_radio_tx_example_main
* @{
*/

#include "radio_config.h"
#include "nrf_delay.h"

/* These are set to zero as ShockBurst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  (0UL)  /**< Packet length field size in bits. */

/**
 * @brief Function for swapping/mirroring bits in a byte.
 *
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 *
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31]
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}


/**
 * @brief Function for configuring the radio to operate in ShockBurst compatible mode.
 *
 * To configure the application running on nRF24L series devices:
 *
 * @verbatim
 * uint8_t tx_address[5] = { 0xC0, 0x01, 0x23, 0x45, 0x67 };
 * hal_nrf_set_rf_channel(7);
 * hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
 * hal_nrf_set_address(HAL_NRF_TX, tx_address);
 * hal_nrf_set_address(HAL_NRF_PIPE0, tx_address);
 * hal_nrf_open_pipe(0, false);
 * hal_nrf_set_datarate(HAL_NRF_1MBPS);
 * hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
 * hal_nrf_setup_dynamic_payload(0xFF);
 * hal_nrf_enable_dynamic_payload(false);
 * @endverbatim
 *
 * When transmitting packets with hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length),
 * match the length with PACKET_STATIC_LENGTH.
 * hal_nrf_write_tx_payload(payload, PACKET_STATIC_LENGTH);
 *
*/


// DFECTRL1 register values

#define NUMBEROF8US         3  // in unit of 8 us
#define DFEINEXTENSION      1  // 1:crc  0:payload
#define TSWITCHSPACING      2  // 1:4us 2:2us 3: 1us
#define TSAMPLESPACINGREF   6  // 1:4us 2:2us 3: 1us 4:500ns 5:250ns 6:125ns
#define SAMPLETYPE          1  // 0: IQ  1: magPhase
#define TSAMPLESPACING      6  // 1:4us 2:2us 3: 1us 4:500ns 5:250ns 6:125ns

// DFECTRL2 register values

#define TSWITCHOFFSET             0 // 
#define TSAMPLEOFFSET             3 //

// DFEMODE

#define DFEOPMODE_DISABLE         0 //
#define DFEOPMODE_AOD             2 //
#define DFEOPMODE_AOA             3 //

void radio_configure_direction_finding_manual(void) {

    // enable direction finding in AoA mode
    NRF_RADIO->DFEMODE = (uint32_t)DFEOPMODE_AOA;

    NRF_RADIO->CTEINLINECONF     = (uint32_t)0;

    NRF_RADIO->DFECTRL1          = (uint32_t)(NUMBEROF8US        << 0)  | 
                                      (uint32_t)(DFEINEXTENSION     << 7)  |
                                      (uint32_t)(TSWITCHSPACING     << 8)  |
                                      (uint32_t)(TSAMPLESPACINGREF  << 12) |
                                      (uint32_t)(SAMPLETYPE         << 15) |
                                      (uint32_t)(TSAMPLESPACING     << 16);

    NRF_RADIO->DFECTRL2          = (uint32_t)(TSWITCHOFFSET      << 0)  |
                                      (uint32_t)(TSAMPLEOFFSET      << 0);
  

   // NRF_RADIO->DFEPACKET.MAXCNT  = MAX_IQSAMPLES;
   // NRF_RADIO->DFEPACKET.PTR     = (uint32_t)(&radio_vars.df_samples[0]);
}

#define CTEINLINECTRLEN     1 // 1: enabled 0: disabled
#define CTEINFOINS1         1 // 1: data channel PDU  0: advertising channel PDU
#define CTEERRORHANDLING    0 // 1: sampling and antenna switch when crc is not OK, 0: no sampling and antenna ...
#define CTETIMEVALIDRANGE   0 // 0: 20, 1: 31, 2: 63 (in uint of 8 us)
// sample spacing in switching period: 
#define CTEINLINERXMODE1US  2 // 1 4us, 2 2us, 3 1us, 4 0.5us, 5 0.25us, 6 0.125us (used when tswitchingspace is set to 2us)
#define CTEINLINERXMODE2US  2 // 1 4us, 2 2us, 3 1us, 4 0.5us, 5 0.25us, 6 0.125us (used when tswitchingspace is set to 4us)
#define S0CONF              0x20
#define S0MASK              0x20

void radio_configure_direction_finding_inline(void) {

    // enable direction finding in AoA mode
    NRF_RADIO->DFEMODE           = (uint32_t)DFEOPMODE_AOA;

    NRF_RADIO->CTEINLINECONF     = (uint32_t)(CTEINLINECTRLEN    << 0) |
                                      (uint32_t)(CTEINFOINS1        << 3) |
                                      (uint32_t)(CTEERRORHANDLING   << 4) |
                                      (uint32_t)(CTETIMEVALIDRANGE  << 6) |
                                      (uint32_t)(CTEINLINERXMODE1US << 10)|
                                      (uint32_t)(CTEINLINERXMODE2US << 13)|
                                      (uint32_t)(S0CONF             << 16)|
                                      (uint32_t)(S0MASK             << 24);

    NRF_RADIO->DFECTRL1          = (uint32_t)(NUMBEROF8US        << 0) | 
                                      (uint32_t)(DFEINEXTENSION     << 7) |
                                      (uint32_t)(TSWITCHSPACING     << 8) |
                                      (uint32_t)(TSAMPLESPACINGREF  << 12)|
                                      (uint32_t)(SAMPLETYPE         << 15)|
                                      (uint32_t)(TSAMPLESPACING     << 16);

    NRF_RADIO->DFECTRL2          = (uint32_t)(TSWITCHOFFSET      << 0) |
                                      (uint32_t)(TSAMPLEOFFSET      << 0);
  

  //  NRF_RADIO->DFEPACKET.MAXCNT  = MAX_IQSAMPLES;
  //  NRF_RADIO->DFEPACKET.PTR     = (uint32_t)(&radio_vars.df_samples[0]);
}



#define DD_MAX_PAYLOAD_LENGTH (31+6)
#define CRC_POLYNOMIAL_INIT_SETTINGS  ((0x5B << 0) | (0x06 << 8) | (0x00 << 16))
uint8_t access_address[4] = {0xD6, 0xBE, 0x89, 0x8E};
uint8_t seed[3] = {0x55, 0x55, 0x55};

/**@brief The default SHORTS configuration. */
#define DEFAULT_RADIO_SHORTS                                             \
(                                                                        \
    (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | \
    (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos)   \
)

void radio_configure()
{
#if 1
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;

    NRF_RADIO->SHORTS = DEFAULT_RADIO_SHORTS;
    
    NRF_RADIO->PCNF0 =   (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)
                       | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)
                       | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk);

    NRF_RADIO->PCNF1 =   (((RADIO_PCNF1_ENDIAN_Little)        << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk)
                       | (((3UL)                              << RADIO_PCNF1_BALEN_Pos)  & RADIO_PCNF1_BALEN_Msk)
                       | (((0UL)                              << RADIO_PCNF1_STATLEN_Pos)& RADIO_PCNF1_STATLEN_Msk)
                       | ((((uint32_t)DD_MAX_PAYLOAD_LENGTH)  << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk)
                       | ((RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);
    
    /* The CRC polynomial is fixed, and is set here. */
    /* The CRC initial value may change, and is set by */
    /* higher level modules as needed. */
    NRF_RADIO->CRCPOLY = (uint32_t)CRC_POLYNOMIAL_INIT_SETTINGS;
    NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                      | (((RADIO_CRCCNF_LEN_Three)      << RADIO_CRCCNF_LEN_Pos)       & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO->RXADDRESSES  = ( (RADIO_RXADDRESSES_ADDR0_Enabled) << RADIO_RXADDRESSES_ADDR0_Pos);

    NRF_RADIO->MODE    = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;

    NRF_RADIO->TIFS = 150;

    NRF_RADIO->PREFIX0 = access_address[3];
    NRF_RADIO->BASE0   = ( (((uint32_t)access_address[2]) << 24) 
                         | (((uint32_t)access_address[1]) << 16)
                         | (((uint32_t)access_address[0]) << 8) );

    NRF_RADIO->CRCINIT = ((uint32_t)seed[0]) | ((uint32_t)seed[1])<<8 | ((uint32_t)seed[2])<<16;

    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);

// Alternate radio configuration
#else
 uint8_t i;

    // clear variables
    memset(&radio_vars,0,sizeof(radio_vars_t));

    // set radio configuration parameters
    NRF_RADIO_NS->TXPOWER     = (uint32_t)RADIO_TXPOWER;

    // configure packet
    NRF_RADIO_NS->PCNF0       = 
        (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | 
        (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |
        (((8UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk);

    NRF_RADIO_NS->PCNF1       = 
        (((RADIO_PCNF1_ENDIAN_Little)    << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)  |
        (((3UL)                          << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)   |
        (((0UL)                          << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
        ((((uint32_t)MAX_PACKET_SIZE)    << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  |
        ((RADIO_PCNF1_WHITEEN_Enabled    << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);

        
    NRF_RADIO_NS->CRCPOLY     = RADIO_CRCPOLY_24BIT;
    NRF_RADIO_NS->CRCCNF      = 
        (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk) |
        (((RADIO_CRCCNF_LEN_Three)     << RADIO_CRCCNF_LEN_Pos)      & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO_NS->CRCINIT     = RADIO_CRCINIT_24BIT;

    NRF_RADIO_NS->TXADDRESS   = 0;
    NRF_RADIO_NS->RXADDRESSES = 1;

    NRF_RADIO_NS->MODE        = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;
    NRF_RADIO_NS->TIFS        = INTERFRAM_SPACING;
    NRF_RADIO_NS->PREFIX0     = ((BLE_ACCESS_ADDR & 0xff000000) >> 24);
    NRF_RADIO_NS->BASE0       = ((BLE_ACCESS_ADDR & 0x00ffffff) << 8 );

    NRF_RADIO_NS->PACKETPTR   = (uint32_t)(radio_vars.payload);
    NRF_RADIO_NS->SHORTS      = RADIO_SHORTS_PHYEND_DISABLE_Enabled << RADIO_SHORTS_PHYEND_DISABLE_Pos;

    // set priority and disable interrupt in NVIC
    NVIC->IPR[((uint32_t)RADIO_IRQn)]     = 
        (uint8_t)(
            (RADIO_PRIORITY << (8 - __NVIC_PRIO_BITS)) & (uint32_t)0xff
        );
    NVIC->ICER[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);

    // enable address and payload interrupts 
    NRF_RADIO_NS->INTENSET   = 
        RADIO_INTENSET_ADDRESS_Set    << RADIO_INTENSET_ADDRESS_Pos |
        RADIO_INTENSET_END_Set        << RADIO_INTENSET_END_Pos     |
        RADIO_INTENSET_PHYEND_Set     << RADIO_INTENSET_PHYEND_Pos  |
        RADIO_INTENSET_CTEPRESENT_Set << RADIO_INTENSET_CTEPRESENT_Pos;
    
    NVIC->ICPR[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);
    NVIC->ISER[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);

    radio_vars.state        = RADIOSTATE_STOPPED;

    radio_vars.array_to_use = 1;
#endif

}

/**
 * @}
 */
