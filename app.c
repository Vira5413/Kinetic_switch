/***************************************************************************//**
 * @file
 * @brief app.c
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided \'as-is\', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 *
 * EVALUATION QUALITY
 * This code has been minimally tested to ensure that it builds with the
 * specified dependency versions and is suitable as a demonstration for
 * evaluation purposes only.
 * This code will be maintained at the sole discretion of Silicon Labs.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include "app_assert.h"
#include "em_emu.h"
#include "efr32mg22_devinfo.h"
#include "sl_gpio.h"

#include "rail.h"
#include "rail_features.h"
#include "rail_ble.h"
#include "rail_types.h"

#include "app_rail_ble.h"
#include "app.h"

//#include "psa/crypto.h"

#include "em_gpio.h"
#include "sl_clock_manager.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

/// Size of the FIFO buffers
#define RAIL_FIFO_SIZE                    256

#define RAIL_NUM_OF_TX                    60
#define TX_INTERVAL_US                    30

#define RAIL_BLE_ADV_CHANNEL              37

static const uint16_t adv_logical_ch[3] = {37, 38, 39};
static const uint16_t adv_physical_ch[3] = { 0, 12, 39 };
static uint8_t adv_ch_idx = 0;
// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

/******************************************************************************
 * Initialization of RAIL TX FIFO
 *****************************************************************************/
static void app_set_rail_tx_fifo(RAIL_Handle_t rail_handle);

/**************************************************************************//**
 * The function initializes the all states, status for application process.
 *****************************************************************************/
static void app_rail_ble_adv_init(RAIL_Handle_t rail_handle);

/*******************************************************************************
 * The API set the rail back to idle state
 ******************************************************************************/
static void set_radio_to_idle_state(RAIL_Handle_t rail_handle);

/**************************************************************************//**
 * Transmits the data packet.
 *
 * @param[in] rail_handle Handle to the RAIL context
 * @param[in] packet      The packet that is desired to send
 * @param[in] rail_handle The size of the packet
 * @return None
 *****************************************************************************/
static void handle_transmit(RAIL_Handle_t rail_handle);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

/// Transmit FIFO
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t tx_fifo[RAIL_FIFO_SIZE];

#define ADV_RAIL_PACKET_LENGTH       39
#define ADV_RAIL_ADVA_OFFSET         2
#define ADV_RAIL_STATUS_OFFSET       15
#define ADV_RAIL_RANDOM_OFFSET       31     // new
#define ADV_RAIL_RANDOM_LEN          8      // new

static const sl_gpio_t sense_pin = { .port = SL_GPIO_PORT_C, .pin = 0 };

//static const sl_gpio_t dbg_pin = { .port = SL_GPIO_PORT_B, .pin = 4 };   // PB04
//#define DBG_HIGH()  sl_gpio_set_pin(&dbg_pin)
//#define DBG_LOW()   sl_gpio_clear_pin(&dbg_pin)

#define DBG_TOGGLE()  GPIO_PinOutToggle(gpioPortB, 4)

static __ALIGNED(RAIL_FIFO_ALIGNMENT)
uint8_t adv_rail_packet[ADV_RAIL_PACKET_LENGTH] =
{
  0x02, // packet type: ADV_NONCONN_­IND, non-connectable
  0x25, // length: 37
  0x2f, 0x45, 0x0a, 0x6f, 0x0d, 0x00, // AdvA: advertiser address
  0x02, // adv data length 2
  0x01, // adv data type 1 (flags)
  0x06, // flags: LE General Discoverable Mode, BR/EDR Not Supported
  0x04, // adv data length 5
  0xff, // adv data type: manufacturer specific
  0xff, 0x02, // manufacturer: Silicon Labs
  0x00,  // STATUS: 0x00 released, 0x01 held   ← new
  0x0a, // adv data length 9
  0x09, // adv data type 9 (complete local name)
  'E', 'H', ' ', 'S', 'w', 'i', 't', 'c', 'h',// "EH Switch"
  0x0B, 0xFF, 0xFF, 0x02, // [27..30] new manufacturer data header
  0x00, 0x00, 0x00, 0x00,                 // [31..34] random ID ← filled from TRNG
  0x00, 0x00, 0x00, 0x00                 // [35..38]
};

static RAIL_ScheduleTxConfig_t shedule_tx_config = {
  .when = TX_INTERVAL_US,
  .mode = RAIL_TIME_DELAY,
  .txDuringRx = RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX
};

static volatile bool tx_requested = false;
static RAIL_Handle_t rail_handle;

/// Notify RAIL Tx, Rx calibration error
static bool rail_error = false;

/// Notify end of packet transmission
static bool rail_packet_sent = false;

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * The function is used for some basic initialization related to the app.
 *****************************************************************************/
//void app_init_pre_clock(void) { DBG_TOGGLE(); }
//void app_init_early(void)     { DBG_TOGGLE(); }

void app_init(void)
{
  //sl_gpio_set_pin_mode(&dbg_pin, SL_GPIO_MODE_PUSH_PULL, 0);   // output, low
  //DBG_HIGH();
  //DBG_TOGGLE();
  rail_ble_init(RAIL_BLE_PROTOCOL_BLE_1MBPS);

  // Get RAIL handle, used later by the application
  rail_handle = rail_ble_get_handle();

  // sets the RAIL TX FIFO
  app_set_rail_tx_fifo(rail_handle);

  //DBG_LOW();
  //DBG_TOGGLE();
  // initializes the ble adv
  app_rail_ble_adv_init(rail_handle);


}

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(void)
{
  if (tx_requested) {
    tx_requested = false;

    // sends the adv packet
    handle_transmit(rail_handle);
  }

  // wait until the packet is sent
  if (rail_packet_sent) {
    rail_packet_sent = false;
    tx_requested = true;
  }
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle,
                           RAIL_Events_t events)
{
  (void) rail_handle;

  // Handle Tx events
  if (events & RAIL_EVENTS_TX_COMPLETION) {
    if ((events & RAIL_EVENT_TX_PACKET_SENT)
        || (events & RAIL_EVENT_TX_SCHEDULED_TX_MISSED)) {
      rail_packet_sent = true;
    } else {
      rail_error = true;
    }
  }
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

//static sl_status_t fill_random(uint8_t *buf, uint32_t len)
//{
//  //DBG_HIGH();                                   // pulse 2: crypto init
//  psa_status_t s = psa_crypto_init();
//  //DBG_LOW();
//  DBG_TOGGLE();
//  if (s != PSA_SUCCESS) {
//    return SL_STATUS_FAIL;
//  }
//
//  //DBG_HIGH();                                   // pulse 3: generate 8 bytes
//  s = psa_generate_random(buf, len);
//  //DBG_LOW();
//  DBG_TOGGLE();
//  return (s == PSA_SUCCESS) ? SL_STATUS_OK : SL_STATUS_FAIL;
//}

/******************************************************************************
 * Initialization of RAIL TX FIFO
 *****************************************************************************/
static void app_set_rail_tx_fifo(RAIL_Handle_t rail_handle)
{
  // RAIL FIFO size allocated by RAIL_SetTxFifo() call
  uint16_t allocated_tx_fifo_size = 0;

  allocated_tx_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo,
                                          0U, RAIL_FIFO_SIZE);
  app_assert(allocated_tx_fifo_size == RAIL_FIFO_SIZE,
             "RAIL_SetTxFifo() failed to allocate a large enough fifo"
             " (%d bytes instead of %d bytes)\n",
             allocated_tx_fifo_size, RAIL_FIFO_SIZE);
}

static void app_rail_ble_adv_init(RAIL_Handle_t rail_handle)
{
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET] = DEVINFO->EUI48L & 0xff;
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET + 1] = (DEVINFO->EUI48L >> 8) & 0xff;
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET + 2] = (DEVINFO->EUI48L >> 16) & 0xff;
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET + 3] = (DEVINFO->EUI48L >> 24) & 0xff;
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET + 4] = DEVINFO->EUI48H & 0xff;
  adv_rail_packet[ADV_RAIL_ADVA_OFFSET + 5] = (DEVINFO->EUI48H >> 8) & 0xff;

//  sl_clock_manager_enable_bus_clock(SL_BUS_CLOCK_HFXO0);
//  // read core bais value.....
//  adv_rail_packet[37] =(HFXO0->XTALCTRL & _HFXO_XTALCTRL_COREBIASANA_MASK) >> _HFXO_XTALCTRL_COREBIASANA_SHIFT;
//  // read status
//  uint8_t flags = 0;
//  if (HFXO0->STATUS & HFXO_STATUS_RDY) {
//    flags |= (1 << 0);     // set bit 0
//  }
//  if (HFXO0->STATUS & HFXO_STATUS_COREBIASOPTRDY) {
//    flags |= (1 << 1);     // set bit 1
//  }
//  if (HFXO0->XTALCTRL & HFXO_XTALCTRL_SKIPCOREBIASOPT) {
//    flags |= (1 << 2);     // set bit 2
//  }
//  adv_rail_packet[38] = flags;


  // Configures us for the first advertising channel (Physical: 0, Logical: 37).
  // The CRC init value and Access Address come from the BLE specification.
  RAIL_BLE_ConfigChannelRadioParams(rail_handle,
                                    0x555555,
                                    0x8E89BED6,
                                    RAIL_BLE_ADV_CHANNEL,
                                    false);

  //fill_random(&adv_rail_packet[ADV_RAIL_RANDOM_OFFSET], ADV_RAIL_RANDOM_LEN);

  bool level = false;
  sl_gpio_set_pin_mode(&sense_pin, SL_GPIO_MODE_INPUT, 0);
  sl_gpio_get_pin_input(&sense_pin, &level);
  sl_gpio_set_pin_mode(&sense_pin, SL_GPIO_MODE_DISABLED, 0);  // input buffer off again
  adv_rail_packet[ADV_RAIL_STATUS_OFFSET] = level ? 0x01 : 0x00;

  tx_requested = true;
}

/*******************************************************************************
 * The API set the rail back to idle state
 ******************************************************************************/
static void set_radio_to_idle_state(RAIL_Handle_t rail_handle)
{
  RAIL_StopTxStream(rail_handle);
  RAIL_Idle(rail_handle, RAIL_IDLE, true);
}
//
//static sl_status_t rail_ble_advertise(RAIL_Handle_t rail_handle,
//                                      uint8_t *packet, uint16_t packet_len)
//{
//  RAIL_Status_t status;
//
//  set_radio_to_idle_state(rail_handle);
//
//  // Add packet to RAIL fifo
//  status = RAIL_WriteTxFifo(rail_handle,
//                            packet,
//                            packet_len,
//                            true);
//
//  // Schedule to send packet
//  status |= RAIL_StartScheduledTx(rail_handle, 0,
//                                  RAIL_TX_OPTIONS_DEFAULT,
//                                  &shedule_tx_config, NULL);
//
//  return status == RAIL_STATUS_NO_ERROR ? SL_STATUS_OK : SL_STATUS_FAIL;
//}

static sl_status_t rail_ble_advertise(RAIL_Handle_t rail_handle,
                                      uint8_t *packet, uint16_t packet_len)
{
  RAIL_Status_t status;

  set_radio_to_idle_state(rail_handle);          // must be idle before reconfiguring

  // 1. Retune: access address, CRC init, and whitening seed for this channel
  status = RAIL_BLE_ConfigChannelRadioParams(rail_handle,
                                             0x555555,
                                             0x8E89BED6,
                                             adv_logical_ch[adv_ch_idx],
                                             false);
  if (status != RAIL_STATUS_NO_ERROR) {
    return SL_STATUS_FAIL;
  }

  // 2. Load the packet
  if (RAIL_WriteTxFifo(rail_handle, packet, packet_len, true) != packet_len) {
    return SL_STATUS_FAIL;
  }

  // 3. Send on the matching physical channel
  status = RAIL_StartScheduledTx(rail_handle,
                                 adv_physical_ch[adv_ch_idx],
                                 RAIL_TX_OPTIONS_DEFAULT,
                                 &shedule_tx_config, NULL);

  // 4. Next time, use the next channel: 0 → 1 → 2 → 0 …
  adv_ch_idx = (adv_ch_idx + 1) % 3;
  if (status == RAIL_STATUS_NO_ERROR) {DBG_TOGGLE();}
  return status == RAIL_STATUS_NO_ERROR ? SL_STATUS_OK : SL_STATUS_FAIL;
}

/*******************************************************************************
 * Use RAIL to transmit a data packet
 ******************************************************************************/
static void handle_transmit(RAIL_Handle_t rail_handle)
{
  rail_ble_advertise(rail_handle,
                     adv_rail_packet, sizeof(adv_rail_packet));
}
