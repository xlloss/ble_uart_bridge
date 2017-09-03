/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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

/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "ble_nus_c.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1024                                         /**< UART RX buffer size. */


#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE      GATT_MTU_SIZE_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

//#define CENTRAL_LINK_COUNT        8                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define CENTRAL_LINK_COUNT        7                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT     0                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1

#define APP_TIMER_PRESCALER       0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS      (2 + BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE   2                                          /**< Size of timer operation queues. */

#define SCAN_INTERVAL             0x00A0                                     /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                     /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                     /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                          /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE               2                                          /**< Size of a UUID, in bytes. */

#define LEDBUTTON_LED             BSP_BOARD_LED_2                            /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON_PIN      BSP_BUTTON_0                               /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define TIMER_BLE_TX_INTERVAL      APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
APP_TIMER_DEF(m_ble_tx_timer_id);

static const char m_target_periph_name[] = "Nordic_UART";
#define PACKET_LEN 31
#define PACKET_HEAD 3
#define PACKET_DATA_LEN (PACKET_LEN - PACKET_HEAD)


#define BUFFER_SZ 1024
#define TEST_VERSION "Multilink BLE UART Example v.20\r\n"
#define TX_ORDER_NUM 1024

#define NEW_MAC_ADDRESS_TEST

unsigned char data_buf_cnt[CENTRAL_LINK_COUNT];
int buf_index = 0;
int buf_data_index = 0;
int start_cnt = 0;


struct uart_tx_fifo {
  app_fifo_t tx_fifo_handle;
  uint8_t *tx_buf;      /**< Pointer to the TX buffer. */
  uint32_t tx_buf_size; /**< Size of the RX buffer. */
};

struct tx_packet_order {
  uint8_t fifo_number;
  uint32_t packet_size;
};

struct uart_tx_fifo tx_fifo[CENTRAL_LINK_COUNT];
struct tx_packet_order packet_order;
uint8_t alloc_tx_buf[CENTRAL_LINK_COUNT][BUFFER_SZ] = {0};
static uint32_t packet_data_cnt[CENTRAL_LINK_COUNT] = {0};
uint8_t packet_data_tx_order[TX_ORDER_NUM] = {0};
int tx_order_index_buy = 0;

uint32_t reset_ble_rx_start = 0;
uint32_t reset_ble_rx_stop = 0;
uint32_t reset_ble_cont = 0;

app_fifo_t packet_data_tx_order_hd;

/** @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,

    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif

    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist  = 0,
        .adv_dir_report = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

static ble_nus_c_t              m_ble_nus_c[TOTAL_LINK_COUNT];                    /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery[TOTAL_LINK_COUNT];             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */

#ifdef NEW_MAC_ADDRESS_TEST
//static const ble_gap_addr_t m_target_periph_addr =
//{
//    /* Possible values for addr_type:
//       BLE_GAP_ADDR_TYPE_PUBLIC,
//       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
//       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
//       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
//    .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
//    .addr      = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55}
//};

struct customer_mac_addr {
  const ble_gap_addr_t m_target_periph_addr;
  uint8_t connected;
  int conn_handle;
};


struct customer_mac_addr cust_mac_addr[CENTRAL_LINK_COUNT] = {
  /* 0 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55},
    },
    .connected = 0,
    .conn_handle = -1,
  },
  
  
  /* 1 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x11, 0x11, 0x22, 0x33, 0x44, 0x55},
    },
    .connected = 0,
    .conn_handle = -1,
  },
  
  /* 2 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x22, 0x11, 0x22, 0x33, 0x44, 0x55},     
    },
    .connected = 0,
    .conn_handle = -1,
  },

  /* 3 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x33, 0x11, 0x22, 0x33, 0x44, 0x55},     
    },
    .connected = 0,
    .conn_handle = -1,
  },

  /* 4 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x44, 0x11, 0x22, 0x33, 0x44, 0x55},     
    },
    .connected = 0,
    .conn_handle = -1,
  },

  /* 5 iteam */  
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x55, 0x11, 0x22, 0x33, 0x44, 0x55},
    },
    .connected = 0,
    .conn_handle = -1,
  },
  
  /* 6 iteam */
  {
    {
      .addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
      .addr      = {0x66, 0x11, 0x22, 0x33, 0x44, 0x55},     
    },
    .connected = 0,
    .conn_handle = -1,
  },
  
};

#endif

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

#ifndef NEW_MAC_ADDRESS_TEST
/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */

static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

#endif

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    NRF_LOG_INFO("Start scanning for device name %s.\r\n", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);
}


/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

//#define DEBUG

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;
    static uint8_t packet_head[CENTRAL_LINK_COUNT][3], ble_hd;
    static uint8_t packet_end[CENTRAL_LINK_COUNT][3];
    uint32_t conn_handle;
    uint8_t find_connhand;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("The device has the Nordic UART Service\r\n");
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
            ble_hd = p_ble_nus_c->conn_handle;
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++) {
              NRF_LOG_INFO ("p_ble_nus_evt->p_data[%d] 0x%c\r\n",i, p_ble_nus_evt->p_data[i]);
              err_code = app_fifo_put(&tx_fifo[ble_hd].tx_fifo_handle, p_ble_nus_evt->p_data[i]);
              if (err_code != NRF_SUCCESS) {
                NRF_LOG_INFO ("tx_fifo[%d].tx_fifo_handle app_fifo_put fail: ble_hd %d\r\n", ble_hd, ble_hd);
              }
            }

            packet_data_cnt[ble_hd] = packet_data_cnt[ble_hd] + 1;

            err_code = app_fifo_put(&packet_data_tx_order_hd, ble_hd);
            if (err_code != NRF_SUCCESS) {
              NRF_LOG_INFO ("packet_data_tx_order_hd app_fifo_put fail: ble_hd %d\r\n", ble_hd);
            }
            NRF_LOG_INFO ("ble_hd %d\r\n", ble_hd);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            for (find_connhand = 0; find_connhand < CENTRAL_LINK_COUNT; find_connhand++) {
              conn_handle = cust_mac_addr[find_connhand].conn_handle;
              if (conn_handle == p_ble_nus_c->conn_handle) {
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[0]);
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[1]);
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[2]);
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[3]);
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[4]);
                NRF_LOG_INFO ("MAC ADDR 0x%x\r\n", cust_mac_addr[find_connhand].m_target_periph_addr.addr[5]);
                cust_mac_addr[find_connhand].connected = 0;
                break;
              }
            }
            scan_start();
            break;
    }
}


/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    /* static uint8_t data_array[BLE_NUS_MAX_DATA_LEN]; */
    /* static uint8_t index = 0; */
    /* uint32_t       err_code; */

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
          /*
						NRF_LOG_INFO("\r\nAPP_UART_DATA_READY\r\n");
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
          */
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_INFO ("!!!!!APP_UART_FIFO_ERROR\r\n");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


#ifdef NEW_MAC_ADDRESS_TEST
/**@brief Function for searching a given addr in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * addr in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   p_addr   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_peer_addr(const ble_gap_evt_adv_report_t *p_adv_report, const ble_gap_addr_t * p_addr)
{
    if (p_addr->addr_type == p_adv_report->peer_addr.addr_type)
    {
        if (memcmp(p_addr->addr, p_adv_report->peer_addr.addr, sizeof(p_adv_report->peer_addr.addr)) == 0)
        {
            return true;
        }
    }
    return false;
}
#endif


/**@snippet [UART Initialization] */

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    uint32_t      err_code;
    uint8_t       find_mac_index;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    bool          do_connect = false;
    ble_gap_addr_t target_periph_addr;
    // For readibility.
    const ble_gap_evt_t * const p_gap_evt    = &p_ble_evt->evt.gap_evt;
    const ble_gap_addr_t  * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
    const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

#ifndef NEW_MAC_ADDRESS_TEST

    //Initialize advertisement report for parsing
    adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.size   = p_gap_evt->params.adv_report.dlen;


    //search for advertising names
    bool found_name = false;
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                &adv_data,
                                &dev_name);
    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit
            return;
        }
        else
        {
            found_name = true;
        }
    }
    else
    {
        found_name = true;
    }
    
    if (found_name)
    {
        if (strlen(m_target_periph_name) != 0)
        {
            if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.size) == 0)
            {
                do_connect = true;
            }
        }
    }

#else
    
    for (find_mac_index = 0; find_mac_index < CENTRAL_LINK_COUNT; find_mac_index++) {
      if (cust_mac_addr[find_mac_index].connected == 0) {
        target_periph_addr = cust_mac_addr[find_mac_index].m_target_periph_addr;
        //break;
      }
    
    
      NRF_LOG_INFO("find_mac_index %d\r\n", find_mac_index);

      //if (find_mac_index < CENTRAL_LINK_COUNT) {
        if (find_peer_addr(&p_gap_evt->params.adv_report, &target_periph_addr)) {
            NRF_LOG_INFO("Address match send connect_request.\r\n");
            do_connect = true;
            break;
        }
      //}
    }

#endif
    if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d\r\n", err_code);
        } else {
          NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                   p_adv_report->peer_addr.addr[0],
                   p_adv_report->peer_addr.addr[1],
                   p_adv_report->peer_addr.addr[2],
                   p_adv_report->peer_addr.addr[3],
                   p_adv_report->peer_addr.addr[4],
                   p_adv_report->peer_addr.addr[5]);
          cust_mac_addr[find_mac_index].conn_handle = p_gap_evt->conn_handle;
          cust_mac_addr[find_mac_index].connected = 1;
        }

    }

}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 *          parses scanning reports, initiating a connection attempt to peripherals when a
 *          target UUID is found, and manages connection parameter update requests. Additionally,
 *          it updates the status of LEDs used to report central applications activity.
 *
 * @note Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *       should be dispatched to the target application before invoking this function.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    // For readability.
    const ble_gap_evt_t * const p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {

        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.\r\n",
                         p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < TOTAL_LINK_COUNT);
			
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle],
                        p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_ble_db_discovery[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            scan_start();

        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)\r\n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            // Start scanning
            scan_start();

        } break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.\r\n");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 * been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_conn_state_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);

    // Make sure taht an invalid connection handle are not passed since
    // our array of modules is bound to TOTAL_LINK_COUNT.
    if (conn_handle < TOTAL_LINK_COUNT)
    {
        ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
        //ble_lbs_c_on_ble_evt(&m_ble_lbs_c[conn_handle], p_ble_evt);
		ble_nus_c_on_ble_evt(&m_ble_nus_c[conn_handle],p_ble_evt);
    }
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;
    uint8_t m_ble_nus_c_count;
	
    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;
	
    for (m_ble_nus_c_count = 0; m_ble_nus_c_count < TOTAL_LINK_COUNT; m_ble_nus_c_count++) {
      err_code = ble_nus_c_init(&m_ble_nus_c[m_ble_nus_c_count], &nus_c_init_t);
      APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Use the max config: 8 central, 0 periph, 10 VS UUID
    ble_enable_params.common_enable_params.vs_uuid_count = 10;

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //NRF_LOG_INFO("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!\r\n",
	NRF_LOG_INFO("call to  instance %d and link 0x%x!\r\n",
                    p_evt->conn_handle,
                    p_evt->conn_handle);
    
	
	ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}



/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

///int my_ble_hd = 0;
uint8_t my_ble_hd = 0;
static void ble_process_buf_handler(void * p_context)
{
  int eat_i = 0, get_out = 0, get_in = 0, i = 0;
  unsigned char uart_tx_buffer, ble_hd;
  uint32_t err_code = NRF_SUCCESS;
  UNUSED_PARAMETER(p_context);
  
//  NRF_LOG_INFO ("ble_process_buf_handler\r\n");
  app_timer_stop(m_ble_tx_timer_id);

  if (my_ble_hd >= 7)
    my_ble_hd = 0;

//  err_code = app_fifo_get(&packet_data_tx_order_hd, &my_ble_hd);
//  if (err_code != NRF_SUCCESS) {
//    app_timer_start(m_ble_tx_timer_id, TIMER_BLE_TX_INTERVAL, NULL);
//    return;
//  }
  
  NRF_LOG_INFO ("ble_process_buf_handler my_ble_hd %d %d\r\n", my_ble_hd, packet_data_cnt[my_ble_hd]);
  if (packet_data_cnt[my_ble_hd] < 3) {
    app_timer_start(m_ble_tx_timer_id, TIMER_BLE_TX_INTERVAL, NULL);
    my_ble_hd++;
    return;
  }

  packet_data_cnt[my_ble_hd] = 0;
  
  while (1) {
    NRF_LOG_INFO ("while loop\r\n");
    err_code = app_fifo_get(&tx_fifo[my_ble_hd].tx_fifo_handle, &uart_tx_buffer);
    if (err_code != NRF_SUCCESS) {
      break;
    }

    if (uart_tx_buffer == '$')
      get_in++;

    if (uart_tx_buffer == '#')
      get_out++;
    
    if (get_in > 0 )
      while (app_uart_put(uart_tx_buffer) != NRF_SUCCESS);
    
    if (get_out >= 3) {
//      packet_data_cnt[my_ble_hd] = 0;
      break;
    }
  };
  
  my_ble_hd++;
//exit:
  nrf_delay_ms(10);
  app_timer_start(m_ble_tx_timer_id, TIMER_BLE_TX_INTERVAL, NULL);
}


static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_ble_tx_timer_id, TIMER_BLE_TX_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/* For BLE uart RX buffer */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_ble_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                ble_process_buf_handler);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t err_code;
    int i = 0;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO(TEST_VERSION);
    
    timers_init();

		uart_init();
    ble_stack_init();

    db_discovery_init();
    nus_c_init();
    
    NRF_LOG_INFO("Multi Link Uart Bridge\r\n");

    for (i = 0; i < CENTRAL_LINK_COUNT; i++) {
      tx_fifo[i].tx_buf = &alloc_tx_buf[i][0];
      tx_fifo[i].tx_buf_size = BUFFER_SZ;
      NRF_LOG_INFO("tx_fifo[%d].tx_buf_size %d\r\n", i, tx_fifo[i].tx_buf_size);
      err_code = app_fifo_init(&tx_fifo[i].tx_fifo_handle, tx_fifo[i].tx_buf, tx_fifo[i].tx_buf_size);
      APP_ERROR_CHECK(err_code);
      packet_data_cnt[i] = 0;
    }
    
    //for (i = 0; i < TX_ORDER_NUM; i++) {
    //  packet_data_tx_order[i] = 0xFF;
    //}
    
    app_fifo_init(&packet_data_tx_order_hd, packet_data_tx_order, TX_ORDER_NUM);
    
    // Start scanning for peripherals and initiate connection to devices which
    // advertise.
    scan_start();
    application_timers_start();

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            // Wait for BLE events.
            power_manage();
        }
    }
}
