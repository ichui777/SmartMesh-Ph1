/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "nrf.h"
#include "boards.h"
#include "nrf_mesh_sdk.h"
#include "nrf_delay.h"
#include "simple_hal.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "nrf_mesh_node_config.h"

#include "simple_on_off_server.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define LED_PIN_NUMBER (BSP_LED_0)
#define LED_PIN_MASK   (1u << LED_PIN_NUMBER)
#define STATIC_AUTH_DATA {0}

/*****************************************************************************
 * Static data
 *****************************************************************************/

static simple_on_off_server_t m_server;

/* Forward declaration */
static bool generic_get_cb(const simple_on_off_server_t * p_server);
static bool generic_set_cb(const simple_on_off_server_t * p_server, bool value);

/*****************************************************************************
 * Static utility functions
 *****************************************************************************/

static void configuration_setup(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n")
    m_server.get_cb = generic_get_cb;
    m_server.set_cb = generic_set_cb;
    ERROR_CHECK(simple_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
}

static void configuration_complete(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    hal_led_blink_ms(LED_PIN_MASK, 200, 4);
}

/*****************************************************************************
 * Light control
 *****************************************************************************/

static bool generic_get_cb(const simple_on_off_server_t * p_server)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got GET command, return LED_PIN_NUMBER %u\n", hal_led_pin_get(LED_PIN_NUMBER));
    return hal_led_pin_get(LED_PIN_NUMBER);
}

static bool generic_set_cb(const simple_on_off_server_t * p_server, bool value)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET command to %u\n", value);
    hal_led_pin_set(LED_PIN_NUMBER, value);
    return value;
}

/*****************************************************************************
 * Listening to iBeacon, company ID 0x8888
 *****************************************************************************/
static uint8_t ibeacon_prefix[] ={0x02, 0x01, 0x06, 0x1A, 0xFF, 0x88, 0x88, 0x02, 0x15};
#define IBEACON_PREFIX_LEN 9

static bool check_ibeacon_header(uint8_t *payload, uint8_t *header)
{
    int i;

    for(i=0; i<IBEACON_PREFIX_LEN; i++){
      if(payload[i] != header[i])
        return false;
    }

    return true;
}

void publish_state_beacon(simple_on_off_server_t * p_server, uint8_t * data, uint16_t len);
#define BEACON_COUNT 10
#define UUID_LEN 16
#define RSSI_LEN 1
#define BID_LEN 1
#define PAYLOAD_LEN UUID_LEN+RSSI_LEN+BID_LEN
#define UUID_OFFSET 9
#define BID_OFFSET UUID_OFFSET+19
static void rx_callback(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    static int count=0;
    uint8_t payload[PAYLOAD_LEN];

    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
    char msg[128];
    //sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
    sprintf(msg, "RX [%d]: RSSI: %d ADV TYPE: %x ADDR: [%x %x %x %x %x %x]",
            p_rx_data->timestamp,
            p_rx_data->rssi,
            p_rx_data->adv_type,
            p_rx_data->addr.addr[0],
            p_rx_data->addr.addr[1],
            p_rx_data->addr.addr[2],
            p_rx_data->addr.addr[3],
            p_rx_data->addr.addr[4],
            p_rx_data->addr.addr[5]);
    if(check_ibeacon_header(p_rx_data->p_payload, ibeacon_prefix) == true){
    //if((check_ibeacon_header(p_rx_data->p_payload, ibeacon_prefix) == true) && !(count++%BEACON_COUNT)){
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BeaconID: %X\n", p_rx_data->p_payload[BID_OFFSET]);

        payload[0]=p_rx_data->p_payload[BID_OFFSET];
        payload[1]=p_rx_data->rssi;
        memcpy(payload+2, p_rx_data->p_payload + UUID_OFFSET, UUID_LEN);
        publish_state_beacon(&m_server, payload, PAYLOAD_LEN);
    }
    LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
}

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Control Server Demo -----\n");

    hal_leds_init();

    static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
    static nrf_mesh_node_config_params_t config_params;
    config_params.prov_caps.num_elements = ACCESS_ELEMENT_COUNT;
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.p_static_data = static_auth_data;
    config_params.complete_callback = configuration_complete;
    config_params.setup_callback = configuration_setup;

#if defined(S130) || defined(S132) || defined(S140)
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
#else
    config_params.lf_clk_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif

    ERROR_CHECK(nrf_mesh_node_config(&config_params));

    /* Start listening for incoming packets */
    nrf_mesh_rx_cb_set(rx_callback);

    while (true)
    {
        nrf_mesh_process();
    }
}
