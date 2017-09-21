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

#ifndef NRF_MESH_CONFIG_PROV_H__
#define NRF_MESH_CONFIG_PROV_H__

/**
 * @defgroup NRF_MESH_CONFIG_PROV Provisioning configuration
 * @ingroup MESH_API_GROUP_PROV
 * @{
 */

/** Maximum number of UUIDs in local UUID cache. */
#ifndef NRF_MESH_PROV_MESH_SERVER_UUID_LIST_SIZE
#define NRF_MESH_PROV_MESH_SERVER_UUID_LIST_SIZE 3
#endif

/** Enable verbose debugging information from the PB-MESH client and server. */
#ifndef PB_REMOTE_DEBUG_VERBOSE
#define PB_REMOTE_DEBUG_VERBOSE 0
#endif

/** Enables debug mode for the common provisioning code and the provisionee and provisioner modules. */
#ifndef PROV_DEBUG_MODE
#define PROV_DEBUG_MODE 0
#endif

/**
 * @defgroup NRF_MESH_CONFIG_PROV_BEARER Provisioning Bearer Configuration
 * @{
 */

/** Unprovisioned beacon default advertisement interval. */
#ifndef NRF_MESH_UNPROV_BEACON_INTERVAL_MS
#define NRF_MESH_UNPROV_BEACON_INTERVAL_MS 2000
#endif

/** @} end of NRF_MESH_CONFIG_PROV_BEARER */

/** @} end of NRF_MESH_CONFIG_PROV */

#endif
