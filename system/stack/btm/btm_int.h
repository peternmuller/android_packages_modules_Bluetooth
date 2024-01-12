/******************************************************************************
*
*  Copyright (C) 1999-2012 Broadcom Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  this file contains the main Bluetooth Manager (BTM) internal
 *  definitions.
 *
 ******************************************************************************/
#ifndef BTM_INT_H
#define BTM_INT_H

#include "bt_target.h"
#include "hcidefs.h"

#include "osi/include/alarm.h"
#include "osi/include/fixed_queue.h"
#include "osi/include/list.h"
#include "rfcdefs.h"

#include "btm_api.h"
#include "device/include/esco_parameters.h"

#include "btm_ble_int.h"
#include "btm_int_types.h"
#include "l2cdefs.h"
#include "smp_api.h"

extern void btm_flow_spec_complete(uint8_t status, uint16_t handle,
                            tBT_FLOW_SPEC* p_flow);

#endif