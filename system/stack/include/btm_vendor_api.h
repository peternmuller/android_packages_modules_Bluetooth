/******************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ******************************************************************************/

#pragma once

#include "stack/include/btm_vendor_types.h"

/*******************************************************************************
 *
 * Function         BTM_GetRemoteQLLFeatures
 *
 * Description      This function is called to get remote QLL features
 *
 * Returns          true if feature value is available
 *
 *
 ******************************************************************************/
extern bool BTM_GetRemoteQLLFeatures(uint16_t handle, uint8_t* features);

/*******************************************************************************
 *
 * Function         BTM_IsQHSPhySupported
 *
 * Description      This function is called to determine if QHS phy can be used
 *
 * Parameter        connection handle
 *
 * Returns          bool true if qhs phy can be used
 *
 ******************************************************************************/
extern bool BTM_IsQHSPhySupported(uint16_t handle);

/*******************************************************************************
 *
 * Function         BTM_BleIsQHSPhySupported
 *
 * Description      This function is called to determine if QHS phy can be used
 *
 * Parameter        bda: BD address of the remote device
 *
 * Returns          bool true if qhs phy can be used, false otherwise
 *
 ******************************************************************************/
extern bool BTM_BleIsQHSPhySupported(const RawAddress& bda);
/*******************************************************************************
 *
 * Function         BTM_GetHostAddOnFeatures
 *
 * Description      BTM_GetHostAddOnFeatures
 *
 *
 * Returns          host add on features array
 *
 ******************************************************************************/
bt_device_host_add_on_features_t* BTM_GetHostAddOnFeatures(
    uint8_t* host_add_on_features_len);

/*******************************************************************************
 *
 * Function         BTM_GetSocAddOnFeatures
 *
 * Description      BTM_GetSocAddOnFeatures
 *
 *
 * Returns          soc add on features array
 *
 ******************************************************************************/
bt_device_soc_add_on_features_t* BTM_GetSocAddOnFeatures(
    uint8_t* soc_add_on_features_len);

/*******************************************************************************
 *
 * Function         BTM_ReadVendorAddOnFeatures
 *
 * Description      BTM_ReadVendorAddOnFeatures
 *
 * Parameters:      None
 *
 ******************************************************************************/
void BTM_ReadVendorAddOnFeatures();