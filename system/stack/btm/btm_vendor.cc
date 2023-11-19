/******************************************************************************
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ******************************************************************************/

/******************************************************************************
 *
 * This file contains functions that handle BTM Vendor interface functions for
 * the Bluetooth Add ON feature, etc.
 *
 ******************************************************************************/

#include <base/logging.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <include/hardware/bt_av.h>
#include "btconfigstore/bt_configstore.h"
#include "btm_api.h"
#include "osi/include/log.h"
#include "stack/include/bt_types.h"
#include "stack/include/btm_vendor_api.h"
#include "stack/include/btm_vendor_types.h"

#define QHS_TRANSPORT_BREDR 0
#define QHS_TRANSPORT_LE 1
#define QHS_TRANSPORT_LE_ISO 2

/* Disable QHS */
#define QHS_HOST_MODE_HOST_DISABLE 0
/* Enable QHS support */
#define QHS_HOST_MODE_HOST_AWARE 3
/* Disable QHS, QLL and QLMP modes */
#define QHS_HOST_DISABLE_ALL 4

#define QHS_BREDR_MASK 0x01
#define QHS_LE_MASK 0x02
#define QHS_LE_ISO_MASK 0x04

#define QBCE_QLL_MULTI_CONFIG_CIS_PARAMETER_UPDATE_HOST_BIT 58

typedef struct {
  const uint8_t as_array[8];
} bt_event_mask_t;

const bt_event_mask_t QBCE_QLM_AND_QLL_EVENT_MASK = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x4A}};

// BT features related defines
static bt_soc_type_t soc_type = BT_SOC_TYPE_DEFAULT;
static char a2dp_offload_Cap[PROPERTY_VALUE_MAX] = {'\0'};
static bool spilt_a2dp_supported = true;
static bool aac_frame_ctl_enabled = false;
static bool max_power_prop_enabled = false;
static uint8_t max_power_prop_value[3];
static uint8_t scrambling_supported_freqs[MAX_SUPPORTED_SCRAMBLING_FREQ_SIZE];
static uint8_t number_of_scrambling_supported_freqs = 0;
static bt_device_soc_add_on_features_t soc_add_on_features;
static uint8_t soc_add_on_features_length = 0;
static uint16_t product_id, response_version;
static bt_device_host_add_on_features_t host_add_on_features;
static uint8_t host_add_on_features_length = 0;
char qhs_value[PROPERTY_VALUE_MAX] = "0";
uint8_t qhs_support_mask = 0;
static bt_device_qll_local_supported_features_t qll_features;
static bt_configstore_interface_t* bt_configstore_intf = NULL;

void BTM_ConfigQHS();

static inline bool is_byte_valid(char ch) {
  return ((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') ||
          (ch >= 'A' && ch <= 'F'));
}

bool decode_max_power_values(char* power_val) {
  bool status = false;
  char *token, *reset = power_val;
  int i;

  if (!strcmp(power_val, "false")) {
    LOG_INFO(": MAX POW property is not set");
    return false;
  } else if (!strchr(power_val, '-') ||
             (!strchr(power_val, 'x') && !strchr(power_val, 'X')) ||
             strlen(power_val) != 14) {
    LOG_WARN(": MAX POW property is not in required order");
    return false;
  } else {
    status = true;
    for (i = 0; (i < 3 && status); i++) {
      token = strtok_r(reset, "-", &reset);
      if (token && strlen(token) == 4 &&
          (token[0] == '0' && (token[1] == 'x' || token[1] == 'X') &&
           (is_byte_valid(token[2]) && is_byte_valid(token[3])))) {
        max_power_prop_value[i] = (uint8_t)strtoul(token, NULL, 16);
      } else {
        status = false;
      }
    }
  }

  if (status) {
    LOG_DEBUG(
        ": MAX_POW_ID: BR MAX POW:%02x, EDR MAX POW:%02x, BLE MAX POW:%02x",
        max_power_prop_value[0], max_power_prop_value[1],
        max_power_prop_value[2]);
  } else {
    LOG_ERROR(": MAX POW property is not in required order");
  }

  return status;
}

char* BTM_GetA2dpOffloadCapablity() { return &a2dp_offload_Cap[0]; }

bool BTM_IsSpiltA2dpSupported() { return spilt_a2dp_supported; }

bool BTM_IsAACFrameCtrlEnabled() { return aac_frame_ctl_enabled; }

uint8_t* BTM_GetScramblingSupportedFreqs(uint8_t* number_of_freqs) {
  if (number_of_scrambling_supported_freqs) {
    *number_of_freqs = number_of_scrambling_supported_freqs;
    return scrambling_supported_freqs;
  }
  return NULL;
}

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
    uint8_t* host_add_on_features_len) {
  *host_add_on_features_len = host_add_on_features_length;
  return &host_add_on_features;
}

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
    uint8_t* soc_add_on_features_len) {
  *soc_add_on_features_len = soc_add_on_features_length;
  return &soc_add_on_features;
}
/*******************************************************************************
 *
 * Function         BTM_BleIsCisParamUpdateLocalHostSupported
 *
 * Description      This function is called to determine if
 *CIS_Parameter_Update_Host feature is supported by local host.
 *
 * Returns          bool true if supported, false otherwise
 *
 ******************************************************************************/
bool BTM_BleIsCisParamUpdateLocalHostSupported() {
  bool supported = false;

  char value[PROPERTY_VALUE_MAX] = "true";
  property_get("persist.vendor.service.bt.cis_param_update_enabled", value,
               "true");
  if (!strncmp("true", value, 4)) supported = true;

  LOG_INFO(": supported=%d", supported);

  return supported;
}

static void qbce_set_qhs_host_mode_hci_cmd_complete(tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status, subcmd;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(subcmd, stream);
    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, subcmd = %d", opcode,
             length, status, subcmd);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": status success");
    }
  }
}

static void qbce_set_qll_event_mask_hci_cmd_complete(tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status, subcmd;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(subcmd, stream);
    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, subcmd = %d", opcode,
             length, status, subcmd);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": status success");
    }
  }
}

static void qbce_set_qlm_event_mask_hci_cmd_complete(tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status, subcmd;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(subcmd, stream);

    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, subcmd = %d", opcode,
             length, status, subcmd);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": status success");
    }
  }
}

static void qbce_qle_set_host_feature_hci_cmd_complete(tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status, subcmd;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(subcmd, stream);
    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, subcmd = %d", opcode,
             length, status, subcmd);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": Status success");
    }
  }
}
static void parse_qll_read_local_supported_features_response(
    tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status, subcmd;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(subcmd, stream);
    STREAM_TO_ARRAY(qll_features.as_array, stream,
                    (int)sizeof(bt_device_qll_local_supported_features_t));
    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, subcmd = %d", opcode,
             length, status, subcmd);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": status success");
      if (BTM_QBCE_QLL_MULTI_CONFIG_CIS_PARAMETER_UPDATE_CONTROLLER(
              qll_features.as_array) &&
          BTM_BleIsCisParamUpdateLocalHostSupported()) {
        uint8_t cmd[3];
        cmd[0] = QBCE_QLE_SET_HOST_FEATURE;
        cmd[1] = QBCE_QLL_MULTI_CONFIG_CIS_PARAMETER_UPDATE_HOST_BIT;
        cmd[2] = 1;
        BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                                  qbce_qle_set_host_feature_hci_cmd_complete);
      }
    }
  }
}

static void parse_controller_addon_features_response(tBTM_VSC_CMPL* p_data) {
  uint8_t *stream, status;
  uint16_t opcode, length;

  if (p_data && (stream = (uint8_t*)p_data->p_param_buf)) {
    opcode = p_data->opcode;
    length = p_data->param_len;
    STREAM_TO_UINT8(status, stream);

    if (stream && (length > 8)) {
      STREAM_TO_UINT16(product_id, stream);
      STREAM_TO_UINT16(response_version, stream);

      soc_add_on_features_length = length - 5;
      STREAM_TO_ARRAY(soc_add_on_features.as_array, stream,
                      soc_add_on_features_length);
    }

    LOG_INFO(": opcode = 0x%04X, length = %d, status = %d, product_id:%d",
             opcode, length, status, product_id);
    if (status == HCI_SUCCESS) {
      LOG_INFO(": status success");

      if (BTM_SPLIT_A2DP_SCRAMBLING_DATA_REQUIRED(
              soc_add_on_features.as_array)) {
        if (BTM_SPLIT_A2DP_44P1KHZ_SAMPLE_FREQ(soc_add_on_features.as_array)) {
          scrambling_supported_freqs[number_of_scrambling_supported_freqs++] =
              BTAV_A2DP_CODEC_SAMPLE_RATE_44100;
          scrambling_supported_freqs[number_of_scrambling_supported_freqs++] =
              BTAV_A2DP_CODEC_SAMPLE_RATE_88200;
        }
        if (BTM_SPLIT_A2DP_48KHZ_SAMPLE_FREQ(soc_add_on_features.as_array)) {
          scrambling_supported_freqs[number_of_scrambling_supported_freqs++] =
              BTAV_A2DP_CODEC_SAMPLE_RATE_48000;
          scrambling_supported_freqs[number_of_scrambling_supported_freqs++] =
              BTAV_A2DP_CODEC_SAMPLE_RATE_96000;
        }
      }
      BTM_ConfigQHS();
    }
  }
}

void BTM_ConfigQHS() {
  if (BTM_QBCE_QLE_HCI_SUPPORTED(soc_add_on_features.as_array)) {
    BT_HDR* response;
    char qhs_iso[PROPERTY_VALUE_MAX] = "false";
    property_get("persist.vendor.btstack.qhs_enable", qhs_iso, "false");
    uint8_t cmd[3];
    uint8_t sub_cmd = QBCE_SET_QHS_HOST_MODE;

    memset(cmd, 0, 3);

    cmd[0] = sub_cmd;
    cmd[1] = QHS_TRANSPORT_LE_ISO;

    BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                              qbce_set_qhs_host_mode_hci_cmd_complete);
    if (!strncmp("true", qhs_iso, 4)) {
      cmd[2] = QHS_HOST_MODE_HOST_AWARE;
    } else {
      cmd[2] = QHS_HOST_DISABLE_ALL;
    }
    BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                              qbce_set_qhs_host_mode_hci_cmd_complete);
    /* This property is for test/debug purpose only */
    property_get("persist.vendor.btstack.qhs_support", qhs_value, "255");
    LOG_INFO(": qhs property value= %s", qhs_value);
    qhs_support_mask = (uint8_t)atoi(qhs_value);
    LOG_INFO(": qhs support mask=%d", qhs_support_mask);
    if (qhs_support_mask != 0xFF) {
      if (qhs_support_mask & QHS_BREDR_MASK) {
        cmd[1] = QHS_TRANSPORT_BREDR;
        cmd[2] = QHS_HOST_MODE_HOST_AWARE;
      } else {
        cmd[1] = QHS_TRANSPORT_BREDR;
        cmd[2] = QHS_HOST_DISABLE_ALL;
      }
      BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                                qbce_set_qhs_host_mode_hci_cmd_complete);
      if (qhs_support_mask & QHS_LE_MASK) {
        cmd[1] = QHS_TRANSPORT_LE;
        cmd[2] = QHS_HOST_MODE_HOST_AWARE;
      } else {
        cmd[1] = QHS_TRANSPORT_LE;
        cmd[2] = QHS_HOST_DISABLE_ALL;
      }
      BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                                qbce_set_qhs_host_mode_hci_cmd_complete);
      if (qhs_support_mask & QHS_LE_MASK) {
        cmd[1] = QHS_TRANSPORT_LE_ISO;
        cmd[2] = QHS_HOST_MODE_HOST_AWARE;
      } else {
        cmd[1] = QHS_TRANSPORT_LE_ISO;
        cmd[2] = QHS_HOST_DISABLE_ALL;
      }
      BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                                qbce_set_qhs_host_mode_hci_cmd_complete);
    }
    uint8_t cmd_qll[9];
    uint8_t* stream = &cmd_qll[0];
    UINT8_TO_STREAM(stream, QBCE_SET_QLL_EVENT_MASK);
    ARRAY8_TO_STREAM(stream, (&QBCE_QLM_AND_QLL_EVENT_MASK)->as_array);

    BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd_qll), cmd_qll,
                              qbce_set_qll_event_mask_hci_cmd_complete);
  }

  if (BTM_QBCE_QCM_HCI_SUPPORTED(soc_add_on_features.as_array)) {
    uint8_t cmd_qlm[9];
    uint8_t* stream = &cmd_qlm[0];
    ;
    UINT8_TO_STREAM(stream, QBCE_SET_QLM_EVENT_MASK);
    ARRAY8_TO_STREAM(stream, (&QBCE_QLM_AND_QLL_EVENT_MASK)->as_array);

    BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd_qlm), cmd_qlm,
                              qbce_set_qlm_event_mask_hci_cmd_complete);
  }

  if (BTM_QBCE_QLE_HCI_SUPPORTED(soc_add_on_features.as_array)) {
    uint8_t cmd[1];
    cmd[0] = QBCE_READ_LOCAL_QLL_SUPPORTED_FEATURES;

    BTM_VendorSpecificCommand(HCI_VS_QBCE_OCF, sizeof(cmd), cmd,
                              parse_qll_read_local_supported_features_response);
  }
}

void BTM_ReadVendorAddOnFeaturesInternal() {
  bt_configstore_intf = get_btConfigStore_interface();
  if (bt_configstore_intf != NULL) {
    std::vector<vendor_property_t> vPropList;
    bt_configstore_intf->get_vendor_properties(BT_PROP_ALL, vPropList);

    for (auto&& vendorProp : vPropList) {
      switch (vendorProp.type) {
        case BT_PROP_SOC_TYPE:
          char soc_name[32];

          strlcpy(soc_name, vendorProp.value, sizeof(soc_name));
          soc_type =
              bt_configstore_intf->convert_bt_soc_name_to_soc_type(soc_name);
          LOG_INFO(": soc_name:%s, soc_type = %d", soc_name, soc_type);
          break;

        case BT_PROP_A2DP_OFFLOAD_CAP:
          strlcpy(a2dp_offload_Cap, vendorProp.value, sizeof(a2dp_offload_Cap));
          LOG_INFO(": a2dp_offload_Cap = %s", a2dp_offload_Cap);
          break;

        case BT_PROP_SPILT_A2DP:
          if (!strncasecmp(vendorProp.value, "true", sizeof("true"))) {
            spilt_a2dp_supported = true;
          } else {
            spilt_a2dp_supported = false;
          }

          LOG_INFO(":: spilt_a2dp_supported = %d", spilt_a2dp_supported);
          break;

        case BT_PROP_AAC_FRAME_CTL:
          if (!strncasecmp(vendorProp.value, "true", sizeof("true"))) {
            aac_frame_ctl_enabled = true;
          } else {
            aac_frame_ctl_enabled = false;
          }

          LOG_INFO(": aac_frame_ctl_enabled = %d", aac_frame_ctl_enabled);
          break;

        case BT_PROP_MAX_POWER:
          max_power_prop_enabled =
              decode_max_power_values((char*)vendorProp.value);
          LOG_INFO(": max_power_prop_enabled = %d", max_power_prop_enabled);
          break;
        default:
          break;
      }
    }
    host_add_on_features_list_t features_list;

    if (bt_configstore_intf->get_host_add_on_features(&features_list)) {
      host_add_on_features_length = features_list.feat_mask_len;
      if (host_add_on_features_length != 0 &&
          host_add_on_features_length <= HOST_ADD_ON_FEATURES_MAX_SIZE)
        memcpy(host_add_on_features.as_array, features_list.features,
               host_add_on_features_length);
    }

    // Read HCI_VS_GET_ADDON_FEATURES_SUPPORT
    if (soc_type >= BT_SOC_TYPE_CHEROKEE) {
      controller_add_on_features_list_t features_list;

      if (bt_configstore_intf->get_controller_add_on_features(&features_list)) {
        product_id = features_list.product_id;
        response_version = features_list.rsp_version;
        soc_add_on_features_length = features_list.feat_mask_len;
        if (soc_add_on_features_length != 0) {
          if (soc_add_on_features_length <= SOC_ADD_ON_FEATURES_MAX_SIZE) {
            memcpy(soc_add_on_features.as_array, features_list.features,
                   soc_add_on_features_length);
            if (BTM_SPLIT_A2DP_SCRAMBLING_DATA_REQUIRED(
                    soc_add_on_features.as_array)) {
              if (BTM_SPLIT_A2DP_44P1KHZ_SAMPLE_FREQ(
                      soc_add_on_features.as_array)) {
                scrambling_supported_freqs
                    [number_of_scrambling_supported_freqs++] =
                        BTAV_A2DP_CODEC_SAMPLE_RATE_44100;
                scrambling_supported_freqs
                    [number_of_scrambling_supported_freqs++] =
                        BTAV_A2DP_CODEC_SAMPLE_RATE_88200;
              }
              if (BTM_SPLIT_A2DP_48KHZ_SAMPLE_FREQ(
                      soc_add_on_features.as_array)) {
                scrambling_supported_freqs
                    [number_of_scrambling_supported_freqs++] =
                        BTAV_A2DP_CODEC_SAMPLE_RATE_48000;
                scrambling_supported_freqs
                    [number_of_scrambling_supported_freqs++] =
                        BTAV_A2DP_CODEC_SAMPLE_RATE_96000;
              }
            }
            BTM_ConfigQHS();
          } else {
            LOG(FATAL) << __func__ << "invalid soc add on features length: "
                       << +soc_add_on_features_length;
          }
        }
      }
    }
  }
}

/*******************************************************************************
 *
 * Function         BTM_ReadVendorAddOnFeatures
 *
 * Description      BTM_ReadVendorAddOnFeatures
 *
 * Parameters:      None
 *
 ******************************************************************************/
void BTM_ReadVendorAddOnFeatures() {
  bool btConfigStore = true;

  if (btConfigStore) {
    BTM_ReadVendorAddOnFeaturesInternal();
  } else {
    LOG_INFO(": Soc Add On");
    if (true) {
      BTM_VendorSpecificCommand(HCI_VS_GET_ADDON_FEATURES_SUPPORT, 0, NULL,
                                parse_controller_addon_features_response);
    }

    /*if (!HCI_LE_CIS_MASTER_SUPPORT(features_ble.as_array)) {
      adv_audio_support_mask &= ~ADV_AUDIO_UNICAST_FEAT_MASK;
    }
    if (!HCI_LE_PERIODIC_SYNC_TRANSFER_SEND_SUPPORTED(features_ble.as_array)) {
      adv_audio_support_mask &= ~ADV_AUDIO_BCA_FEAT_MASK;
    }
    if (!HCI_LE_ISO_BROADCASTER_SUPPORTED(features_ble.as_array)) {
      adv_audio_support_mask &= ~ADV_AUDIO_BCS_FEAT_MASK;
    }
    snprintf(adv_audio_property, 2, "%d", adv_audio_support_mask);
    osi_property_set("persist.vendor.service.bt.adv_audio_mask",
    adv_audio_property); */
  }
}
