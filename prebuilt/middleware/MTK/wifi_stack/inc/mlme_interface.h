/* Copyright Statement:
 *
 * (C) 2005-2016  MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. ("MediaTek") and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User").  If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */


#ifndef __MLME_INTERFACE_H__
#define __MLME_INTERFACE_H__

#include "wifi_api.h"

extern int fw_mlme_set_ssid(unsigned char port, unsigned char *ssid, unsigned char ssid_length);

extern int fw_mlme_get_ssid(unsigned char port, unsigned char *ssid, unsigned char *ssid_length);

extern int fw_mlme_set_security(unsigned char port, unsigned char auth_mode, unsigned char encrypt_type);

extern int fw_mlme_get_security(unsigned char port, unsigned char *auth_mode, unsigned char *encrypt_type);

extern int fw_mlme_set_wpapsk(unsigned char port, unsigned char *passphrase, unsigned char passphrase_length);

extern int fw_mlme_get_wpapsk(unsigned char port, unsigned char *passphrase, unsigned char *passphrase_length);

extern int fw_mlme_set_wep_key(unsigned char port, wifi_wep_key_t *wep_keys);

extern int fw_mlme_get_wep_key(unsigned char port, wifi_wep_key_t *wep_keys);

extern int fw_mlme_get_opmode(uint8_t *mode);

extern int fw_mlme_get_link_status(uint8_t *link_status);

extern int fw_mlme_set_bssid(uint8_t *bssid);

extern int fw_mlme_set_opmode(uint8_t mode);

extern int fw_mlme_set_reload(void);

extern int fw_mlme_set_radio(uint8_t opmode ,uint8_t onoff);

extern int fw_mlme_disconnect_ap(void);

extern int fw_mlme_find_connected_sta_by_mac(uint8_t * mac);

#endif
