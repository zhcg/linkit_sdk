/* Copyright Statement:
*
* (C) 2005-2017  MediaTek Inc. All rights reserved.
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

#include "iot_lib.h"
#include "patch_config_ram.h"

#define PARTIAL_ACTIVE_SCAN_CHAN_TIME			40		/* in ms unit */
#define PARTIAL_PASSIVE_SCAN_CHAN_TIME			120
#define MAX_CHAN_NUMBER							15
#define BEACON_INTERVAL							100
#define IOT_PID_BEACON			0x0c
#define MAX_CHANNEL_STAY_TIME		280			/* in miliseconds */
#define MIN_CHANNEL_STAY_TIME		70			/* in miliseconds */

extern	BOOLEAN g_PartialScan ;
extern	BOOLEAN g_PartialScanRunning;
extern	UCHAR g_PartialScanChannel;
extern	UCHAR g_PeriodicBcnCnt;
extern	UCHAR g_BcnCntAfterScan;
extern	UINT_32 g_PartialScanTime;
extern	UINT_32 g_PartialScanInterval;
extern	PARTIAL_SCAN_CH_LIST g_PartialScanChList[MAX_CHAN_NUMBER];
extern BOOLEAN g_bEnableRxToHost;
extern UINT16 g_ChannelStayTime;
#ifndef E2_ROM_7682
typedef enum _ENUM_PM_STATE_T {
    PM_STATE_ACTIVE_MODE = 0,
    PM_STATE_POWER_SAVE,
    PM_STATE_NUM
} ENUM_PM_STATE_T, *P_ENUM_PM_STATE_T;

typedef enum _EXT_ENUM_PM_FEATURE_T
{
    PM_CMD_FEATURE_PSPOLL_OFFLOAD = 0x00000001,
    PM_CMD_FEATURE_PS_TX_REDIRECT = 0x00000002,
    PM_CMD_FEATURE_SMART_BCN_SP =   0x00000004,
    PM_CMD_FEATURE_SEND_PS1_NULL =  0x00000008
} EXT_ENUM_PM_FEATURE_T;
#endif
typedef enum _ENUM_PM_BCN_TO_STEPS_T {
    BCN_TO_STEP_0 = 0,
    BCN_TO_STEP_1,
    BCN_TO_STEP_2,
    BCN_TO_STEP_3,
    BCN_TO_STEP_4,
    BCN_TO_STEP_NUM
} ENUM_PM_BCN_TO_STEPS_T;
#define PM_BEACON_TIMEOUT_COUNT_ADHOC          30
#define PM_BEACON_TIMEOUT_COUNT_INFRA          10

#define PM_BEACON_MAX_TIMEOUT_TU               20
#define PM_BEACON_MIN_TIMEOUT_TU               5

#define PM_BEACON_L1_MIN_TIMEOUT_TU            15
#define PM_BEACON_L2_MIN_TIMEOUT_TU            31
#define PM_BEACON_L3_MIN_TIMEOUT_TU            63
#define PM_BEACON_L4_MIN_TIMEOUT_TU            0   // 0 means disable min timeout

#define PM_BEACON_MAX_TIMEOUT_VALID            TRUE
#define PM_BEACON_MIN_TIMEOUT_VALID            TRUE

#define PM_BMC_MAX_TIMEOUT_TU                  20// 100   /* 7603 change BMC timeout from 100 to 20 */
#define PM_BMC_MIN_TIMEOUT_TU                  5
#define PM_BMC_MAX_TIMEOUT_VALID               TRUE
#define PM_BMC_MIN_TIMEOUT_VALID               TRUE

#ifndef E2_ROM_7682
typedef struct _pwr_mgmt_info {

	/*------------------------------------------------------------------------------------------*/
  /* Power Management related fields  (for STA/ AP/ P2P/ BOW power saving mode)*/
  /*------------------------------------------------------------------------------------------*/

  ENUM_PM_STATE_T		  eCurrentPMState;			  /* Indicate the protocal Power saving State */
  ENUM_PWR_STATE_T		  ePwrMacState; 			  /* Indicate whether MAC enter Low power own */
  RALINK_TIMER_STRUCT	  rPsPollSPTimer;			  /* For Infra Mode, the timer for PS-POLL Service Period. (move to BSSInfo)*/
  RALINK_TIMER_STRUCT	  rBcnDynaSPTimer;			  /* Smart dynamic beacon service period timer (move to BSSInfo)*/
  UINT_16				  u2BeaconContentHash;		  /* store last Beacon hash code */
  UINT_16				  u2BeaconInterval; 		  /* The Beacon Interval of this BSS */
  UINT_8				  ucDTIMPeriod; 			  /* For Infra/AP Mode */
  UINT_8				  ucReportBcnTOCount;		  /* beacon TO count (move to BSSInfo)*/
  UINT_8				  ucBcnSpStep;				  /* Service period step (move to BSSInfo)*/
  BOOLEAN				  fgIsInPsPollSP;			  /* For Infra Mode, to indicate we've sent a PS POLL to AP and start*/
														/* the PS_POLL Service Period(LP) (move to BSSInfo) */
  UINT_16                 u2AssocId;
  ENUM_RADIO_STATE_T      rRadioState;
} pwr_mgmt_info;
#endif

#if (MT7682_WSAP00033668 == 1)
typedef struct glbl_wlan_statistics
{
	UINT_32 Current_Temperature;
	UINT_32 Tx_Success_Count;
	UINT_32 Tx_Retry_Count;
	UINT_32 Tx_Fail_Count;
	UINT_32 Rx_Success_Count;
	UINT_32 Rx_with_CRC;
	UINT_32 Rx_Drop;
	UINT_32 MIC_Error_Count;
	UINT_32 Cipher_Error_Count;
	INT_8 Rssi;
	UINT_32 Tx_AGG_Range_1;
	UINT_32 Tx_AGG_Range_2;
	UINT_32 Tx_AGG_Range_3;
	UINT_32 Tx_AGG_Range_4;
	UINT_32 AMPDU_Tx_Success;
	UINT_32 AMPDU_Tx_Fail;
	UINT_32 SNR;
} __attribute__((__packed__)) glbl_wlan_statistics_t, *p_glbl_wlan_statistics;
#endif


#if (MT7682_WSAP00027933 == 1)
extern UCHAR g_no_wait_for_beacon;
#define IOT_PID_NULL_PS_NO_WAIT_FOR_BEACON						0x15
#endif


