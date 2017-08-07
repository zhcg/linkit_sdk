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

#include "ble_gatt_common.h"
#include "ble_lls.h"
#include "ble_gatt_srv.h"
#include "ble_gatt_utils.h"
#include "ble_gatt_srv_internal.h"

#include <FreeRTOS.h>
#include "timers.h"

/*
#include "task.h"
#include "task_def.h"
*/

uint8_t tmp = 0;

typedef struct {
    uint16_t conn_handle;               /**< connect handle */
    uint16_t is_notify;          /**< client config is notify or not*/
    uint8_t lls_value[20];         /**< HeartRate Measurement vaule */

    TimerHandle_t timer;
} lls_app_cntx_t;

lls_app_cntx_t g_lls_app;

static ble_lls_context_t g_lls_context;

static ble_lls_dev_info_t g_lls_dev_info[BLE_GATT_MAX_LINK];

static void bt_lls_notify();

static uint32_t bt_if_lls_alert_level_callback (const uint8_t rw, uint16_t handle,
    void *data, uint16_t size, uint16_t offset);

static uint32_t bt_if_lls_client_config_callback(const uint8_t rw, uint16_t handle, void *data, uint16_t size, uint16_t offset);

static ble_lls_dev_info_t *ble_lls_get_dev_info(ble_lls_dev_info_type_t type,
    void *para);

static void ble_lls_reset_dev_info(ble_lls_dev_info_t *dev_info);

static const bt_uuid_t BT_SIG_UUID_ALERT_LEVEL =
    BT_UUID_INIT_WITH_UUID16(BT_SIG_UUID16_ALERT_LEVEL);

static const bt_uuid_t BT_SIG_UUID_ALERT_LEVEL_READ =
    BT_UUID_INIT_WITH_UUID16(BT_SIG_UUID16_ALERT_LEVEL_READ);

BT_GATTS_NEW_PRIMARY_SERVICE_16(bt_if_lls_primary_service,
    BT_GATT_UUID16_LINK_LOSS_SERVICE);


	/* write */
BT_GATTS_NEW_CHARC_16_WRITABLE(bt_if_lls_char4_alert_level,
    //BT_GATT_CHARC_PROP_READ | BT_GATT_CHARC_PROP_WRITE, BLE_LLS_VAL_HD_AL,
    BT_GATT_CHARC_PROP_WRITE, BLE_LLS_VAL_HD_AL,
    BT_SIG_UUID16_ALERT_LEVEL);

BT_GATTS_NEW_CHARC_VALUE_CALLBACK(bt_if_lls_alert_level, BT_SIG_UUID_ALERT_LEVEL,
    BT_GATTS_REC_PERM_WRITABLE, bt_if_lls_alert_level_callback);


	/* read */
BT_GATTS_NEW_CHARC_16(bt_if_lls_char4_alert_level_read,
	BT_GATT_CHARC_PROP_READ | BT_GATT_CHARC_PROP_NOTIFY | BT_GATT_CHARC_PROP_INDICATE, BLE_LLS_VAL_HD_AL_READ,
    BT_SIG_UUID16_ALERT_LEVEL_READ);

BT_GATTS_NEW_CHARC_VALUE_UINT16(bt_if_lls_alert_level_read, BT_SIG_UUID_ALERT_LEVEL_READ,
                                BT_GATTS_REC_PERM_READABLE,
                                0);
	/* config */
BT_GATTS_NEW_CLIENT_CHARC_CONFIG(bt_if_lls_client_config,
                                 BT_GATTS_REC_PERM_READABLE | BT_GATTS_REC_PERM_WRITABLE,
                                 bt_if_lls_client_config_callback);


static const bt_gatts_service_rec_t *bt_if_lls_service_rec[] = {
    (const bt_gatts_service_rec_t *) &bt_if_lls_primary_service,
    (const bt_gatts_service_rec_t *) &bt_if_lls_char4_alert_level,
    (const bt_gatts_service_rec_t *) &bt_if_lls_alert_level,

    (const bt_gatts_service_rec_t *) &bt_if_lls_char4_alert_level_read,
    (const bt_gatts_service_rec_t *) &bt_if_lls_alert_level_read,

    (const bt_gatts_service_rec_t *) &bt_if_lls_client_config,
};

const bt_gatts_service_t bt_if_lls_service = {
    .starting_handle = BLE_LLS_START_HANDLE,
    .ending_handle = BLE_LLS_END_HANDLE,
    .required_encryption_key_size = BLE_LLS_REKS,
    .records = bt_if_lls_service_rec
};


/*****************************************************************************
* FUNCTION
*  bt_if_lls_alert_level_callback
* DESCRIPTION
* Alert level callback
* PARAMETERS
*  rw            [IN]        Reading or writing
*  handle        [IN]        Connect handle
*  data          [IN]        Data
*  size          [IN]        Data size
*  offset        [IN]        Offset
* RETURNS
*  uint32_t
*****************************************************************************/
static uint32_t bt_if_lls_alert_level_callback (const uint8_t rw, uint16_t handle,
    void *data, uint16_t size, uint16_t offset)
{
    uint32_t ret = 0;
    bt_status_t status;
	int i;
    ble_gatt_char_alter_level_t char_al;
    ble_lls_dev_info_t *dev_info = NULL;

	uint8_t buff[27] = {0};
    bt_gattc_charc_value_notification_indication_t *notify;
    notify = (bt_gattc_charc_value_notification_indication_t *)buff;

    ble_gatt_report("[gatt][lls]al_cb(s)--hd: %d, size: %d, data: %02x %02x %02x %02x\n",
        handle, size, *((uint8_t *)data+0), *((uint8_t *)data+1), *((uint8_t *)data+2), *((uint8_t *)data+3));

	notify->attribute_value_length = 23;
	notify->att_req.opcode = BT_ATT_OPCODE_HANDLE_VALUE_NOTIFICATION;;
	notify->att_req.handle = 0x0704;
	memcpy(notify->att_req.attribute_value, data, 20);
	status = bt_gatts_send_charc_value_notification_indication(handle, notify);
	BT_LOGD("LLS", "bt_lls_notify : notificaiton status = %d", status);

/*
	if (rw == BT_GATTS_CALLBACK_WRITE) {
		LOG_I(common, "slz=====BT_GATTS_CALLBACK_WRITE====size: %d, rw: %d\n", size, rw);
		for (i=0; i<size; i++) {
			LOG_I(common, "data[%d]: 0x%x\n",i, *((uint8_t *)data+i));
		}
	}
*/

/*
	dev_info = ble_lls_get_dev_info(BLE_LLS_DEV_INFO_HANDLE, (void *)&handle);

    if (rw == BT_GATTS_CALLBACK_WRITE) {
        if (size == sizeof(dev_info->alert_level)) { //Size check
            dev_info->alert_level = *(uint8_t *)data;
            ret = size;
            char_al.al = dev_info->alert_level;
            char_al.handle = dev_info->handle;
            ble_gatt_srv_notify(BLE_GATT_SRV_LLS_AL_WRITE, &char_al);
        }
    }
    ble_gatt_report("[gatt][lls]al_cb(e)--ret: %d\n", ret);
*/
    return ret;
}

static uint32_t bt_if_lls_client_config_callback(const uint8_t rw, uint16_t handle, void *data, uint16_t size, uint16_t offset)
{
    ble_gatt_report("client_config_callback : RW= %d, hd= %d, notify= %d, size= %d", rw, handle, *(uint16_t *)data, size);

	if (rw == BT_GATTS_CALLBACK_WRITE) {
		if (size != sizeof(g_lls_app.is_notify)) { //Size check
			return 0;
		}
		g_lls_app.is_notify = *(uint16_t *)data;

		if(g_lls_app.is_notify == CLIENT_CHARC_CONFIGURATION_NOTIFICATION)
		{
			g_lls_app.conn_handle = handle;
		//	g_lls_app.timer = xTimerCreate("LLS Timer", 0xffff, pdFALSE, ( void *) 0,
		//			(TimerCallbackFunction_t)bt_lls_notify);
		//	xTimerChangePeriod(g_lls_app.timer, 60 / portTICK_PERIOD_MS, 0);
		//	xTimerReset(g_lls_app.timer, 0);
		//	bt_timer_start(g_lls_app.timer, 0, 60, bt_lls_notify);

		}
		else
		{
			if (g_lls_app.timer) {
				xTimerStop(g_lls_app.timer, 0);
				xTimerDelete(g_lls_app.timer, 0);
		//		bt_timer_cancel(g_lls_app.timer);
				BT_LOGD("LLS", "bt_lls_notify : stop... ");
			}
		
		}
			
	}else {
		if (size != 0) {
			uint16_t *buf = (uint16_t *) data;
			*buf = g_lls_app.is_notify;
		}
	}
	return sizeof(g_lls_app.is_notify);
}

//======slz temporary define tx rx buffer struct========== 
typedef struct xtransfer_packet {
	uint8_t head_l;				//AA
	uint8_t head_h;				//55
	uint8_t packet_num;			//packet num
	uint8_t buff[17];			//audio compression data
}XTRANSFER;


XTRANSFER g_tx_packet;
XTRANSFER g_rx_packet;
uint8_t g_num = 0;

//======slz end===========================================
/*
	g_tx_packet.head_l = 0xAA;
	g_tx_packet.head_h = 0x55;
	g_tx_packet.packet_num = g_num;
	for (i=0; i<sizeof(g_tx_packet.buff); i++) {
		g_tx_packet.buff[i] = i;
	}
*/

/* heart rate start ...*/
static void bt_lls_notify()
{
    bt_status_t status;
    uint16_t conn_handle;
    TimerHandle_t timer;
    uint8_t buff[6] = {0};
    ble_lls_dev_info_t *dev_info = NULL;
    bt_gattc_charc_value_notification_indication_t *notify;
    notify = (bt_gattc_charc_value_notification_indication_t *)buff;
    ble_gatt_report("bt_lls_notify : start... ");

    conn_handle = g_lls_app.conn_handle;
    timer = g_lls_app.timer;

    dev_info = ble_lls_get_dev_info(BLE_LLS_DEV_INFO_HANDLE, (void *)&conn_handle);

	if (dev_info->handle == NULL) {/*MAYBE link disconnect*/
		xTimerStop(timer, 0);
		xTimerDelete(timer, 0);
		bt_timer_cancel(timer);
		memset(&g_lls_app, 0 , sizeof(lls_app_cntx_t));
		BT_LOGD("LLS", "connection link is invalid bt_lls_notify : stop... ");
	} else {
		if (timer) {
			xTimerChangePeriod(timer, 60 / portTICK_PERIOD_MS, 0);
			xTimerReset(timer, 0);
			bt_timer_cancel(timer);
		}
		notify->attribute_value_length = 23;
		notify->att_req.opcode = BT_ATT_OPCODE_HANDLE_VALUE_NOTIFICATION;;
		notify->att_req.handle = 0x0704;

		if(g_num==128)
			g_num=0;
		g_lls_app.lls_value[0] = g_num++;
		for(tmp=1;tmp < notify->attribute_value_length - 3;tmp++)
			g_lls_app.lls_value[tmp] = tmp;
		memcpy(notify->att_req.attribute_value, &g_lls_app.lls_value, notify->attribute_value_length - 3);
		status =  bt_gatts_send_charc_value_notification_indication(conn_handle, notify);
		BT_LOGD("LLS", "bt_lls_notify : notificaiton packet=%d, status = %d", g_num, status);
	}
}

/*****************************************************************************
* FUNCTION
*  ble_lls_get_dev_info
* DESCRIPTION
* Get connected device's detail infromation
* PARAMETERS
*  type        [IN]        Device type
*  para        [IN]        Pointer
* RETURNS
*  ble_ias_dev_info_t
*****************************************************************************/
static ble_lls_dev_info_t *ble_lls_get_dev_info(
    ble_lls_dev_info_type_t type, void *para)
{
    int32_t i = 0, ret = 0;
    ble_lls_dev_info_t *info = NULL;
    ble_lls_context_t *lls_ctx = NULL;
    bt_handle_t *hd = NULL;

    lls_ctx = ble_lls_get_context();

    ble_gatt_report("[gatt][lls]get_dev_info(s)--type: %d, para: 0x%x\n",
        type, para);

    switch (type) {

        case BLE_LLS_DEV_INFO_HANDLE: {
            hd = (bt_handle_t *) para;
            for (i = 0; i < BLE_GATT_MAX_LINK; ++i) {
                if ((lls_ctx->dev_info[i].flag & BLE_LLS_DEV_FLAG_USED) &&
                    lls_ctx->dev_info[i].handle == *(hd)) {
                    info = &(lls_ctx->dev_info[i]);
                    break;
                }
            }
            break;
        }

        case BLE_LLS_DEV_INFO_UNUSED: {
            for (i = 0; i < BLE_GATT_MAX_LINK; ++i) {
                if (!(lls_ctx->dev_info[i].flag & BLE_LLS_DEV_FLAG_USED)) {
                    info = &(lls_ctx->dev_info[i]);
                    lls_ctx->dev_info[i].flag |= BLE_LLS_DEV_FLAG_USED;
                    break;
                }
            }
            break;
        }
        default:
            break;
    }

    ble_gatt_report("[gatt][lls]get_dev_info(e)--info: 0x%x, ret: %d\n",
        info, ret);

    return info;
}


/*****************************************************************************
* FUNCTION
*  ble_lls_reset_dev_info
* DESCRIPTION
* Reset connection information pointer
* PARAMETERS
*  dev_info        [IN]        Connect_info pointer
* RETURNS
*  void
*****************************************************************************/
static void ble_lls_reset_dev_info(ble_lls_dev_info_t *dev_info)
{
    ble_gatt_memset(dev_info, 0x00, sizeof(ble_lls_dev_info_t));
}


/*****************************************************************************
* FUNCTION
*  ble_lls_init
* DESCRIPTION
* IAS init
* PARAMETERS
*  void
* RETURNS
*  void
*****************************************************************************/
void ble_lls_init(void)
{
    ble_gatt_memset(&g_lls_context, 0x00, sizeof(ble_lls_context_t));
    ble_gatt_memset(&g_lls_dev_info[0], 0x00,
        sizeof(ble_lls_dev_info_t) * BLE_GATT_MAX_LINK);

    g_lls_context.dev_info = g_lls_dev_info;

    ble_gatt_report("[gatt][lls]init\n");
}


/*****************************************************************************
* FUNCTION
*  ble_lls_dev_connect
* DESCRIPTION
* Notify LLS connection
* PARAMETERS
*  handle        [IN] Connection handle
* RETURNS
*  void
*****************************************************************************/
void ble_lls_dev_connect(bt_handle_t handle)
{
    ble_lls_dev_info_t *dev_info = NULL;

    dev_info = ble_lls_get_dev_info(BLE_LLS_DEV_INFO_UNUSED, NULL);
    dev_info->handle = handle;
}


/*****************************************************************************
* FUNCTION
*  ble_lls_dev_disconnect
* DESCRIPTION
* Notify LLS disconnection
* PARAMETERS
*  handle        [IN] Connection handle
* RETURNS
*  void
*****************************************************************************/
void ble_lls_dev_disconnect(bt_handle_t handle)
{
    ble_lls_dev_info_t *dev_info = NULL;

    dev_info = ble_lls_get_dev_info(BLE_LLS_DEV_INFO_HANDLE, (void *)&handle);
    ble_lls_reset_dev_info(dev_info);
}


/*****************************************************************************
* FUNCTION
*  ble_lls_get_context
* DESCRIPTION
* Get LLS context
* PARAMETERS
*  void
* RETURNS
*  ble_ias_context_t
*****************************************************************************/
ble_lls_context_t *ble_lls_get_context(void)
{
    return &g_lls_context;
}

