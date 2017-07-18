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

/**
 * @file        wifi_cli.h
 * @brief       WiFi API Example CLI
 *
 * @history     2015/08/18   Initial for 1st draft  (Michael Rong)
 */

/**@addtogroup WiFiCLI
* @{
* This section introduces the Wi-Fi CLI APIs including terms and acronyms, supported features,
* details on how to use the Wi-CLI, function groups, enums, structures and functions.
* @section WiFiCLI_Terms_Chapter Wi-Fi CLI Command List
*  |        Command Format                 |           Parameter  Value       |Usage Example|
*  |---------------------------------------|----------------------------------|-------------|
*  | <br><b>wifi config set opmode <mode></b></br>  |<br><b><mode></b></br>:\n 1: STA  mode;\n  2: AP mode;\n  3: Repeater mode\n 4: Monitor mode |<br><b><usage_example></b></br> wifi config set opmode 1\n <br><b><note></b></br> Set the device's current operation mode as STA mode.|
*  | <br><b>wifi config get opmode</b></br>  |<br><b>output</b></br>:\n 1: STA  mode; \n 2: AP mode;\n  3: Repeater mode |<br><b><usage_example></b></br> wifi config get opmode\n  <br><b><note></b></br>  Get the current operation mode.|
*  | <br><b>wifi config set ssid <port> <ssid></b></br>  |<br><b><port></b></br>:\n  0: STA/APCLI; \n 1: AP\n <br><b><ssid></b></br>\n The ssid of target AP| <br><b><usage_example></b></br> wifi config set ssid 0 APRouter\n  <br><b><note></b></br>  Set the device STA/APCLI port's ssid named as "APRouter". \n The device will connect the AP which has the same SSID, \n when the device is doing wifi connection process.|
*  | <br><b>wifi config get ssid <port></b></br> |<br><b>output</b></br>:\n 1: STA  mode;\n  2: AP mode; \n 3: Repeater mode |<br><b><usage_example></b></br> wifi config get ssid 0\n  <br><b><note></b></br>  Get the device's STA or APCLI port's  current SSID.\n <br><b><usage_example></b></br> wifi config get ssid 1\n  <br><b><note></b></br>  Get  current SSID for AP port|
*  | <br><b>wifi config set bssid <BSSID></b></br> |<br><b><BSSID></b></br>:\n target AP's Wi-Fi BSSID|\n <br><b><usage_example></b></br> wifi config set bssid 00:0c:45:56:78:90\n  <br><b><note></b></br> Set the target AP's BSSID when device is STA mode, \n the device will connect the AP which has the same BSSID, \n when the device doing wifi connection process.\n  |
*  | <br><b>wifi config get bssid</b></br> |<br><b>output</b></br>: \n Target AP's Wi-Fi BSSID |<br><b><usage_example></b></br> wifi config get bssid \n  <br><b><note></b></br> Get the current Wi-Fi BSSID which the device used to connect to AP,  or the AP's BSSID which the device is connecting.|
*  | <br><b>wifi config set sec <port> <auth_mode> <encrypt_type></b></br> |<br><b><port></b></br>:\n  0: STA/APCLI;  \n  1: AP\n Detail authentication mode and encryption mode info please refer to section <br><b>Combinations of Auth Mode and Encrypt Type</b></br>.  |<br><b><usage_example></b></br> wifi config set sec 0 0 1 \n  <br><b><note></b></br> Set the authentication mode and encryption mode for the specified AP.\n <br><b><usage_example></b></br> wifi config set sec 0 7 6\n  <br><b><note></b></br> Set the device STA/APCLI port's security setting as WPA2-PSK AES mode. \n The device must connect the AP with same security setting.|
*  | <br><b>wifi config get sec <port></b></br> |<br><b>output</b></br>:\n Get the authentication mode and encryption mode  |<br><b><usage_example></b></br> wifi config get sec 0\n  <br><b><note></b></br> This cli get the device STA/APCLI's current security setting.|
*  | <br><b>wifi config set psk <port> <password></b></br>  |<br><b><port></b></br>:\n  0: STA/APCLI;  1: AP\n <br><b><password></b></br>:\n 8 ~ 63 bytes ASCII or 64 bytes Hex |<br><b><usage_example></b></br> wifi config set psk 0 12345678\n  <br><b><note></b></br> Set the device STA/APCLI port's password as "12345678", \n  the device will use this password to connect target AP. \n If the password has 64 character, it will be treated as PMK which is the 64 HEX value |
*  | <br><b>wifi config get psk <port></b></br> |<br><b>output</b></br>:\n The current password.| <br><b><usage_example></b></br>  wifi config get psk 0\n  <br><b><note></b></br> Get the device STA/APCLI port's current password|
*  | <br><b>wifi config set pmk <port> <pmk></b></br>  |<br><b><port></b></br>:\n  0: STA/APCLI; \n  1: AP\n <br><b><pmk></b></br>:\n Hex:  00, 05, 03,...(size 32)| <br><b><note></b></br> Set PMK for the specified STA/AP port. It is replaced by psk API/CLI|
*  | <br><b>wifi config get pmk <port></b></br> |PMK:\n Hex:  00, 05, 03 ,...(size 32)|  <br><b><note></b></br> get the PMK information of the specified STA/AP port.It is replaced by psk API/CLI. |
*  | <br><b>wifi config set rxfilter <flag></b></br> | <br><b><flag></b></br>:\n flag defined in wifi_rx_filter_t of wifi api reference| <br><b><usage_example></b></br> wifi config set rxfilter 0x1de00a\n  <br><b><note></b></br> Set the filter to drop specific packets.\n bit 1 Drops the FCS error frames.\n bit 3 Drops the version field of Frame Control field.\n bit 13 Drops reserved definition control frames.\n bit 14 Drops CTS frames.\n bit 15 Drops RTS frames.\n bit 16 Drops duplicate frames.\n bit 18 Drops not unicast to me frames.\n bit 19 Drops different BSSID TIM Broadcast frame  |
*  | <br><b>wifi config get rxfilter</b></br> |<br><b>output</b></br>:\n flag defined in  wifi_rx_filter_t of wifi api reference| <br><b><usage_example></b></br>  wifi config get rxfilter \n  <br><b><note></b></br> get current rx filter settings|
*  | <br><b>wifi config set mtksmart <flag></b></br> |<br><b><flag></b></br>:\n 0: disable;  \n 1: enable\n | <br><b><usage_example></b></br>  wifi config set mtksmart 1 \n  <br><b><note></b></br> Enable mtk smart connection Filter.\n|
*  | <br><b>wifi config get mtksmart </b></br> |<br><b>output</b></br>:\n the value of mtksmart flag| <br><b><usage_example></b></br>  wifi config set mtksmart \n get the current mtksmart flag value.|
*  | <br><b>wifi config set wep <port> <key_id> <key_string_id></b></br>  |<br><b><port></b></br>:\n  0: STA/APCLI; \n 1: AP\n <br><b><key_id></b></br>\n 0~3\n <br><b><key_string_id></b></br>:\n The key string of specified id | <br><b><usage_example></b></br>  wifi config set wep 0 0 1234567890\n  <br><b><note></b></br> Set the device STA/APCLI port's WEP key \n index0's value as "0x12 0x34 0x56 0x78 0x90"\n  <br><b><usage_example></b></br>  wifi config set wep 0 0 12345 \n  <br><b><note></b></br> Set the device STA/APCLI port's WEP key \n index0's value as "0x31 0x32 0x33 0x34 0x35" \n The wep key string length should be 5 or 10 or 13 or 26. \n Key string with 5 or 13 length be treated as ASCLL, \n Key string with 10 or 26 length be treated as Hex|
*  | <br><b>wifi config get wep <port> <key_id></b></br> |<br><b>output</b></br>:\n Get the key string of specified key id | <br><b><usage_example></b></br>  wifi config get wep 0 0\n  <br><b><note></b></br> Get the key string of the key index0 for the device's STA/APCLI port.|
*  | <br><b>wifi config set ch <port> <ch></b></br> |<br><b><port></b></br>:\n  0: STA/APCLI; \n 1: AP\n <br><b><ch></b></br>:\n 1~14 are supported for 2.4G only product | <br><b><usage_example></b></br>  wifi config set ch 0 1\n  <br><b><note></b></br> Set the device STA/APCLI port's current channel as 1\n The channel needs to be limited and  \n match the country region/country code/wireless mode setting.|
*  | <br><b>wifi config get ch <port></b></br>  |<br><b>output</b></br>:\n The current channel| <br><b><usage_example></b></br>  wifi config get ch 0\n  <br><b><note></b></br> Get the device STA/APCLI port's current channel|
*  | <br><b>wifi config set bw <port> <bw></b></br> |<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP\n <br><b><bw></b></br>:\n Bandwidth supported:\n 0: 20MHz\n 1: 40MHz | <br><b><usage_example></b></br>  wifi config set bw 0 0\n  <br><b><note></b></br> Set the device STA/APCLI port's bandwidth as 20MHz. \n The bandwidth needs to be limited and match the wireless mode settings.|
*  | <br><b>wifi config get bw <port></b></br> |<br><b>output</b></br>:\n The current bandwidth | <br><b><usage_example></b></br>  wifi config get bw 0\n Get the device STA/APCLI port's current bandwidth |
*  | <br><b>wifi config set wirelessmode <port> <mode></b></br> |<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP\n <br><b><mode></b></br>:\n 0 - - 11BG mixed\n 1 - - 11B only\n 2 - - 11A only	(5G band, not support)\n 3 - - 11A/B/G mixed (not supported)\n 4 - - 11G only\n 5 - - 11ABGN mixed (both band, not support )\n 6 - - 11N only in 2.4G\n 7 - - 11GN mixed\n 8 - - 11AN mixed (5G band, not support)\n 9 - - 11BGN mixed\n 10 - - 11AGN mixed (not support)\n 11 - - 11N  5G (not support) | <br><b><usage_example></b></br>  wifi config set wirelessmode 0 0\n  <br><b><note></b></br> Set the current wirless mode as 11BG mixed mode|
*  | <br><b>wifi config get wirelessmode <port></b></br> |<br><b>output</b></br>:\n The current wirelessmode | <br><b><usage_example></b></br>  wifi config get wirelessmode 0\n  <br><b><note></b></br> Get the device STA/APCLI port's wireless mode |
*  | <br><b>wifi config set country <band> <region></b></br> |<br><b><band></b></br>:\n 0: 2.4G; \n 1: 5G\n <br><b><region></b></br>:\n WiFi Country Region.  | <br><b><usage_example></b></br>   wifi config set country 0 1 \n  <br><b><note></b></br> Set 2.4G band country Region as 1. \n Details of country region and channel range of 2.4G band or 5G band \n please refer to LinkIt for RTOS Wi-Fi Developer's Guide under the doc folder in the SDK package. |
*  | <br><b>wifi config get country <band></b></br> |<br><b>output</b></br>:\n The country region info | <br><b><usage_example></b></br>  wifi config get country 0\n  <br><b><note></b></br> Get 2.4G Band country region value \n  <br><b><usage_example></b></br>  wifi config get country 1\n  <br><b><note></b></br> Get 5G Band country region value |
*  | <br><b>wifi config set country_code <country_code></b></br> |<br><b><country_code></b></br>: \n the country code that the driver uses. For more information please refer to wifi_country_code_t in Wi-Fi API reference guide.\n | <br><b><usage_example></b></br>  wifi config set country_code CN \n  <br><b><note></b></br> Only support country code: CN, US, UK for reference design.|
*  | <br><b>wifi config get country_code</b></br> |<br><b>output</b></br>: \n The country region info. | <br><b><usage_example></b></br>  wifi config get country_code  \n  <br><b><note></b></br> Get the country code information.|
*  | <br><b>wifi config get mac <port></b></br> |<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP\n | <br><b><usage_example></b></br>  wifi config get mac 0\n  <br><b><note></b></br> Get the device's STA/APCLI port's mac address|
*  | <br><b>wifi config set reload</b></br> |<br><b>output</b></br>:None\n   | <br><b><usage_example></b></br>  wifi config set reload \n  <br><b><note></b></br> Reload the wifi configuration, \n and make it take effect, such as: \n ssid, psk, sec , pmk, wep and so on |
*  | <br><b>wifi config set radio <on_off>  </b></br> |<br><b><on_off></b></br>:\n 0: OFF; \n 1: ON\n | <br><b><usage_example></b></br>  wifi config set radio 1\n  <br><b><note></b></br> WIFI interface radio will be turn on\n  In dual mode, both WIFI interface radio will be turn on/off at the same time.|
*  | <br><b>wifi config get radio</b></br> |<br><b>output</b></br>:\n 0: OFF; \n 1: ON| <br><b><usage_example></b></br>  wifi config get radio\n  <br><b><note></b></br> Get the current status of WIFI interface radio |
*  | <br><b>wifi config set rxraw <enable></b></br> |<br><b><enable></b></br>:\n 0: unregister \n 1: register | <br><b><usage_example></b></br>  wifi config set rxraw 1 \n  <br><b><note></b></br> Enable the pre-registered RX RAW packet handler,\n the 802.11 packet will be report to the  handler, \n and print part of raw content. \n This CLI can cooperate with another CLI: \n wifi config set rxfilter ***  for the test and verification|
*  | <br><b>wifi config set txraw <raw_packets></b></br>|<br><b><raw_packets></b></br>:\n 802.11 packets content that includes 802.11 MAC header + Payload, \n it does not include the FCS.| <br><b><usage_example></b></br>  (send a NULL packet):\n  wifi config set txraw 48113C00D0C7C08200437C1DD97038BFD0C7C08200437001B3015194.\n  <br><b><note></b></br> Send a TX RAW packet(A 802.11 NULL packet) with basic rate (CCK, 1Mbps)|
*  | <br><b>wifi config set listen <listen_interval></b></br>|<br><b><listen_interval></b></br>:1~255 | <br><b><usage_example></b></br>  wifi config set listen 2 \n  <br><b><note></b></br> Set device's wifi listen interval value as 2, \n only used when device is STA mode|
*  | <br><b>wifi config get listen</b></br>|<br><b>output</b></br>:\nlisten interval(range is 1~255)| <br><b><usage_example></b></br>  wifi config get listen \n Get device's current wifi listen interval value|
*  | <br><b>wifi config set dtim <dtim_interval></b></br>|<br><b><dtim_interval></b></br>:1~255 | <br><b><usage_example></b></br>  wifi config set dtim 1 \n  <br><b><note></b></br> set device's wifi DTIM interval value as 1, \n only used when device is AP mode.|
*  | <br><b>wifi config get dtim</b></br>|<br><b>output</b></br>:\n DTIM interval(range is 1~255)| <br><b><usage_example></b></br>  wifi config get dtim \n  <br><b><note></b></br> Get device's current wifi DTIM interval value|
*  | <br><b>wifi connect get linkstatus</b></br>|<br><b>output</b></br>:\n 0: disconnected \n 1: connected| <br><b><usage_example></b></br>  wifi connect get linkstatus \n  <br><b><note></b></br> Return the link status , only for STA mode|
*  | <br><b>wifi connect get stalist</b></br>|<br><b>output</b></br>:\n station list information \n | <br><b><usage_example></b></br>  wifi connect get stalist \n  <br><b><note></b></br> Return the number of associated stations \n and the stations information only for AP mode.|
*  | <br><b>wifi connect get max_num_sta</b></br>|<br><b>output</b></br>:\n Max station link supported.| <br><b><usage_example></b></br>  wifi connect get max_num_sta \n Get the maximum number of supported stations \n in AP mode or Repeater mode|
*  | <br><b>wifi connect get rssi</b></br>|<br><b>output</b></br>:\n Get rssi of the connected AP\n | <br><b><usage_example></b></br>  wifi connect get rssi \n  <br><b><note></b></br> Get rssi of the connected AP. \n Only used for STA mode and \n the station has connected to the AP.|
*  | <br><b>wifi connect set deauth <MAC></b></br>|<br><b><MAC></b></br>:the station's mac address plan to disconnect.| <br><b><usage_example></b></br>  wifi connect set deauth 00:0c:ae:80:03:01\n  <br><b><note></b></br> Disconnect the station with this MacAdderss, only used in AP mode.|
*  | <br><b>wifi connect set eventcb <enable> <enable_ID></b></br>|<br><b><enable></b></br>:\n 0:register \n 1:unregister\n <br><b><enable_ID></b></br>:\n 0: link up event\n 1:scan complete event\n 2:disconnect event\n 3:port secure event\n 4:report beacon/probe response frames.| <br><b><usage_example></b></br>  wifi connect set eventcb 1 0 \n  <br><b><note></b></br> Register the example handler for connected event, \n when the event is reported,the handler will be triggered. |
*  | <br><b>wifi wps set device_info <Device_Name> <Manufacturer> <Model_Name> <Model_number> <Serial_number></b></br>|<br><b><Device_Name></b></br>:The name of the device\n <br><b><Manufacturer></b></br>: Manufacturer of the device\n  <br><b><Model_Name></b></br>:Module name of the device\n <br><b><Model_number></b></br>:Module number of the device\n <br><b><Serial_number></b></br>: Serial number of the device | <br><b><usage_example></b></br>  wifi wps set device_info "MTK IoT Demo" "MTK Demo" "MTK Wireless Device Demo" "MTK7687 Demo" "123456789" \n |
*  | <br><b>wifi wps get device_info</b></br>|<br><b>output</b></br>:\n The WPS device information | <br><b><usage_example></b></br>  wifi wps get device_info\n  <br><b><note></b></br> Get the WPS device information |
*  | <br><b>wifi wps set auto_connection <on_off></b></br>|<br><b><on_off></b></br>:\n on: enable \n off: disable | <br><b><usage_example></b></br>  wifi wps set auto_connection on\n   <br><b><note></b></br> The enrollee will automatically connect to the target AP \n obtained from the WPS credential information after WPS process is complete. |
*  | <br><b>wifi wps get auto_connection</b></br>|<br><b>output</b></br>:\n 1: on \n 0: off | <br><b><usage_example></b></br>  wifi wps get auto_connection\n  <br><b><note></b></br> Get the current status of WPS auto connection |
*  | <br><b>wifi wps get pin_code <port></b></br>|<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP | <br><b><usage_example></b></br>  wifi wps get pin_code 0\n  <br><b><note></b></br> Get 8-digital PIN code of STA.|
*  | <br><b>wifi wps trigger pbc <port> <BSSID></b></br>|<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP\n <br><b><BSSID></b></br>:\n any: no special AP BSSID\n xx:xx:xx:xx:xx:xx :48 bit AP BSSID address| <br><b><Example_STA></b></br> :\n (1) wifi wps trigger pbc 0 any \n Trigger STA WPS PBC with any triggered WPS PBC AP\n  (2) wifi wps trigger pbc 0 01:02:03:04:05:06 \n  STA WPS PBC with BSSID=01:02:03:04:05:06 WPS PBC AP\n <br><b><Example_AP></b></br>:\n wifi wps trigger pbc 1 any \n Ttrigger AP WPS PBC.  AP will ignor BSSID, so please only input "any".|
*  | <br><b>wifi wps trigger pin <port> <BSSID> <PIN_code></b></br>|<br><b><port></b></br>:\n  0: STA/APCLI;  \n 1: AP\n <br><b><BSSID></b></br>:\n any: no special AP BSSID\n xx:xx:xx:xx:xx:xx :48 bit AP BSSID address\n <br><b><PIN_code></b></br>:\n Support both Static PIN code and Dynamic PIN code input.\n  Dynamic PIN code is generated by cli command "wifi wps get pin_code"| <br><b><Example_STA></b></br> :\n (1) wifi wps trigger pin 0 any  XXXXXXXX:\n "XXXXXXXX" instead a 8-digital PIN code got from CLI "wifi wps get pin_code", The  STA do WPS PIN connection with any triggered WPS PIN AP. On AP side, we need input this PIN  in it.\n (2) wifi wps trigger pin 0 x:xx:xx:xx:xx:xx XXXXXXXX:\n STA do WPS PIN use "XXXXXXXX" PIN code with BSSID="xx:xx:xx:xx:xx:xx" AP. \n  <br><b><Example_AP></b></br> :\n wifi wps trigger pin 1 any XXXXXXXX:\n AP will ignore BSSID, so please only input "any". \n  Then AP will do WPS PIN connection with the STA's PIN code="XXXXXXXX".|
*  | <br><b>config read <group_name> <data_item_name></b></br>|Read the data from NVDM with specified group name and data item name.\n <br><b><group_name></b></br>: \n the group name of the data item, such as:\n common; STA; AP\n <br><b><data_item_name></b></br>: \n the name of the data item, such as:\n IpMode, MacAddr, IpAddr, OpMode, Ssid, SsidLen, BW, AuthMode, EncrypType, WpaPsk, WpaPskLen, DefaultKeyId \n  ListenInterval(range from 1 to 255), \n SharedKey\n (aaaaaaaaaaaaa,bbbbbbbbbbbbb,\n ccccccccccccc,ddddddddddddd),\n SharedKeyLen\n (the key index is 0~4, and the length of each key should be 5,10,13 or 26),\n PSMode(0 means disable, 1 means enable), \n KeepAlivePeriod(it is not used now), \n BeaconLostTime | <br><b><usage_example></b></br>  config read STA AuthMode\n  <br><b><note></b></br> Read authmode of STA from NVDM |
*  | <br><b>config write <group_name> <data_item_name> <item_value></b></br>|Write value of specified group name and data item name to NVDM.\n <br><b><group_name></b></br>: \n the group name of the data item, such as:\n common; STA; AP\n <br><b><data_item_name></b></br>: \n The same with config read cli.\n <br><b><item_value></b></br>: \n The value depend on the specified group name and data item name. | <br><b><usage_example></b></br>  :\n 1.config write STA MacAddr 00:00:4c:76:87:01\n (Write the MAC address of STA to NVDM)\n 2. config write STA SsidLen 6\n (the maximum length of SSID is 32 bytes)\n 3. config write STA BW 1\n (BW:0 IS 20MHz, 1 is 40MHz)\n 4. config write STA WirelessMode 9\n (set the wireless mode of station as 11bgn mixed mode, detail wireless mode refer to "wifi config set wirelessmode <port> <mode>" )\n 5. config write STA IpMode dhcp\n (set the IpMode of station as dhcp)\n 6. config write STA BeaconLostTime 2\n (if the station does not receive the beacon after 2s,then the station will disconnect from the AP)  |
*  | <br><b>config reset <group_name></b></br>|<br><b><group_name></b></br>: \n the group name of the data item, such as:\n common; STA; AP\n | <br><b>Example1</b></br>:\n config reset STA\n Recover the default value of the group STA.\n <br><b>Example2</b></br>: \n config reset\n Recover the default value of the group not specified. |
*  | <br><b>config show <group_name></b></br>|<br><b><group_name></b></br>: \n the group name of the data item, such as:\n common; STA; AP| <br><b><usage_example></b></br>  :\n config show STA\n  <br><b><note></b></br> Show content of the group STA.\n  <br><b><usage_example></b></br>  : config show\n  <br><b><note></b></br> Show content of the group not specified. |
*  | <br><b>smart connect <type> <key></b></br>|<br><b><type></b></br>: \n b: broadcast (V5) \n m:multicast (V4) \n <br><b><key></b></br>: \n 16 bytes key string. \n This parameter is optional. User can use it to encrypt the smart connection data.| <br><b><usage_example></b></br>  :smart connect bm \n  <br><b><note></b></br> start smart connection both V5, V4 protocol \n  <br><b><usage_example></b></br>  : smart connect b \n  <br><b><note></b></br> start smart connection V5 protocol only \n  <br><b><usage_example></b></br>  : smart connect m \n  <br><b><note></b></br> start smart connection V4 protocol only. \n  <br><b><usage_example></b></br>  : smart connect \n  <br><b><note></b></br> start smart connection both V5, V4 protocol |
*  | <br><b>smart stop</b></br>|Stop smart conenction | <br><b><usage_example></b></br>  :smart stop \n  <br><b><note></b></br> stop smart connection |
*
*
*
* @section WiFiCLI_Table3_Chapter Wi-Fi CLI Usage Examples
*  | Example  Items                |	"wifi Config" Cli Example       |
*  |-------------------------------|----------------------------------|
*  |<br><b>MT7687 Station Mode</b></br>| |
*  |<br><b>AP Router :OPEN </b></br>|<br><b>connect AP with OPEN mode:</b></br>\n wifi config set opmode 1\n wifi config set ssid 0 MTK_AP\n wifi config set reload |
*  |<br><b>AP Router :WPA2-PSK(AES)</b></br>|<br><b>connect AP with WPA2-PSK(AES)mode, password is 12345678:</b></br>\n wifi config set opmode 1\n wifi config set ssid 0 MTK_AP\n wifi config set psk 0 12345678\n wifi config set reload |
*  |<br><b>AP Router :WPA2-PSK(TKIP)</b></br> |<br><b>connect AP with WPA2-PSK(TKIP) mode:</b></br>\n wifi config set opmode 1\n wifi config set ssid 0 MTK_AP\n wifi config set psk 0 12345678\n wifi config set reload |
*  |<br><b>AP Router :WPA-PSK(TKIP)</b></br> |<br><b>connect AP with WPA-PSK(TKIP) mode,password is 12345678:</b></br>\n wifi config set opmode 1\n wifi config set ssid 0 MTK_AP\n wifi config set psk 0 12345678\n wifi config set reload |
*  |<br><b>AP Router :WEP(OPEN)(64bit)</b></br> |<br><b>connect AP with WEP OPEN(64bit) mode,WEP key is 12345:</b></br>\n wifi config set opmode 1\n wifi config set ssid 0 MTK_AP\n wifi config set wep 0 0 12345\n (use the first key index,the key index start from 0)\n wifi config set reload|
*  |<br><b>MT7687 SoftAP Mode</b></br>| |
*  |<br><b>Soft AP :OPEN</b></br>|<br><b>config SoftAP as OPEN mode:</b></br>\n wifi config set opmode 2\n wifi config set ssid 1 MTK_AP\n wifi config set sec 1 0 1\n wifi config set bw 1 1(set the bandwidth of softAP as 40MHZ)\n wifi config set reload|
*  |<br><b>Soft AP :WPA2-PSK(AES)</b></br>|<br><b>config SoftAP as WPA2-PSK(AES) mode,Password is 12345678:</b></br>\n wifi config set opmode 2\n wifi config set ssid 1 MTK_AP\n wifi config set sec 1 7 6\n wifi config set psk 1 12345678\n wifi config set bw 1 1\n wifi config set reload|
*  |<br><b>Soft AP :WPA2-PSK(TKIP)</b></br>|<br><b>config SoftAP as WPA2-PSK(TKIP) mode,Password is 12345678:</b></br>\n wifi config set opmode 2\n wifi config set ssid 1 MTK_AP\n wifi config set sec 1 7 4\n wifi config set psk 1 12345678\n wifi config set bw 1 1\n wifi config set reload|
*  |<br><b>Soft AP :WPA-PSK(TKIP)</b></br>|<br><b>config SoftAP as WPA-PSK(TKIP) mode,Password is 12345678:</b></br>\n wifi config set opmode 2\n wifi config set ssid 1 MTK_AP\n wifi config set sec 1 4 4\n wifi config set psk 1 12345678\n wifi config set bw 1 1\n wifi config set reload|
*  |<br><b>Soft AP :WEP(OPEN)(64bit)</b></br>|<br><b>config SoftAP as WEP(OPEN)(64bit)mode,Password is 12345:</b></br>\n wifi config set opmode 2\n wifi config set ssid 1 MTK_AP\n wifi config set sec 1 0 0\n wifi config set wep 1 0 12345(use the first key index,the key index start from 0)\n wifi config set bw 1 1\n wifi config set reload|
*  |<br><b>Repeater Mode</b></br>| |
*  |<br><b>AP Port :Open Mode\n AP Client Port:Open Mode</b></br>|<br><b>The remote AP Router is in OPEN mode, 40MHz, channel 6:</b></br>\n wifi config set opmode 3\n wifi config set ssid 0 MTK_AP\n wifi config set ssid 1 MTK_SOFT_AP\n wifi config set sec 1 0 1\n wifi config set ch 1 6\n wifi config set bw 1 1\n wifi config set reload|
*  |<br><b>AP Port :WPA-PSK(TKIP)\n AP Client Port:WPA2-PSK(AES)</b></br>|<br><b>The remote AP Router is in WPA2-PSK(AES) mode, 40MHz, channel 6:</b></br>\n wifi config set opmode 3\n wifi config set ssid 0 MTK_AP\n wifi config set psk 0 12345678\n wifi config set ssid 1 MTK_SOFT_AP\n wifi config set sec 1 4 4\n wifi config set psk 1 12345678\n wifi config set ch 1 6\n wifi config set bw 1 1\n wifi config set reload |
*  |<br><b>AP Port :WEP(OPEN)(64bit)\n AP Client Port: WPA-PSK(TKIP)</b></br>|<br><b>The remote AP Router is in WPA-PSK(TKIP) mode, 40MHz, channel 6:</b></br>\n wifi config set opmode 3\n wifi config set ssid 0 MTK_AP\n wifi config set psk 0 12345678\n wifi config set ssid 1 MTK_SOFT_AP\n wifi config set sec 1 0 0\n wifi config set wep 1 0 12345(use the first key index,the key index start from 0)\n wifi config set ch 1 6\n wifi config set bw 1 1\n wifi config set reload |
*  |<br><b>Switch Operation Mode</b></br>| |
*  |<br><b>Switch to Station Mode\n Note:\n </b></br> opmode value:\n STA mode:1\n SoftAP mode:2\n Repeater mode:3|wifi config get opmode (get current opmode)\n wifi config set opmode 1(switch to station mode)\n wifi config set reload |
*  |<br><b>Switch to SoftAP Mode</b></br> |wifi config get opmode (get current opmode)\n wifi config set opmode 2(switch to SoftAP mode)|
*  |<br><b>WPS Connection by wifi wps cli</b></br>| |
*  |<br><b>MT7687 as STA-Enrollee</b></br>|<br><b>Connect with AP Router by Push Button Connection(PBC)method\n Operation on MT7687 side:</b></br>\n wifi config set opmode 1(set opmode as station mode)\n wifi config set reload\n wifi wps trigger pbc 0 any(triger WPS PBC connection)\n <br><b>Operation on Common AP-Registrar side</b></br>\n Push the PBC button on Common AP-Registrar side |
*  |<br><b>MT7687 as STA-Enrollee</b></br>|<br><b>Connect with AP Router by Personal Identification Number(PIN) method\n Method 1:(do the WPS Connection without BSSID of the Common AP-Registrar )\n Operation on MT7687 side</b></br>\n wifi config set opmode 1 (switch to station mode)\n wifi config set reload\n wifi wps get pin_code 0 (get 8-digital PIN number of station enrollee, such as "34794265")\n wifi wps trigger pin 0 any 34794265 (do wps connection with AP-Registrar, AP-Registrar need input STA's PIN)\n <br><b>Operation on Common AP-Registrar side</b></br>\n 1.Input the STA PIN "34794265" in UI or Web of Common AP-Registrar\n 2.trigger WPS PIN in UI or Web.\n <br><b>Method2:(do the WPS Connection with BSSID of the Common AP-Registrar )(BSSID=00:0C:43:46:46:31)\n Operation on MT7687 side</b></br>\n wifi config set opmode 1(switch to station mode)\n wifi config set reload\n wifi wps get pin_code 0(get 8-digital PIN number of station enrollee, such as "34794265")\n wifi wps trigger pin 0 00:0C:43:46:46:31 34794265(do wps connection with AP-Registrar, AP-Registrar need input STA's PIN)\n <br><b>Operation on Common AP-Registrar side</b></br>\n 1.Input the STA PIN "34794265" in UI or Web of Common AP-Registrar\n 2.trigger WPS PIN in UI or Web page.|
*  |<br><b>MT7687 as AP-Registrar</b></br>|<br><b>Connect with AP Router by Push Button Connection(PBC)method\n Operation on MT7687 side:</b></br>\n wifi config set opmode 2(set opmode as SoftAP mode)\n wifi wps trigger pbc 1 any(triger WPS PBC connection)\n <br><b>Operation on Common STA-Enrollee side</b></br>\n Push the PBC button on Common STA-Enrollee side|
*  |<br><b>MT7687 as AP-Registrar</b></br>|<br><b>Connect with AP Router by Personal Identification Number(PIN) method\n Operation on Common STA-Enrollee side</b></br>\n 1.trigger the WPS PIN connection and we will get STA's 8 ditital PIN, such as "42205597"\n <br><b>Operation on MT7687 side</b></br>\n wifi config set opmode 2(set opmode as SoftAP mode)\n wifi wps trigger pin 1 any 42205597(input STA's PIN "42205597" and triger WPS PIN connection process).|
*
*  | Example  Items                |	      "Config" Cli Example       |
*  |-------------------------------|-------------------------------------|
*  |<br><b>MT7687 Station Mode</b></br>| |
*  |<br><b>AP Router :OPEN </b></br>|<br><b>connect AP with OPEN mode:</b></br>\n config write common OpMode 1 \n config write STA Ssid MTK_AP\n config write STA SsidLen 6 \n config write STA AuthMode 0\n config write STA EncrypType 1\n reboot |
*  |<br><b>AP Router :WPA2-PSK(AES)</b></br>|<br><b>connect AP with WPA2-PSK(AES)mode, password is 12345678:</b></br>\n config write common OpMode 1\n config write STA Ssid MTK_AP \n config write STA SsidLen 6 \n config write STA AuthMode 7\n config write STA EncrypType 6\n config write STA WpaPsk 12345678 \n config write STA WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WPA2-PSK(TKIP)</b></br> |<br><b>connect AP with WPA2-PSK(TKIP) mode:</b></br>\n config write common OpMode 1 1\n config write STA Ssid MTK_AP \n config write STA SsidLen 6 \n config write STA AuthMode 7\n config write STA EncrypType 4\n config write STA WpaPsk 12345678 \n config write STA WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WPA-PSK(TKIP)</b></br> |<br><b>connect AP with WPA-PSK(TKIP) mode,password is 12345678:</b></br>\n config write common OpMode 1 \n config write STA Ssid MTK_AP\n config write STA SsidLen 6\n config write STA SsidLen 6 \n config write STA AuthMode 4\n config write STA EncrypType 4\n config write STA WpaPsk 12345678 \n config write STA WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WEP(OPEN)(64bit)</b></br> |<br><b>connect AP with WEP OPEN(64bit) mode,WEP key is 12345:</b></br>\n config write common OpMode 1\n config write STA Ssid MTK_AP\n config write STA SsidLen 6\n config write STA AuthMode 0\n config write STA EncrypType 0\n config write STA SharedKey 12345,0,0,0\n config write STA SharedKeyLen 5,0,0,0\n config write STA DefaultKeyId 0(use the first key index,the key index start from 0)\n reboot |
*  |<br><b>MT7687 SoftAP Mode</b></br>| |
*  |<br><b>AP Router :OPEN </b></br>|<br><b>connect AP with OPEN mode:</b></br>\n config write common OpMode 1 \n config write AP Ssid MTK_AP\n config write AP SsidLen 6 \n config write AP AuthMode 0\n config write AP EncrypType 1\n config write AP BW 1\n reboot |
*  |<br><b>AP Router :WPA2-PSK(AES)</b></br>|<br><b>connect AP with WPA2-PSK(AES)mode, password is 12345678:</b></br>\n config write common OpMode 1\n config write AP Ssid MTK_AP \n config write AP SsidLen 6 \n config write AP AuthMode 7\n config write AP EncrypType 6\n config write AP WpaPsk 12345678 \n config write AP WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WPA2-PSK(TKIP)</b></br> |<br><b>connect AP with WPA2-PSK(TKIP) mode:</b></br>\n config write common OpMode 1 1\n config write AP Ssid MTK_AP \n config write AP SsidLen 6 \n config write AP AuthMode 7\n config write AP EncrypType 4\n config write AP WpaPsk 12345678 \n config write AP WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WPA-PSK(TKIP)</b></br> |<br><b>connect AP with WPA-PSK(TKIP) mode,password is 12345678:</b></br>\n config write common OpMode 1 \n config write AP Ssid MTK_AP\n config write AP SsidLen 6\n config write AP SsidLen 6 \n config write AP AuthMode 4\n config write AP EncrypType 4\n config write AP WpaPsk 12345678 \n config write AP WpaPskLen 8 \n reboot |
*  |<br><b>AP Router :WEP(OPEN)(64bit)</b></br> |<br><b>connect AP with WEP OPEN(64bit) mode,WEP key is 12345:</b></br>\n config write common OpMode 1\n config write AP Ssid MTK_AP\n config write AP SsidLen 6\n config write AP AuthMode 0\n config write AP EncrypType 0\n config write AP SharedKey 12345,0,0,0\n config write AP SharedKeyLen 5,0,0,0\n config write AP DefaultKeyId 0(use the first key index,the key index start from 0)\n reboot |
*  |<br><b>Repeater Mode</b></br>| |
*  |<br><b>AP Port :Open Mode\n AP Client Port:Open Mode</b></br>|<br><b>The remote AP Router is in OPEN mode, 40MHz, channel 6:</b></br>\n config write common OpMode 3\n config write STA Ssid MTK_AP\n config write STA SsidLen 6\n config write STA AuthMode 0\n config write STA EncrypType 1\n config write STA Channel 6\n config write STA BW 1\n config write AP Ssid MTK_SOFT_AP\n config write AP SsidLen 11\n config write AP AuthMode 0\n config write AP EncrypType 1\n config write AP Channel 6\n config write AP BW 1\n reboot|
*  |<br><b>AP Port :WPA-PSK(TKIP)\n AP Client Port:WPA2-PSK(AES)</b></br>|<br><b>The remote AP Router is in WPA2-PSK(AES) mode, 40MHz, channel 6:</b></br>\n config write common OpMode 3\n config write STA Ssid MTK_AP\n config write STA SsidLen 6\n config write STA AuthMode 7\n config write STA EncrypType 6\n config write STA WpaPskLen 8\n config write STA WpaPsk 12345678\n config write STA Channel 6\n config write STA BW 1\n config write AP Ssid MTK_SOFT_AP\n config write AP SsidLen 11\n config write AP AuthMode 4\n config write AP EncrypType 4\n config write AP WpaPskLen 8\n config write AP WpaPsk 12345678\n config write AP Channel 6\n config write AP BW 1\n reboot|
*  |<br><b>AP Port :WEP(OPEN)(64bit)\n AP Client Port: WPA-PSK(TKIP)</b></br>|<br><b>The remote AP Router is in WPA-PSK(TKIP) mode, 40MHz, channel 6:</b></br>\n config write common OpMode 3\n config write STA Ssid MTK_AP\n config write STA SsidLen 6\n config write STA AuthMode 4\n config write STA EncrypType 4\n config write STA WpaPskLen 8\n config write STA WpaPsk 12345678\n config write STA Channel 6\n config write STA BW 1\n config write AP Ssid MTK_SOFT_AP\n config write AP SsidLen 11\n config write AP AuthMode 0\n config write AP EncrypType 0\n config write AP SharedKey 12345,0,0,0\n config write AP SharedKeyLen 5,0,0,0\n config write AP DefaultKeyId 0\n config write AP Channel 6\n config write AP BW 1\n reboot |
*  |<br><b>Switch Operation Mode</b></br>| |
*  |<br><b>Switch to Station Mode\n Note:\n </b></br> opmode value:\n STA mode:1\n SoftAP mode:2\n Repeater mode:3|config read  common OpMode (get current opmode)\n config write common OpMode 1(switch to station mode)\n reboot |

* @section WiFiCLI_Table1_Chapter Combinations of Auth Mode and Encrypt Type
*  | Combination Type |  Auth  Mode | Encrypt  Type |
*  |------------------|-------------|---------------|
*  |WPA2PSK(AES)|7|6|
*  |WPA2PSK(TKIP)|7	|4|
*  |WPA2PSK(AES+TKIP)|7	|8|
*  |WPAPSK(AES)|4	|6|
*  |WPAPSK(TKIP)|4	|4|
*  |WPAPSK+WPA2PSK(AES+TKIP)|9	|8|
*  |WEP(OPEN)|0	|0|
*
*
* @section WiFiCLI_Table2_Chapter Country region and Channel range of 2.4G band
*  |    Band |  2.4G |     |      |      |      |      |      |      |
*  |---------|-------|-----|------|------|------|------|------|------|
*  |\b Region| 0|	1|	2|	3|	4|	5|	6|	7|
*  |\b Channel|CH1-11|CH1-13|CH10-11|CH10-13|CH14|CH1-14|CH3-9|CH5-13|
*
*/
#ifndef __WIFI_CLI_H__
#define __WIFI_CLI_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "cli.h"

/**
* @brief Example of Set WiFi Raw Packet Receiver
* <br><b>
* wifi config set rxraw <enable>
* </b></br>
* @param [IN]enable 0: unregister, 1: register
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_rx_raw_pkt_ex(uint8_t len, char *param[]);



/**
* @brief send txraw packets
* <br><b>
* wifi config set txraw <raw_packets>
* </b></br>
* @param  [IN] packets The 802.11 packets content that includes 802.11 MAC header + Payload, it does not include the FCS.
*
* @return  =0 means success, >0 means fail
*/
uint8_t wifi_tx_raw_pkt_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get MAC address for STA/AP wireless port
* <br><b>
* wifi config get mac <port> --> get port0 (STA) MAC address
* </b></br>
* @param [IN]port  0: STA/APCLI, 1: AP
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_mac_address_ex(uint8_t len, char *param[]);
/**
* @brief Example of Configure bandwidth for STA/AP  wireless port.
* <br><b>
* wifi config set bw <port> <bandwidth>
* </b></br>
* @param [IN]port  0: STA/APCLI, 1: AP
* @param [IN]bw   0: HT20, 1: HT40, 2: HT20/40 coexistence
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_bandwidth_ex(uint8_t len, char *param[]);
/**
* @brief Example of get bandwidth for STA/AP wireless port.
* <br><b>
* wifi config get bw <port>
* </b></br>
* @param [IN]port  0: STA/APCLI, 1: AP
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_bandwidth_ex(uint8_t len, char *param[]);

/**
* @brief Example of Configure bandwidth for STA/AP  wireless port.
*
* wifi config set bw_extended <port>  <0:HT20/1:HT40> <below_above_ch>
* @parameter
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_bandwidth_extended_ex(uint8_t len, char *param[]);

/**
* @brief Example of get bandwidth for STA/AP wireless port.
*  wifi config get bw_extended <port>
* @parameter
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_bandwidth_extended_ex(uint8_t len, char *param[]);

/**
* @brief Example of Configure channel for STA/AP wireless port. STA will keep idle and stay in channel specified
* <br><b>
* wifi config set ch <port> <ch>
* </b></br>
* @param [IN]port  0: STA/APCLI, 1: AP
* @param [IN]ch  1~14 are supported for 2.4G only product
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_channel_ex(uint8_t len, char *param[]);
/**
* @brief Example of get the current channel for STA/AP wireless port.
* <br><b>
* wifi config get ch <port>
* </b></br>
* @param [IN]port  0: STA/APCLI, 1: AP
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_channel_ex(uint8_t len, char *param[]);
/**
* @brief Example of configure rx filter for packets wanted to be received
* <br><b>
* wifi config set rxfilter <flag>
* </b></br>
* @param [IN]flag defined in  wifi_rx_filter_t
* @return >=0 means success, <0 means fail
* @note Default value will be WIFI_DEFAULT_IOT_RX_FILTER
*/
uint8_t wifi_config_set_rx_filter_ex(uint8_t len, char *param[]);
/**
* @brief Example of get rx filter for packets format wanted to be received
* <br><b>
* wifi config get rxfilter
* </b></br>
* @param [OUT]flag defined in  wifi_rx_filter_t
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_rx_filter_ex(uint8_t len, char *param[]);
/**
* @brief Example of set the authentication mode and encryption mode for the specified STA/AP port
* <br><b>
* wifi config set sec <port> <auth> <encrypt>
* </b></br>
* @param [IN]authmode
*           1 WPAPSK
*           2 WPA2PSK
*           3 WPA1PSKWPA2PSK
* @param  [IN] encryption method index:
*              1 AES
*              2 TKIP
*              3 TKIPAES
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_set_security_mode_ex(uint8_t len, char *param[]);
/**
* @brief Example of set the psk for the specified STA/AP port
* <br><b>
* wifi config set psk <port> <password>
* </b></br>
@param [IN]passphrase 8 ~ 63 bytes ASCII or 64 bytes Hex
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_set_psk_ex(uint8_t len, char *param[]);

#if 0 //Add new cli to set authentication mode and psk separately.
/**
* @brief Example of Set the authentication mode for the specified STA/AP port
* <br><b>
* wifi config set psk <port> <auth> <encrypt> <passphrase>
* </b></br>
*
* @param [IN]port  0: STA/APCLI, 1: AP
* @param [IN]auth authentication mode
*           1: WPAPSK,
*           2: WPA2PSK,
*           3: WPA1PSKWPA2PSK
* @param  [IN]encrypt encryption method index:
*           1: AES,
*           2: TKIP,
*           3: TKIPAES
* @param [IN]passphrase 8 ~ 63 bytes ASCII or 64 bytes Hex
* @return >=0 means success, <0 means fail
* @note Default to OPEN
*/
uint8_t wifi_config_set_psk_ex(uint8_t len, char *param[]);
#endif

/**
* @brief Example of Set PMK for the specified STA/AP port
* <br><b>
* wifi config set pmk <port> <PMK>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [IN]PMK (in hex)
*       00, 05, 30, ......(size 32)
* @return >=0 means success, <0 means fail
* @note Default to OPEN
*/
uint8_t wifi_config_set_pmk_ex(uint8_t len, char *param[]);

#if 0 //Add new cli to get authentication mode and psk separately.
/**
* @brief Example of Get the authentication mode for the specified STA/AP port
* <br><b>
* wifi config get psk <port>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [OUT]authmode
*           1: WPAPSK,
*           2: WPA2PSK,
*           3: WPA1PSKWPA2PSK
* @param  [OUT] encryption method index:
*           1: AES,
*           2: TKIP,
*           3: TKIPAES
* @param [OUT]passphrase 8 ~ 63 bytes ASCII or 64 bytes Hex
* @param [OUT]passphrase_len 8 ~ 64
* @return >=0 means success, <0 means fail
* @note Default to OPEN
*/
uint8_t wifi_config_get_psk_ex(uint8_t len, char *param[]);
#endif

/**
* @brief Example of get the authentication mode for the specified STA/AP port
* <br><b>
* wifi config get sec <port>
* </b></br>
* @param [OUT]authmode method index:
*           1 WPAPSK
*           2 WPA2PSK
*           3 WPA1PSKWPA2PSK
* @param [OUT] encryption method index:
*              1 AES
*              2 TKIP
*              3 TKIPAES
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_get_security_mode_ex(uint8_t len, char *param[]);

/**
* @brief Example of get the psk for the specified STA/AP port
* <br><b>
* wifi config get psk <port> <password>
* </b></br>
@param [IN]passphrase 8 ~ 63 bytes ASCII or 64 bytes Hex
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_get_psk_ex(uint8_t len, char *param[]);


/**
* @brief Example of Get PMK for the specified STA/AP port
* <br><b>
* wifi config get pmk <port>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [OUT]PMK (in hex)
*       00, 05, 30, ......(size 32)
* @return >=0 means success, <0 means fail
* @note Default to OPEN
*/
uint8_t wifi_config_get_pmk_ex(uint8_t len, char *param[]);
/**
* @brief Example of get WiFi WEP Keys
* <br><b>
* wifi config get wep <port>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [OUT]wifi_wep_key_t
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_wep_key_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi WEP Keys
* <br><b>
* wifi config set wep <port> <key_id> <key_string>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [IN]key_id 0~3
* @param [IN]key_string 0~26 (ASCII length: 5 or 13; Hex length:10 or 26)
* @return >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_wep_key_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get WiFi Operation Mode.
* <br><b>
* wifi config get opmode
* </b></br>
* @param [OUT]mode
*        1: WIFI_MODE_STA_ONLY,
*        2: WIFI_MODE_AP_ONLY,
*        3: WIFI_MODE_REPEATER
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_opmode_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi Operation Mode.
* <br><b>
* wifi config set opmode <mode>
* </b></br>
* @param [IN]mode
*       1: WIFI_MODE_STA_ONLY,
*       2: WIFI_MODE_AP_ONLY,
*       3: WIFI_MODE_REPEATER
* @return  >=0 means success, <0 means fail
* @note Set WiFi Operation Mode will RESET all the configuration set by previous WIFI-CONFIG APIs
*/
uint8_t wifi_config_set_opmode_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get WiFi SSID.
* <br><b>
* wifi config get ssid <port>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [OUT]ssid SSID
* @param [OUT]ssid_len Length of SSID
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_ssid_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi SSID.
* <br><b>
* wifi config set ssid <port> <ssid>
* </b></br>
* @param [IN]port  0: STA/APCLI / AP Client, 1: AP
* @param [IN]ssid SSID
* @param [IN]ssid_len Length of SSID
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_ssid_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get WiFi SSID.
* <br><b>
* wifi config get bssid
* </b></br>
* @param [OUT]bssid BSSID
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_bssid_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi SSID.
* <br><b>
* wifi config set ssid <bssid>
* </b></br>
* @param [IN]bssid BSSID
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_bssid_ex(uint8_t len, char *param[]);

/**
* @brief Example of Reload configuration
* <br><b>
* wifi config set reload
* </b></br>
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_reload_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get MTK Smart Connection Filter
* <br><b>
* wifi config get mtk_smart
* </b></br>
* @param [OUT]flag
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_mtk_smartconnection_filter_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set MTK Smart Connection Filter
* <br><b>
* wifi config set mtk_smart <flag>
* </b></br>
* @param [IN]flag  0: Disable, 1: Enable
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_mtk_smartconnection_filter_ex(uint8_t len, char *param[]);

/**
* @brief Example of Set WiFi Radio ON/OFF
* <br><b>
* wifi config set radio <onoff>
* </b></br>
* @param [IN]onoff  0: OFF, 1: ON
* @return  >=0 means success, <0 means fail
* @note in MODE_Dual, both WiFi interface radio will be turn on/off at the same time
*/
uint8_t wifi_config_set_radio_on_ex(uint8_t len, char *param[]);
/**
* @brief Example of get WiFi Radio ON/OFF
* <br><b>
* wifi config get radio
* </b></br>
* @param [OUT]onoff 0: OFF, 1: ON
* @return  >=0 means success, <0 means fail
* @note in MODE_Dual, both WiFi interface radio will be turn on/off at the same time
*/
uint8_t wifi_config_get_radio_on_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get WiFi Wireless Mode
* <br><b>
* wifi config get wirelessmode
* </b></br>
* @param [OUT] wirelessmode
*        1: legacy 11B only,
*        2: legacy 11A only,
*        3: legacy 11A/B/G mixed,
*        4: legacy 11G only,
*        5: 11ABGN mixed,
*        6: 11N only in 2.4G,
*        7: 11GN mixed,
*        8: 11AN mixed,
*        9: 11BGN mixed,
*       10: 11AGN mixed
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_wireless_mode_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi Wireless Mode
* <br><b>
* wifi config set wirelessmode <mode>
* </b></br>
* @param [IN]mode
*        1: legacy 11B only,
*        2: legacy 11A only,
*        3: legacy 11A/B/G mixed,
*        4: legacy 11G only,
*        5: 11ABGN mixed,
*        6: 11N only in 2.4G,
*        7: 11GN mixed,
*        8: 11AN mixed,
*        9: 11BGN mixed,
*       10: 11AGN mixed
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_wireless_mode_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get WiFi Country Region
* <br><b>
* wifi config get country <band>
* </b></br>
* @param [IN]band  0: 2.4G, 1: 5G
* @param [OUT]region
* <br>
*        region: (2.4G)
* </br>
*        0:  CH1-11,
*        1:  CH1-13,
*        2:  CH10-11,
*        3:  CH10-13,
*        4:  CH14,
*        5:  CH1-14 all active scan,
*        6:  CH3-9,
*        7:  CH5-13
* </br>
* <br>
*        region: (5G)
* </br>
* <br>
*        0:   CH36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165
* </br>
* <br>
*        1:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140
* </br>
* <br>
*        2:   CH36, 40, 44, 48, 52, 56, 60, 64
* </br>
* <br>
*        3:   CH52, 56, 60, 64, 149, 153, 157, 161
* </br>
* <br>
*        4:   CH149, 153, 157, 161, 165
* </br>
* <br>
*        5:   CH149, 153, 157, 161
* </br>
* <br>
*        6:   CH36, 40, 44, 48
* </br>
* <br>
*        7:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165
* </br>
* <br>
*        8:   CH52, 56, 60, 64
* </br>
* <br>
*        9:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165
* </br>
* <br>
*        10:  CH36, 40, 44, 48, 149, 153, 157, 161, 165
* </br>
* <br>
*        11:  CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 149, 153, 157, 161
* </br>

* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_country_region_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi Country Region
* <br><b>
* wifi config set country <band> <region>
* </b></br>
* @param [IN]band  0: 2.4G, 1: 5G
* @param [IN]region
* <br>
*        region: (2.4G)
* </br>
*        0:  CH1-11,
*        1:  CH1-13,
*        2:  CH10-11,
*        3:  CH10-13,
*        4:  CH14,
*        5:  CH1-14 all active scan,
*        6:  CH3-9,
*        7:  CH5-13
* </br>
* <br>
*        region: (5G)
* </br>
* <br>
*        0:   CH36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165
* </br>
* <br>
*        1:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140
* </br>
* <br>
*        2:   CH36, 40, 44, 48, 52, 56, 60, 64
* </br>
* <br>
*        3:   CH52, 56, 60, 64, 149, 153, 157, 161
* </br>
* <br>
*        4:   CH149, 153, 157, 161, 165
* </br>
* <br>
*        5:   CH149, 153, 157, 161
* </br>
* <br>
*        6:   CH36, 40, 44, 48
* </br>
* <br>
*        7:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165
* </br>
* <br>
*        8:   CH52, 56, 60, 64
* </br>
* <br>
*        9:   CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165
* </br>
* <br>
*        10:  CH36, 40, 44, 48, 149, 153, 157, 161, 165
* </br>
* <br>
*        11:  CH36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 149, 153, 157, 161
* </br>
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_country_region_ex(uint8_t len, char *param[]);

/**
* @brief Example of Set Country Code
* wifi config set country_code <code>
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_set_country_code_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get Country Code
* wifi config get country_code
*
* @return =0 means success, >0 means fail
*/
uint8_t wifi_config_get_country_code_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get WiFi DTIM Interval
* <br><b>
* wifi config get dtim
* </b></br>
* @param  [OUT]interval: 1~255
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_dtim_interval_ex(uint8_t len, char *param[]);

/**
* @brief Example of Set WiFi DTIM Interval
* <br><b>
* wifi config set dtim <interval>
* </b></br>
* @param [IN]interval 1 ~ 255
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_dtim_interval_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get WiFi Listen Interval
* <br><b>
* wifi config get listen
* </b></br>
* @param  [OUT]interval: 1~255
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_get_listen_interval_ex(uint8_t len, char *param[]);
/**
* @brief Example of Set WiFi Listen Interval
* <br><b>
* wifi config set listen <listen interval>
* </b></br>
* @param [IN]interval 1 ~ 255
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_config_set_listen_interval_ex(uint8_t len, char *param[]);



/* ---------------------- Connect Ex ----------------------------*/




/**
* @brief Example of  connect / link up to specifed AP
* <br><b>
* wifi connect set connection <linkup/down>
* </b></br>
* @param [IN] link  0: link down, 1: link up
* @return >=0 means success, <0 means fail
* @note Be sure to configure security setting before connect to AP
*/
uint8_t wifi_connect_set_connection_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get the current STA port link up / link down status of the connection
* <br><b>
* wifi connect get linkstatus
* </b></br>
* @param [out] link  0: link down, 1: link up
* @return >=0 means success, <0 means fail
* @note WIFI_STATUS_LINK_DISCONNECTED indicates STA may in IDLE/ SCAN/ CONNECTING state
*/
uint8_t wifi_connect_get_link_status_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get WiFi Associated Station List
* <br><b>
* wifi connect get stalist
* </b></br>
* @param [OUT]station_list
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_connect_get_station_list_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get the max number of the supported stations in ap mode or Repeater Mode.
* wifi connect get max_num_sta
* @param [OUT] number The max number of supported stations will be returned
*
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_connect_get_max_station_number_ex(uint8_t len, char *param[]);

/**
* @brief Example of deauth some WiFi connection
* <br><b>
* wifi connect set deauth <MAC>
* </b></br>
* @param [IN]addr STA MAC Address
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_connect_deauth_station_ex(uint8_t len, char *param[]);
/**
* @brief Example of Get rssi of the connected AP
* <br><b>
* wifi connect get rssi
* </b></br>
* @param [OUT]rssi
* @return >=0 means success, the status will be:
*/
uint8_t wifi_connect_get_rssi_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get inband queue spaces
* <br><b>
* wifi mq get iq
* </b></br>
* @param [OUT]inband queue spaces
* @return >=0 means success, the status will be:
*/
uint8_t wifi_mq_get_iq_ex(uint8_t len, char *param[]);

/**
* @brief Example of Register WiFi Event Notifier
* <br><b>
* wifi connect set eventcb <enable> <event_id>
* </b></br>
* @param [IN]enable 0: unregister, 1: register
* @param [IN]event_id Event ID
*        LinkUP(0)
*        SCAN COMPLETE(1)
*        DISCONNECT(2)
*        PORT_SECURE(3)
*        REPORT_BEACON_PROBE_RESPONSE(4)
* @return  >=0 means success, <0 means fail
*/
uint8_t wifi_connect_set_event_callback_ex(uint8_t len, char *param[]);

uint8_t wifi_connect_set_scan_ex(uint8_t len, char *param[]);

/*WIFI CLI ENTRY definiation*/

#define WIFI_PUB_CLI_ENTRY          { "wifi",    "wifi api",     NULL,   wifi_pub_cli },

#if defined(MTK_SMTCN_V5_ENABLE) || defined(MTK_SMTCN_V4_ENABLE)
#define WIFI_PUB_CLI_SMNT_ENTRY     { "smart",   "smart connection", _smart_config_test_cli },
#else
#define WIFI_PUB_CLI_SMNT_ENTRY
#endif


/*extern function*/
#if defined(MTK_SMTCN_V5_ENABLE) || defined(MTK_SMTCN_V4_ENABLE)

/**
* @brief Example of smart connect config
* <br><b>
* smart <cmd> [option]
* smart connect <bm/m/b>   smart connect use both mc and bc, mc, bc 
* smart stop                           smart connect stop 
* smart debug   <0/1>           smart connect debug 0/1
* </b></br>
* @param [IN] cmd
* @param [IN] option
* @return  =0 means success, >0 means fail
*/
extern uint8_t _smart_config_test_cli(uint8_t len, char *param[]);
#endif
extern cmd_t   wifi_pub_cli[];
extern cmd_t   wifi_mq_get_cli[];


#ifdef MTK_WIFI_PROFILE_ENABLE
extern cmd_t   wifi_profile_set_cli[];
extern cmd_t   wifi_profile_get_cli[];
uint8_t wifi_profile_reset_ex(uint8_t len, char *param[]);
#endif


#if defined(MTK_WIFI_WPS_ENABLE)

/**
* @brief Example of Set default WPS device information, config default value inside the function
* <br><b>
* wifi wps set device_info <Device_Name> <Manufacturer> <Model_Name> <Model_number> <Serial_number>
* </b></br>
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_set_device_info_ex(uint8_t len, char *param[]);

/**
* @brief Example of Set auto connection when Enrollee got WPS credential information from the target AP.
* <br><b>
* wifi wps set auto connection <onoff>
* </b></br>
* @param [IN]onoff on: enable auto connection to target AP after WPS done,   off: disable auto connection.
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_set_auto_connection_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get WPS device information
* <br><b>
* wifi wps get device_info
* </b></br>
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_get_device_info_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get WPS done auto connection control status.
* <br><b>
* wifi wps get auto connection
* </b></br>
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_get_auto_connection_ex(uint8_t len, char *param[]);

/**
* @brief Example of Get 8-digital PIN code of AP or STA.
* <br><b>
* wifi wps get pin <port>
* </b></br>
* @param [IN]port  0: STA/APCLI,   1:AP.
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_get_pin_code_ex(uint8_t len, char *param[]);

/**
* @brief Example of trigger WPS PBC.
* <br><b>
* wifi wps trigger pbc <port> <BSSID>
* </b></br>
* @param [IN]port  0: STA/APCLI,   1:AP.
* @param [IN]BSSID any: no special BSSID,   xx:xx:xx:xx:xx:xx :special BSSID.
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_connection_by_pbc_ex(uint8_t len, char *param[]);


/**
* @brief Example of trigger WPS PIN.
* <br><b>
* wifi wps trigger pin <port> <BSSID> <PIN_code>
* </b></br>
* @param [IN]port  0: STA/APCLI,   1:AP.
* @param [IN]BSSID any: no special BSSID,   xx:xx:xx:xx:xx:xx :special BSSID.
* @param [IN]PIN_code:   8-digital PIN code.(for example: PIN code can be got from wifi_wps_connection_by_pin_ex())
* @return =0 means success, else means fail
*/
uint8_t wifi_wps_connection_by_pin_ex(uint8_t len, char *param[]);

#endif/*MTK_WIFI_WPS_ENABLE*/


#ifdef MTK_WIFI_MIB_ENABLE
/**
 * @brief Show Wi-Fi MIB counters.
 * <br><b>
 * wifi mib
 * </b></br>
 * @return 0 means success, else means fail
 */
uint8_t wifi_mib_show(uint8_t len, char *param[]);
#endif //MTK_WIFI_MIB_ENABLE


#if (PRODUCT_VERSION == 5932)
#define MTK_CLI_SLIM_FOR_32
#endif

#ifdef __cplusplus
}
#endif

/**
*@}
*/
#endif /*  __WIFI_CLI_H__ */
