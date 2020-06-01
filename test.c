/* Standart includes*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Includes for RTOS*/
#include "freertos/FreeRTOS.h"		// For initial RTOS
#include "freertos/task.h"		// For tasks from RTOS
#include "freertos/event_groups.h" 	// For event from RTOS
#include "esp_system.h"			// ESP init
#include "nvs_flash.h"			// Non-volatile storage flash 
/* Bluetooth/BLE interaction */
#include "bt.h"				// BT controller and VHCI config
#include "bta_api.h"			// BT controller API
#include "esp_gap_ble_api.h"		// API for BLE-GAP
#include "esp_gatts_api.h"		// API for BLE-GATT
#include "esp_bt_defs.h"		// Defines for BT controller
#include "esp_bt_main.h"		// BLE API
/* project configuration */
#include "sdkconfig.h"


/**
 *	Initialize configuration
 *	for interaction with BLE
 *	@param ret 	- ESP error handler
 */
void initialize(esp_err_t ret)
{
	// Init Non-Volatile Storage lib
	ret = nvs_flash_init();
	// Check flash to error	
	if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Init BT controller and check to error
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if(ret){
		ESP_LOGE(GATTS_TAG, "%s intialize controller failed\n", __func__);
		return;
	}

	// Common API
	// Init Bluedroid stack and check to error
	ret = esp_bluedroid_init();
	if(ret){
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
		return;
	}
	
	ret = esp_bluedroid_enable();
	if(ret){
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed", __func__);
		return;
	}

	// Init GAP and GATT config
	// GATT register
	ret  = esp_ble_gatts_register_callback(gatts_event_handler);
	if(ret){
		ESP_LOGE(GATTS_TAG, "gatts register error, error code  %x", ret);
		return;
	}
	// GAP register
	ret = esp_ble_gap_register_callback(gap_event_handler);
	if(ret){
		ESP_LOGE(GATTS_TAG, "gap register error, error code %x", ret);
		return;
	}

	// Registers Apllicttion using Applications ID
	ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
	if(ret){
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
	if(ret){
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	// Local MTU
	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
	if(local_mtu_ret){
		ESP_LOGE(GATTS_TAG, "set lcoal MTU failed, error code = %x", local_mtu_ret);
	}
	return;
};

/**
 * Structure of GATT Aplication profile
 * @mem gatts_cb	- GATT profile event handler
 * @mem gatts_if	- GATT interface 
 * @mem app_id		- Application ID
 * @mem conn_id		- Connection ID
 * @mem service_handle	- Service handle
 * @mem service_id	- Service ID
 * @mem char_handle	- Characteristic handle
 * @mem char_uuid	- Characteristic UUID
 * @mem perm		- Attribute permissions
 * @mem property	- Characteristic properties
 * @mem descr_handle	- Client Characteristic Configuration descriptor handle
 * @mem descr_uuid	- Client Characteristic Configuration descriptor UUID
 */
typedef struct gatts_profile_inst{
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
}gatts_profile_inst gatts_profile_inst_t;

/**
 * Array for storing application profiles
 */
static gatts_profile_inst_t gl_profile_tab[PROFILE_NUM] = {
	[PROFILE_A_APP_ID] = {
		.gatts_cb = gatts_profile_a_event_handler,
		.gatts_id = ESP_GATT_IF_NONE, // Aplication profile is not linked to any client yet
	}
	[PROFILE_B_APP_ID] = {
		.gatts_cb = gatts_profile_b_event_handler,
		.gatts_id = ESP_GATT_IF_NONE, // Aplication profile is not linked to any client yet
	}
	
};

/**
 * Structure of Bluetooth specification adverstiment parameters
 * @mem set_scan_rsp		- Set this adversting data as scan reponse or not(true/false)
 * @mem include_name		- Adversting data include device or not (true/false)
 * @mem include_txpower		- Adversting data include TX power or not (true/false)
 *
 * @mem min_interval		- Connection min interval
 * @mem max_interval 		- Connection max interval
 * @mem appearance		- External appearance of device
 *
 * @mem manufacturer_len 	- Manafacturer data length
 * @mem p_manufacturer_data	- Pointer to manufacturer data
 * @mem service_data_len	- Service data length
 * @mem p_service_data		- Pointer to service data
 * @mem service_uuid_len	- Service UUID length
 * @mem p_service_uuid		- Pointer to service UUID 
 * @mem flag			- Adversting flag of discovery mode (more detail BLE_ADV_FLAG)
 */
typedef struct esp_ble_adv_data{
	bool set_scan_rsp;
	bool include_name;
	bool include_txpower;
	
	int min_interval;
	int max_interval;
	int appearance;
	
	uint16_t manufacturer_len;
	uint16_t * p_manufacturer_data;
	
	uint16_t service_data_len;
	uint16_t * p_service_data;
	
	uint16_t service_uuid_len;
	uint16_t * p_service_uuid;
	
	uint16_t flag;
}esp_ble_adv_data_t;

/**
 * Intialize structure of adversting data
 */
static esp_ble_adv_data_t adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = true,
	
	.min_interval = 0x0006,
	.max_interval = 0x0010,
	.appearance = 0x00,
	
	.manufacturer_len = 0, // test data
	.p_manufacturer_data = NULL,
	
	.service_data_len = 0, // test data
	.p_service_data = NULL,
	
	.service_uuid_len = 0, //test data
	.p_service_uuid = test_service_uuid128,
	
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREADR_NOT_SPT),
}

/**
 *  The registring event handler of GATT Profile A
 *  @param event	- External event
 *  @param gatts_if	- GATT profile interface
 *  @param param	- Parameter data
 */
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
					  esp_gatts_if_t gatts_if,
					  esp_ble_gatts_cb_param_t * param)
{
	// TODO: Switch for alone case,
	//	 need refactoring
	//	 or defines another events

	// Event register
	switch(event){
		case ESP_GATTS_REG_EVT:
			ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app+id %d\n", param->reg.status, param->reg.app_id);
			// Init global profile A
			gl_profile_tab[PROFILE_A_APP_ID].service_idis_primary = true;.
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
			gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

			// Set device name
			esp_ble_gap_set_device_name(TEST_DEVICE_NAME);

//Custom config using raw data			
#ifdef CONFIG_SET_RAW_ADV_DATA
			esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
			if(raw_adv_ret){
				ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x", raw_adv_ret);
			}
			adv_config_done |= adv_config_config_flag;
			esp_err_t raw_can_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
			if(raw_scan_ret){
				ESP_LOFE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
			}
			// Set adversitize config by value of scanning flag
			adv_config_done |= scan_rsp_config_flag;

//Default config
#else
			//config adv data
			esp_err_t ret = esp_ble_gap_config_adv_data(&scam_rsp_data);
			if(ret){
				ESP_LOGE(GATTS_TAG, "config adv data failed, error code")
			}
			
			//config scan repsonse
			adv_config_done |= adv_config_config_flag;
			ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
			if(raw_scan_ret){
				ESP_LOFE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", ret);
			}
			// Set adversitize config by value of scanning flag
			adv_config_done |= scan_rsp_config_flag;

#endif			

	}

}

/**
 * The registring GAP event handler 
 * @param event		- External event
 * @param param		- Parameter data
 */
static void gap_event_handler ( esp_gap_ble_cb_event_t event,
				esp_ble_gap_cb_param_t * param)
{
	switch(event){
// Config using raw data		
#ifdef CONFIG_SET_RAW_ADV_DATA
		case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
			adv_config_done &= (~adv_config_flag);
			if (adv_config_done == 0){
				esp_ble_gap_start_advertising(&adv_params);
			}
			break;
		case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
			adv_config_done &= (~scan_rsp_config_flag);
			if (adv_config_done == 0){
				esp_ble_gap_start_adversting(&adv_params);
			}
			break
// Default config
#else		
		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
			adv_config_done &= (~adv_config_flag);
			if (adv_config_done == 0){
				esp_ble_gap_start_adversting(&adv_params)
			}
			break
		case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
			adv_config_done &= (~scan_rsp_config_flag);
			if (adv_config_done == 0){
				esp_ble_gap_start_adversting(&adv_params);
			}
			break;	
#endif
	}
}

/**
 * Structure of parameters for adversting
 * @mem adv_int_min 		- Minimum adversting interval 
 *					#Range: 0x0020 to 0x4000
 *					#Default: N = 0x0800(1.28 s)
 *					#Time = N * 0.625 msec
 *					#Time Range: 20 ms to 10.24 sec
 *
 * @mem adv_int_max		- Maximum adversting interval
 *					#Range: 0x0020 to 0x4000
 *					#Default: N = 0x0800(1.28 s)
 *					#Time = N * 0.625 msec
 *					#Time Range: 20 ms to 10.24 sec
 *
 * @mem adv_type		- Adversting type
 * @mem own_addr_type		- Owner bluetooth device address type
 * @mem peer_addr		- Peer device bluetooth device address
 * @mem peer_addr_type		- Peer device bluetooth device address type
 * @mem channel_map		- Adversting channel map
 * @mem adv_filter_policy	- Adverting filter policy
 */
typedef struct esp_ble_adv_params{
	uint16_t adv_int_min;
	uint16_t adv_int_max;
	
	esp_ble_adv_type_t adv_type;
	esp_ble_addr_type_t own_addr_type;
	esp_bd_addr_t peer_addr;
	esp_ble_addr_type_t peer_addr_type;
	esp_ble_adv_channel_t channel_map;
	esp_ble_adv_filter_t adv_filter_policy;
}esp_ble_adv_params_t;

/**
 * Init struct of parameters for adversting
 * with testing data
 */
static esp_ble_adv_params_t test_adv_params = {
	.adv_int_min		= 0x20,
	.adv_int_max		= 0x40,
	.adv_type		= ADV_TYPE_IND,
	.own_addr_type		= BLE_ADDR_TYPE_PUBLIC,
	//.peer_addr		=
	//,peer_addr_type	=
	.channel_map		= ADV_CHNL_ALL,
	.adv_filter_policy	= ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/**
 * Common GATT profile handler
 * Store the generated interface,
 * correspondig profile event handler
 * @param event 	- External event
 * @param param 	- Pointer to parameters for GATT
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
	       			esp_ble_gatts_cb_param_t * param)
{
	// If event is register event, store tthe gatts_if for each profile
	if(event == ESP_GATTS_REG_EVT){
		if(param->reg,status == ESP_GATT_OK){
			gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
		}else{
			ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x status %d\n", param->reg.app_id, param->reg.status);
			return;
		}
	}

	// If the gatts_if equal to profile A, call profile A cb handler,
	// so here call each profile`s callback
	do{
		int idx;
		for(idx = 0; idx < PROFILE_NUM; idx++){
			if(gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if){
				if(gl_profile_tab[idx].gatts_cb){
					gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}

	}while(0);
}


/* Entry point of project */
void app_main()
{
	esp_err_t ret;
	initialize(ret);

}
