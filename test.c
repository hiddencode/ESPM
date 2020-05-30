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
	// Init NVS and check to error
	ret = nvs_flash_init();
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

	// App registers
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


/* Entry point of project */
void app_main()
{
	esp_err_t ret;
	initialize(ret);

}
