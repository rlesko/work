
#include <nordic_common.h>
#include <app_util.h>

#include "ble_sam.h"

static uint32_t 
accel_char_add(ble_sam_t * p_sam, const ble_sam_init_t * p_sam_init)
{
	  ble_gatts_char_md_t chm;
		ble_gatts_attr_md_t ccm;
	  ble_gatts_attr_t acv;
	  ble_gatts_attr_md_t atm;
  	ble_uuid_t id;
	
	  zes_lis3dsh_d initial_data;
		initial_data.x_G = 1.0;
	  initial_data.y_G = 1.0;
	  initial_data.z_G = 1.0;
	
		CLEAR_OBJECT(&ccm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ccm.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ccm.write_perm);
	  ccm.vloc = BLE_GATTS_VLOC_STACK;
	
		CLEAR_OBJECT(&chm);
	  chm.char_props.read = 1;
	  chm.char_props.notify = 1;
	  chm.p_char_user_desc = NULL;
	  chm.p_char_pf = NULL;
	  chm.p_user_desc_md = NULL;
	  chm.p_cccd_md = &ccm;
  	chm.p_sccd_md = NULL;
	
	  id.type = p_sam->uuid_type;
		id.uuid = ZES_UUID_ACCEL_CHAR;
	
	  CLEAR_OBJECT(&atm);
//		atm.read_perm = p_sam_init->accel_char_attr_md.read_perm;
//		atm.write_perm = p_sam_init->accel_char_attr_md.write_perm;
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&atm.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&atm.write_perm);
		atm.vloc = BLE_GATTS_VLOC_STACK;
		atm.rd_auth = 0;
		atm.wr_auth = 0;
		atm.vlen = 0;
	
	  CLEAR_OBJECT(&acv);
		acv.p_uuid = &id;
		acv.p_attr_md = &atm;
		acv.init_len = sizeof(zes_lis3dsh_d);
		acv.max_len = sizeof(zes_lis3dsh_d);
		acv.init_offs = 0;
		acv.p_value = NULL;

//	  id.type = p_sam->uuid_type;
//	  id.uuid = ZES_UUID_ACCEL_CHAR;
	
	  return sd_ble_gatts_characteristic_add(p_sam->srvc_handle, &chm, &acv, &p_sam->accel_char_handles);
}

uint32_t ble_sam_init(ble_sam_t * p_sam, const ble_sam_init_t * p_sam_init)
{
		uint32_t ec;
		ble_uuid_t id;

		// Create service to advertise
	  ble_uuid128_t base_uuid = ZENDO_UUID_BASE;
	  ec = sd_ble_uuid_vs_add(&base_uuid, &p_sam->uuid_type);
	  if ( ec != NRF_SUCCESS)
			return ec;
	
	  p_sam->conn_handle = BLE_CONN_HANDLE_INVALID;
	  
	  id.type = p_sam->uuid_type;
	  id.uuid = ZES_UUID_ACCEL_SERVICE;
	
	  ec = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &id, &p_sam->srvc_handle);
	 	if (ec != NRF_SUCCESS)
		  return ec;
	
		ec = accel_char_add(p_sam, p_sam_init);
	  if (ec != NRF_SUCCESS)
		  return ec;
    
		return NRF_SUCCESS;
}

uint32_t ble_sam_measurement_send(ble_sam_t * p_sam, zes_lis3dsh_d * data)
{
	  uint32_t ec;
	  if (p_sam->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			  uint16_t hvx_len = sizeof(zes_lis3dsh_d);
			  ble_gatts_hvx_params_t hvx_params;
			
			  hvx_params.handle = p_sam->accel_char_handles.value_handle;
			  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
			  hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = (uint8_t *)data;
			
			  ec = sd_ble_gatts_hvx(p_sam->conn_handle, &hvx_params);
			
			  if (ec != NRF_SUCCESS)
        {
            ec = NRF_ERROR_DATA_SIZE;
        }  
		}
		else
		{
			  ec = NRF_ERROR_INVALID_STATE;
		}
		
		return ec;
}

void ble_sam_on_ble_evt(ble_sam_t * p_sam, ble_evt_t * p_ble_evt)
{
	
}

uint32_t ble_sam_on_accelerometer_update(ble_sam_t * p_sam, zes_lis3dsh_d *d)
{
	  uint32_t ec;
	  uint16_t len;
	  ble_gatts_hvx_params_t hvx_params;
	  zes_lis3dsh_d data;
	  len = sizeof(zes_lis3dsh_d);
	
	  data.x_G = 0xFF;
	  data.y_G = 0xFF;
	  data.z_G = 0xFF;
	
	  CLEAR_OBJECT(&hvx_params);
	  hvx_params.handle = p_sam->accel_char_handles.value_handle;
	  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
	  hvx_params.offset = 0;
	  hvx_params.p_len = &len;
	  hvx_params.p_data = (uint8_t *)&data;
	
	  ec = sd_ble_gatts_hvx(p_sam->conn_handle, &hvx_params);
}