
#include <nordic_common.h>
#include <app_util.h>

#include "ble_sam.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_sam       Sam Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_sam_t * p_sam, ble_evt_t * p_ble_evt)
{
    p_sam->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_sam       SAM Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sam_t * p_sam, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sam->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling write events to the SAM Measurement characteristic.
 *
 * @param[in]   p_sam         Sam Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_sam_cccd_write(ble_sam_t * p_sam, ble_gatts_evt_write_t * p_evt_write)
{
//    if (p_evt_write->len == 2)
//    {
        // CCCD written, update notification state
        if (p_sam->evt_handler != NULL)
        {
            ble_sam_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SAM_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SAM_EVT_NOTIFICATION_DISABLED;
            }

            p_sam->evt_handler(p_sam, &evt);
        }
//    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_sam       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sam_t * p_sam, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_sam->accel_char_handles.cccd_handle)
    {
        on_sam_cccd_write(p_sam, p_evt_write);
    }
}


static uint32_t 
accel_char_add(ble_sam_t * p_sam, const ble_sam_init_t * p_sam_init)
{
	  uint32_t ec;
	
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
	  ccm.write_perm = p_sam_init->accel_char_attr_md.cccd_write_perm;
	  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ccm.write_perm);
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
		atm.read_perm = p_sam_init->accel_char_attr_md.read_perm;
		atm.write_perm = p_sam_init->accel_char_attr_md.write_perm;
//	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&atm.read_perm);
//	  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&atm.write_perm);
		atm.vloc = BLE_GATTS_VLOC_STACK;
		atm.rd_auth = 0;
		atm.wr_auth = 0;
		atm.vlen = 0;
	
	  CLEAR_OBJECT(&acv);
		acv.p_uuid = &id;
		acv.p_attr_md = &atm;
		acv.init_len = 0x0C; //sizeof(zes_lis3dsh_d);
		acv.max_len = 0x0C; //sizeof(zes_lis3dsh_d);
		acv.init_offs = 0;
		acv.p_value = NULL;
	
	  ec = sd_ble_gatts_characteristic_add(p_sam->srvc_handle, &chm, &acv, &p_sam->accel_char_handles);
		
		if (ec !=  NRF_SUCCESS)
			return ec;
		
		BLE_UUID_BLE_ASSIGN(id, BLE_UUID_REPORT_REF_DESCR);
		
		CLEAR_OBJECT(&atm);
		atm.read_perm = p_sam_init->accel_report_read_perm;
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&atm.write_perm);
		
		atm.vloc = BLE_GATTS_VLOC_STACK;
		atm.rd_auth = 0;
		atm.wr_auth = 0;
		atm.vlen = 0;
		
		CLEAR_OBJECT(&acv);
		acv.p_uuid = &id;
		acv.p_attr_md = &atm;
		acv.init_len = 12;
		acv.max_len = 12;
		acv.init_offs = 0;
		acv.p_value = 0;
		
		//ec = sd_ble_gatts_descriptor_add(p_sam->accel_char_handles.value_handle, &acv, &p_sam->report_ref_handle);
	  if (ec !=  NRF_SUCCESS)
			return ec;
		
		return NRF_SUCCESS;
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
	
		p_sam->evt_handler = p_sam_init->evt_handler;
	  p_sam->conn_handle = BLE_CONN_HANDLE_INVALID;
	  
	  id.type = p_sam->uuid_type;
	  id.uuid = ZES_UUID_ACCEL_SERVICE;
	
	  ec = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &id, &p_sam->srvc_handle);
	 	if (ec != NRF_SUCCESS)
		  return ec;
	
		return accel_char_add(p_sam, p_sam_init);
}

uint32_t ble_sam_measurement_send(ble_sam_t * p_sam, zes_lis3dsh_d * data)
{
	  uint32_t ec;
	  if (p_sam->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			  uint16_t hvx_len = sizeof(zes_lis3dsh_d);
			  ble_gatts_hvx_params_t hvx_params;
			
			  CLEAR_OBJECT(&hvx_params);
			  hvx_params.handle = p_sam->accel_char_handles.value_handle;
			  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
			  hvx_params.offset = 0;
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
	      switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sam, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sam, p_ble_evt);
            break;
				
				case BLE_GATTS_EVT_WRITE:
            on_write(p_sam, p_ble_evt);
            break;
				        
				default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_sam_on_accelerometer_update(ble_sam_t * p_sam, zes_lis3dsh_d *d)
{
	  uint32_t ec;
	  uint16_t len;
	  ble_gatts_hvx_params_t hvx_params;
	  zes_lis3dsh_d data;
	  len = sizeof(zes_lis3dsh_d);
	
	  data.x_G = 1.0;
	  data.y_G = 1.0;
	  data.z_G = 1.0;
	
	  ec = sd_ble_gatts_value_set(p_sam->accel_char_handles.value_handle, 0, &len, (uint8_t*)&data);
	
	  if (ec != NRF_SUCCESS)
    {
        return ec;
    } 
		
	  CLEAR_OBJECT(&hvx_params);
	  hvx_params.handle = p_sam->accel_char_handles.value_handle;
	  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
	  hvx_params.offset = 0;
	  hvx_params.p_len = &len;
	  hvx_params.p_data = (uint8_t *)&data;
	
	  ec = sd_ble_gatts_hvx(p_sam->conn_handle, &hvx_params);
		
		return NRF_SUCCESS;
}