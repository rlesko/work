
#ifndef ZES_SAM_H_
#define ZES_SAM_H_

#include <ble.h> // From S110
#include <ble_srv_common.h> // From BLE/BLE_Services
#include <zes_lis3dsh.h>

#define ZENDO_UUID_BASE { 0x86, 0xEE, 0x3E, 0xBB, 0x08, 0xAF, 0xD4, 0xF1, 0xAD, 0xD8, 0x9B, 0x82, 0x00, 0x00, 0x1E, 0xF2}
#define ZES_UUID_ACCEL_SERVICE 0x1523
#define ZES_UUID_ACCEL_CHAR 0x1524 //Characteristic

typedef enum
{
	BLE_SAM_EVT_NOTIFICATION_DISABLED,
	BLE_SAM_EVT_NOTIFICATION_ENABLED
} ble_sam_evt_type_t;

typedef struct
{
	ble_sam_evt_type_t evt_type;
} ble_sam_evt_t;

typedef struct ble_sam_s ble_sam_t;
typedef void (*ble_sam_evt_handler_t)(ble_sam_t *p, ble_sam_evt_t *e);
typedef void (*ble_sam_led_handler_t)(ble_sam_t *p, uint8_t v);

typedef struct
{
  ble_sam_evt_handler_t          evt_handler;
  bool                           support_notification;
  ble_srv_cccd_security_mode_t   accel_char_attr_md;
  ble_gap_conn_sec_mode_t        accel_report_read_perm;
	ble_srv_report_ref_t *         p_report_ref;
  //uint8_t initial_batt_level;
} ble_sam_init_t;

typedef struct ble_sam_s
{
	ble_sam_evt_handler_t          evt_handler;
	uint16_t                       srvc_handle;
	ble_gatts_char_handles_t       accel_char_handles;  
  uint16_t                       report_ref_handle;	
	uint8_t                        uuid_type;
	uint16_t                       conn_handle;
} ble_sam_t;

uint32_t ble_sam_init(ble_sam_t * p_sam, const ble_sam_init_t * p_sam_init);

void ble_sam_on_ble_evt(ble_sam_t * p_sam, ble_evt_t * p_ble_evt);

uint32_t ble_sam_measurement_send(ble_sam_t * p_sam, zes_lis3dsh_d * data);

uint32_t ble_sam_on_accelerometer_update(ble_sam_t * p_sam, zes_lis3dsh_d *d);


#endif