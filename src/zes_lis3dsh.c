#include "zes_lis3dsh.h"

#include <nrf_delay.h>
#include <spi_master.h>
#include <app_util_platform.h>


res_t
zes_lis3dsh_setup(zes_lis3dsh_t *s, zes_bus_t *b)
{
	CLEAR_OBJECT(s);

	s->bus = b;
	s->address = LIS3DSH_ADDR;

	return res_Ok;
}

res_t
zes_lis3dsh_init(zes_lis3dsh_t *s)
{
	uint8_t b;
	// Power up device
	
		// Validate id 
	if (zes_sns_read((zes_sns_t *)s, LIS3DSH_ID, 1, &b) != res_Ok)
		return -1; // Error reading ID
	
	zes_sns_read((zes_sns_t *)s, LIS3DSH_CTL4, 1, &b);
	b = b | lis3dsh_(BDU, On) | lis3dsh_(ODR, 1600Hz);
	if (zes_sns_write((zes_sns_t *)s, LIS3DSH_CTL4, 1, &b) != res_Ok)
		return -2; // Error powering up device
	
	b = 0x10;
	if (zes_sns_write((zes_sns_t *)s, LIS3DSH_CTL6, 1, &b) != res_Ok)
		return -3; // Error setting SPI settings
	
	//= lis3dsh_(PW, On) | lis3dsh_(MD, Cont); // No power down, continuous mode
	//if (zes_sns_write((zes_sns_t *)s, LIS3DSH_CTL3, 1, &b) != res_Ok)
	//	return -1;



	if (b != lis3dsh_ID_Val)
		return -4; // Invalid ID
	
	return res_Ok; // Init OK
}

res_t
zes_lis3dsh_data(zes_lis3dsh_t *s, zes_lis3dsh_d *d)
{
	uint8_t buf[6];

	if (zes_sns_read((zes_sns_t *)s, LIS3DSH_X0, 6, buf) != res_Ok)
		return -3;

	s->data.x_G = (float)(*(int16_t *)&buf[0]);
	s->data.y_G = (float)(*(int16_t *)&buf[2]);
	s->data.z_G = (float)(*(int16_t *)&buf[4]);

	// CRITICAL_REGION_ENTER();
	if (d)
		*d = s->data;
	// CRITICAL_REGION_EXIT();

	return res_Ok;
}

void
zes_lis3dsh_handler(zes_lis3dsh_t *s)
{

}
