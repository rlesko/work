/** @file
	Zendo ProtoElement system setup
*/

#include "zes_sys.h"

// #include "zpe_buzzer.h"
// #include <app_timer.h>


#include <nrf_gpio.h>

res_t
zes_sys_init()
{
	nrf_gpio_pin_clear(PW_CTRL); // Disable load (same as pull-down)
	nrf_gpio_cfg_output(PW_CTRL);

	return res_Ok;
}


res_t
zes_sys_pwld(bool t)
{
	if (t)
		nrf_gpio_pin_set(PW_CTRL);
	else
		nrf_gpio_pin_clear(PW_CTRL);

	return res_Ok;
}
