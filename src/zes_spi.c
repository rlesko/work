/** @file
	SPI implementation
*/

#include "zes_spi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include <stdbool.h>
#include <stdint.h>

res_t
zes_spi_setup(zes_spi_t *s, NRF_SPI_Type *p, uint8_t ss, uint8_t mo, uint8_t mi, uint8_t ck, uint8_t k)
{
	CLEAR_OBJECT(s);

	s->device = p;
	s->type = zes_bus_Spi;
	s->speed = k;
	s->ssel = ss;
	s->mosi = mo;
	s->miso = mi;
	s->sclk = ck;

	s->init = (zes_bus_init_t)zes_spi_init;
	s->trx = (zes_bus_trx_t)zes_spi_trx;
	// s->handler = zes_spi_handler; //@RL: Why is this commented out?
	
	return res_Ok;
}

res_t 
zes_spi_init(zes_spi_t *s)
{
	nrf_gpio_pin_set(s->ssel);
	nrf_gpio_pin_set(s->sclk);
	nrf_gpio_pin_clear(s->mosi);
	nrf_gpio_cfg_output(s->ssel);
	nrf_gpio_cfg_output(s->mosi);
	nrf_gpio_cfg_input(s->miso, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(s->sclk);

	s->device->PSELMOSI = s->mosi;
	s->device->PSELMISO = s->miso;
	s->device->PSELSCK = s->sclk;

	s->device->CONFIG = 
		(SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos)
		| (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos)
		| (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos);
	
	switch (s->speed)
	{
		case zes_spi_125kbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K125; break;
		case zes_spi_250kbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K250; break;
		case zes_spi_500kbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K500; break;
		case zes_spi_1Mbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M1; break;
		case zes_spi_2Mbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M2; break;
		case zes_spi_4Mbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M4; break;
		case zes_spi_8Mbps: s->device->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8; break;
	}

	s->device->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos;
	
	// s->device->EVENTS_READY = 0;
	
	return res_Ok;
}

/** @todo Implement asynch mode */
res_t
zes_spi_trx(zes_spi_t *s, uint8_t l, const uint8_t *dt, uint8_t *dr, uint8_t f)
{
	if ((NRF_GPIO->OUT >> s->ssel) & 1)
	{
		nrf_gpio_pin_clear(s->ssel);
		s->device->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos;
	}
	
	int i = 0, to;
	
	while (i < l)
	{
		s->device->TXD = (dt) ? dt[i] : 0;
		to = s->timeout;
		while (s->device->EVENTS_READY == 0)
			if (s->timeout)
			{
				--to;
				if (!to)
					return res_ETimeout;
			}

		uint8_t rx = s->device->RXD;
		if (dr)
			dr[i] = rx;
		s->device->EVENTS_READY = 0;
		++i;
	}
	
	if (f & zes_bus_Stop)
	{
		nrf_gpio_pin_set(s->ssel);
		s->device->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
	}
	
	return res_Ok;
}

