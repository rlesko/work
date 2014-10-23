/** @file
	Sensor interface implementation
*/

#include "zes_sns.h"
#include "zes_bus.h"

res_t
zes_sns_setup(zes_sns_t *s, zes_bus_t *b, zes_sns_handler_t h)
{
	CLEAR_OBJECT(s);

	s->bus = b;
	s->handler = h;

	return res_Ok;
}

res_t
zes_sns_read(zes_sns_t *s, uint8_t a, uint8_t l, uint8_t *b)
{
	switch (s->bus->type)
	{
	case zes_bus_Twi:
		s->bus->context = s->address;
		if (l > 1)
			a |= 0x80; // Sequential
		break;
	
	case zes_bus_Spi:
		a |= 0x80; // Read operation
		if (l > 1)
			;//a |= 0x40; // Sequential
		break;
	
	default:
		return res_EArgument;
	}
	
	res_t r = res_Ok;
	if (r == res_Ok)
		r = s->bus->trx(s->bus, 1, &a, NULL, zes_bus_Tx);
	if (r == res_Ok)
		r = s->bus->trx(s->bus, l, NULL, b, zes_bus_Rx|zes_bus_Stop);

	return r;
}

res_t
zes_sns_write(zes_sns_t *s, uint8_t a, uint8_t l, const uint8_t *b)
{
	switch (s->bus->type)
	{
	case zes_bus_Twi:
		s->bus->context = s->address;
		if (l > 1)
			a |= 0x80; // Sequential
		break;
	
	case zes_bus_Spi:
		if (l > 1)
			a |= 0x40; // Sequential
		break;
	
	default:
		return res_EArgument;
	}

	res_t r = res_Ok;
	if (r == res_Ok)
		r = s->bus->trx(s->bus, 1, &a, NULL, zes_bus_Tx);
	if (r == res_Ok)
		r = s->bus->trx(s->bus, l, b, NULL, zes_bus_Tx|zes_bus_Stop);

	return r;
}
