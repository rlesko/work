/** @file
	Sensor driver
*/

#ifndef ZES_SNS_H_
#define ZES_SNS_H_

#include "zes.h"

#include "zes_bus.h"

/** Sensor values */
enum zes_sns_value_t
{
	// State flags
	zes_sns_Data = 1<<0,
};

/** Sensor object header
	- state: sensor state
	- code: last event/error code
	- bus: Bus device pointer
	- handler: Sensor handler function
	- last: Last data update tick time
	- address: Device address (I2C)
*/
#define zes_sns_HEADER volatile uint8_t state; volatile res_t code; zes_bus_t *bus; zes_sns_handler_t handler; uint32_t last; uint8_t address

typedef struct zes_sns_t zes_sns_t;

/** Sensor handler */
typedef void (*zes_sns_handler_t)(zes_sns_t *s);

/** Sensor state (base) */
struct zes_sns_t
{
	zes_sns_HEADER;
};

/** Initialize sensor state */
res_t zes_sns_setup(zes_sns_t *s, zes_bus_t *b, zes_sns_handler_t h);

/** Read registers */
res_t zes_sns_read(zes_sns_t *s, uint8_t a, uint8_t l, uint8_t *b);

/** Write registers */
res_t zes_sns_write(zes_sns_t *s, uint8_t a, uint8_t l, const uint8_t *b);

#endif // ZES_SNS_H_
