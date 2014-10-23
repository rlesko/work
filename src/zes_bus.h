/** @file
	Sensor bus management
*/

#ifndef ZES_BUS_H_
#define ZES_BUS_H_

#include <stdint.h>
#include <stdbool.h>

#include "zes_res.h"

enum zes_bus_value_t
{
	// Bus types
	zes_bus_Twi = 0,
	zes_bus_Spi,

	// Flags
	zes_bus_Tx = 1<<0, ///< Transmit
	zes_bus_Rx = 1<<1, ///< Receive
	zes_bus_Stop = 1<<2, // Stop after transfer
	zes_bus_Asynch = 1<<3, // Asynchronous operation
};

/** Bus object header
	- device: Peripheral pointer (NRF_TWIx)
	- type: Bus type id
	- code: Error/event
	- speed: Speed id
	- timeout: Timeout cycles
	- channel: PPI channel
	- context: Bus-specific context (eg I2C device address, structure pointer)
	- init: Initialize bus
	- trx: Read and/or write bus
	- handler: Asynch operation/error handler
*/
#define zes_bus_HEADER(x) x *device; uint8_t type; res_t code; uint8_t speed; uint16_t timeout; uint8_t channel; \
	uintptr_t context; zes_bus_init_t init; zes_bus_trx_t trx; zes_bus_handler_t handler

typedef struct zes_bus_t zes_bus_t;

/** Initialize the bus module */
typedef res_t (*zes_bus_init_t)(zes_bus_t *s);

/** Bus read/write */
typedef res_t (*zes_bus_trx_t)(zes_bus_t *s, uint8_t l, const uint8_t *din, uint8_t *dout, uint8_t f);

/** Bus handler */
typedef void (*zes_bus_handler_t)(zes_bus_t *s);

/** Bus base state */
struct zes_bus_t
{
	zes_bus_HEADER(void);
};

#endif // ZES_BUS_H_
