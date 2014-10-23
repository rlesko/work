/** @file
	Sensor SPI management
*/

#ifndef ZES_SPI_H_
#define ZES_SPI_H_

#include "zes.h"
#include "zes_bus.h"

/** SPI values */
enum zes_spi_value_t
{
	// Speed values
	zes_spi_125kbps = 0,
	zes_spi_250kbps,
	zes_spi_500kbps,
	zes_spi_1Mbps,
	zes_spi_2Mbps,
	zes_spi_4Mbps,
	zes_spi_8Mbps,
};

/** SPI device state type */
typedef struct
{
	zes_bus_HEADER(NRF_SPI_Type);
	uint8_t ssel; ///< SSEL pin
	uint8_t mosi; ///< MOSI pin
	uint8_t miso; ///< MISO pin
	uint8_t sclk; ///< SCLK pin
} zes_spi_t;

/** Initialize the SPI object */
res_t zes_spi_setup(zes_spi_t *s, NRF_SPI_Type *p, uint8_t ss, uint8_t mo, uint8_t mi, uint8_t cl, uint8_t k);

/** Initialize the SPI peripheral */
res_t zes_spi_init(zes_spi_t *s);

/** SPI read/write */
res_t zes_spi_trx(zes_spi_t *s, uint8_t l, const uint8_t *dt, uint8_t *dr, uint8_t f);

#endif // ZES_SPI_H_
