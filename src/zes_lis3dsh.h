/** @file
	LIS3MDL sensor implementation
*/

#ifndef ZES_LIS3DSH_H_
#define ZES_LIS3DSH_H_

#include "zes.h"
#include "zes_sns.h"
#include <lis3dsh.h>

/** Sensor data object */
typedef struct
{
	float x_G; ///< X-axis
	float y_G; ///< Y-axis
	float z_G; ///< Z-axis
} zes_lis3dsh_d;

/** Sensor state object */
typedef struct
{
	zes_sns_HEADER;
	zes_lis3dsh_d data;
} zes_lis3dsh_t;

/** Initialize state object */
res_t zes_lis3dsh_setup(zes_lis3dsh_t *s, zes_bus_t *b);

/** Initialize */
res_t zes_lis3dsh_init(zes_lis3dsh_t *s);

/** Get data */
res_t zes_lis3dsh_data(zes_lis3dsh_t *s, zes_lis3dsh_d *d);

/** Handler */
void zes_lis3dsh_handler(zes_lis3dsh_t *s);

#endif // ZES_LIS3DSH_H_
