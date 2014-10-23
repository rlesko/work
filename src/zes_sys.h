/** @file
	Zendo sensor system setup
*/

#ifndef ZES_SYS_H_
#define ZES_SYS_H_

#include "zes.h"

/** Initialize system */
res_t zes_sys_init(void);

/** Control power load supply */
res_t zes_sys_pwld(bool c);

#endif // ZES_SYS_H_
