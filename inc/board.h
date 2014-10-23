/** @file
	Zendo board selection
*/

#ifndef BOARD_H_
#define BOARD_H_

#if defined(BOARD_ZPE)
#include "zpe_pin.h"
#elif defined(BOARD_SAM)
#include "sam_pin.h"
#elif defined(BOARD_SEN)
#include "sen_pin.h"
#elif defined(BOARD_SWL)
#include "swl_pin.h"
#else
#error "Board not defined"
#endif

#endif // BOARD_H_
