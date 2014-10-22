/** @file
	Persistent storage definitions 
*/

#ifndef PSTORAGE_PLATFORM_H_
#define PSTORAGE_PLATFORM_H_

#include <stdint.h>

#define PSTORAGE_FLASH_PAGE_SIZE ((uint16_t)NRF_FICR->CODEPAGESIZE) ///< Size of one flash page
#define PSTORAGE_FLASH_EMPTY_MASK 0xffffffff ///< Bit mask that defines an empty address in flash

#define PSTORAGE_FLASH_PAGE_END \
	((NRF_UICR->BOOTLOADERADDR != PSTORAGE_FLASH_EMPTY_MASK) \
	? (NRF_UICR->BOOTLOADERADDR / PSTORAGE_FLASH_PAGE_SIZE) \
	: NRF_FICR->CODESIZE)


#define PSTORAGE_MAX_APPLICATIONS 2 ///< Maximum number of applications that can be registered with the module, configurable based on system requirements
#define PSTORAGE_MIN_BLOCK_SIZE 0x0010 ///< Minimum size of block that can be registered with the module. Should be configured based on system requirements, recommendation is not have this value to be at least size of word.

#define PSTORAGE_DATA_START_ADDR ((PSTORAGE_FLASH_PAGE_END - PSTORAGE_MAX_APPLICATIONS - 1) \
	* PSTORAGE_FLASH_PAGE_SIZE) ///< Start address for persistent data, configurable according to system requirements
#define PSTORAGE_DATA_END_ADDR ((PSTORAGE_FLASH_PAGE_END - 1) * PSTORAGE_FLASH_PAGE_SIZE) ///< End address for persistent data, configurable according to system requirements
#define PSTORAGE_SWAP_ADDR PSTORAGE_DATA_END_ADDR ///< Top-most page is used as swap area for clear and update

#define PSTORAGE_MAX_BLOCK_SIZE PSTORAGE_FLASH_PAGE_SIZE ///< Maximum size of block that can be registered with the module. Should be configured based on system requirements. And should be greater than or equal to the minimum size.
#define PSTORAGE_CMD_QUEUE_SIZE 30 ///< Maximum number of flash access commands that can be maintained by the module for all applications. Configurable.

/** Abstracts persistently memory block identifier */
typedef uint32_t pstorage_block_t;

typedef struct
{
	uint32_t module_id; ///< Module id
	pstorage_block_t block_id; ///< Block id
} pstorage_handle_t;

typedef uint16_t pstorage_size_t; ///< Size of length and offset fields

/** Handle Flash access result events */
void pstorage_sys_event_handler (uint32_t sys_evt);

#endif // PSTORAGE_PLATFORM_H_
