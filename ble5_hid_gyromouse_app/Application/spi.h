#ifndef APPLICATION_SPI_H_
#define APPLICATION_SPI_H_

#include <stdlib.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"          // Access to the GET_MCU_CLOCK define
#include "inc/hw_ioc.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"

#define BSP_SPI_CS       IOID_20
#define BSP_SPI_MOSI            IOID_9
#define BSP_SPI_MISO            IOID_8
#define BSP_SPI_CLK_FLASH       IOID_10

/**
* Initialize SPI interface
*
* @return none
*/
extern void bspSpiOpen(uint32_t bitRate, uint32_t clkPin);

/**
* Close SPI interface
*
* @return True when successful.
*/
extern void bspSpiClose(void);

/**
* Clear data from SPI interface
*
* @return none
*/
extern void bspSpiFlush(void);

/**
* Read from an SPI device
*
* @return True when successful.
*/
extern int bspSpiRead( uint8_t *buf, size_t length);

/**
* Write to an SPI device
*
* @return True when successful.
*/
extern int bspSpiWrite(const uint8_t *buf, size_t length);

#endif /* APPLICATION_SPI_H_ */
