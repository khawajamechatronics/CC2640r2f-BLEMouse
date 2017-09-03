#ifndef UART_PRINTF_H
#define UART_PRINTF_H

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/UART.h>

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      UartPrintf_init
 *
 * @brief   Initializes the putchar hooks with the handle to the UART.
 *
 * @param   handle - UART driver handle to an initialized and opened UART.
 *
 * @return  None.
 */
void UartPrintf_init(UART_Handle handle);


#ifdef __cplusplus
}
#endif

#endif // UART_PRINTF_H
