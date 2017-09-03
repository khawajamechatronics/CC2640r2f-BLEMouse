/*********************************************************************
 * INCLUDES
 */
#include <Board.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <stdint.h>

/*********************************************************************
 * CONSTANTS
 */
#define UART_PRINTF_BUF_LEN      1024

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t  uartPrintf_outArray[UART_PRINTF_BUF_LEN];
static uint16_t uartPrintf_head = 0;
static uint16_t uartPrintf_tail = 0;
static UART_Handle hUart = NULL;


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
void UartPrintf_init(UART_Handle handle)
{
    hUart = handle;
}

/*********************************************************************
 * SYSTEM HOOK FUNCTIONS
 */

/*********************************************************************
 * @fn      uartPrintf_putch
 *
 * @brief   User supplied PutChar function.
 *          typedef Void (*SysCallback_PutchFxn)(Char);
 *
 *          This function is called whenever the System module needs
 *          to output a character.
 *
 *          This implementation fills a very basic ring-buffer, and relies
 *          on another function to flush this buffer out to UART.
 *
 *          Requires SysCallback to be the system provider module.
 *          Initialized via SysCallback.putchFxn = "&uartPrintf_putch"; in the
 *          TI-RTOS configuration script.
 *
 * @param   ch - Character
 *
 * @return  None.
 *
 * @post    ::uartPrintf_head is incremented by one with wrap at UART_PRINTF_BUF_LEN
 *          if there is room.
 */
void uartPrintf_putch(char ch)
{
    // uartPrintf_tail should never catch up with uartPrintf_head. Discard in-between bytes.
    if ( (uartPrintf_head + 1) % UART_PRINTF_BUF_LEN == uartPrintf_tail )
        return;

    uartPrintf_outArray[uartPrintf_head] = ch;
    uartPrintf_head++;

    if (uartPrintf_head >= UART_PRINTF_BUF_LEN)
        uartPrintf_head = 0;
}

/*********************************************************************
 * @fn      uartPrintf_flush
 *
 * @brief   Printf-buffer flush function
 *
 *          In this implementation it is intended to be called by the
 *          Idle task when nothing else is running.
 *
 *          This is achieved by setting up the Idle task in the TI-RTOS
 *          configuration script like so:
 *
 *          var Idle = xdc.useModule('ti.sysbios.knl.Idle');
 *          Idle.addFunc('&uartPrintf_flush');
 *
 * @param   None. Relies on global state.
 *
 * @return  None.
 *
 * @post    ::uartPrintf_tail is incremented to where uartPrintf_head
 *          was at the time the function was called.
  */
void uartPrintf_flush()
{
    // Abort in case UART hasn't been initialized.
    if (NULL == hUart)
        return;

  // Lock head position to avoid race conditions
  uint16_t curHead = uartPrintf_head;

  // Find out how much data must be output, and how to output it.
    bool needWrap = curHead < uartPrintf_tail;
  uint16_t outLen = needWrap?(UART_PRINTF_BUF_LEN-uartPrintf_tail+curHead):(curHead-uartPrintf_tail);

    if (outLen)
    {
        if (needWrap)
        {
            UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], UART_PRINTF_BUF_LEN - uartPrintf_tail);
            UART_write(hUart, uartPrintf_outArray, curHead);
        }
        else
        {
            UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], outLen);
        }
    }

    uartPrintf_tail = curHead;
}
