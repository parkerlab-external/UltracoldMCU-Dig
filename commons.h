#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#ifdef UART_BASE_DEBUG
char g_debug_vals[500] = {0};
char g_debug_pos = 0;
#endif

inline int32_t UARTCharGetAndLog(uint32_t uart_base) {
    int32_t val_char;
    val_char = UARTCharGet(uart_base);
#ifdef UART_BASE_DEBUG
    if (uart_base == UART_BASE_DEBUG)
    g_debug_vals[g_debug_pos] = val_char;
    g_debug_pos++;
#endif
    return val_char;
}

inline int32_t UARTCharGetWithWait(uint32_t uart_base) {
    int cycle_wait = 100000;
    while ( !UARTCharsAvail(uart_base) && (cycle_wait > 0 )) {
        cycle_wait--;
        if (cycle_wait == 0) return -1;
    }
    return (UARTCharGetAndLog(uart_base));
}

inline void UARTClear(uint32_t uart_base) {
    while ( UARTCharsAvail(uart_base) ) {
        UARTCharGet(uart_base);
    }
}

