#include "commons.h"
#include "driverlib/pwm.h"

#define N_SLOT 1
#define I_SLOT 0
#define INIT_WAIT 3000 // needs to be >= 3000 or else it will receive
// UART input during the cycle even when none exists
//#define CYCLE_WAIT 100 // unused in code
#define MAX_POS 1000

// Pin reference
#define CLK_BASE   GPIO_PORTD_BASE
#define CLK_PIN    GPIO_PIN_2

#define START_BASE  GPIO_PORTD_BASE
#define START_PIN   GPIO_PIN_3

#define RACK_UART_PORT GPIO_PORTD_BASE
#define RACK_UART_BASE	UART2_BASE
#define RACK_UART_RX_PIN GPIO_PIN_6
#define RACK_UART_TX_PIN GPIO_PIN_7

#define COM_UART_PORT   GPIO_PORTA_BASE
#define COM_UART_BASE   UART0_BASE
#define COM_UART_RX_PIN GPIO_PIN_0
#define COM_UART_TX_PIN GPIO_PIN_1

#define INTERNAL_CLOCK_BASE GPIO_PORTD_BASE
#define INTERNAL_CLOCK_PIN  GPIO_PIN_1

#define PWM_CLOCK_RATE 50000 /* 50 kHz */

#define FRAMES_TO_SKIP_LEADER 4
#define FRAMES_TO_SKIP_FOLLOW 2

// Outputs
// Port B (all)
// Port E (0-3)
// Port A (4-7)

#define UART_BASE_DEBUG COM_UART_BASE
