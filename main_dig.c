//*****************************************************************************
//
// digout_launchpad.c
//
// This program provides 16 digital output channels and a mode switching.  These modes
// either enable the controller to output a single "sync" line, or it enables the
// controller to function with respect to an already existing "sync" line.
//
//
//*****************************************************************************

#include "data_io_dig.h"

inline void read_absl_val(uint32_t *p_absl_val) {
    *p_absl_val = g_table_dig_val[I_SLOT][g_chkpt_pos[I_SLOT]];
    advance_frame_pos(I_SLOT);
    recede_chkpt_pos(I_SLOT);
}

// Handles UART for regular mode, called directly in repeater mode
void UARTHandlerDigital() {
    bool keep_going = true;
    uint32_t char_prefix = 0;
    int32_t pos;
    g_checksum_uart = 0;
#ifdef REGULAR_MODE
    UARTIntClear(DIG_INPUT_UART_BASE, UARTIntStatus(DIG_INPUT_UART_BASE, true));
#endif
    char_prefix = readi8_uart(DIG_INPUT_UART_BASE);
    keep_going = (char_prefix == 0xAA);
    while ( keep_going ) {
        char_prefix = readi8_uart(DIG_INPUT_UART_BASE);
        switch (char_prefix) {
            case 'U' :  // Update one channel's arrays to have new values
                pos = readi16_uart(DIG_INPUT_UART_BASE);
                pos = (pos < 0) ? 0 : ((pos > MAX_POS) ? MAX_POS - 1 : pos);
                // Receive and update increment durations and values
                g_table_poly_dur[I_SLOT][pos] = readi32_uart(DIG_INPUT_UART_BASE);
                g_table_dig_val[I_SLOT][pos] = readi16_uart(DIG_INPUT_UART_BASE);
                break;
            case 'R' : // Reset the array index
                g_chkpt_pos[I_SLOT] = readi16_uart(DIG_INPUT_UART_BASE);
                break;
            case 'Z' : // Make a given channel output its minimum voltage and set the arrays to zero.
                for (pos = 0; pos < MAX_POS; pos += 1) {
                    g_table_poly_dur[I_SLOT][pos] = 0;
                    g_table_dig_val[I_SLOT][pos] = 0;
                }
                break;
            case 'E' : // Change the effective array length
                g_n_chkpt[I_SLOT] = readi16_uart(DIG_INPUT_UART_BASE);
                break;
#ifdef LEADER_MODE
            // Only leader mode sends the start signal.
            case 'S' : // Start the sequence if stopped at the end
                g_t_start--;
                SysCtlDelay( 3 );
                keep_going = false;
                break;
            case 'A' : // Abort the sequence by setting the start pin low
                while (GPIOPinRead(CLK_BASE, CLK_PIN)); // wait for the next trigger
                GPIOPinWrite( START_BASE, START_PIN, 0 );
                break;
#endif
            case 'T' : // Break the UART Interrupt
#ifdef REGULAR_MODE
                // UART write only happens only for regular mode
                GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_TX_PIN);
                SysCtlDelay(INIT_WAIT);                
                write_checksum_uart();      
                while(UARTBusy(RACK_UART_BASE));
                GPIOPinTypeGPIOInput(RACK_UART_PORT, RACK_UART_TX_PIN);
#endif      
                keep_going = false;
                break;
        }
    }
}

// Local UART that does not go through rack
// Only in repeater mode
void UARTHandlerCOM() {
    int32_t checksum_recv, checksum_uart = 0;
    int32_t len_data = 0;
    int32_t address_regular;
    int i_data;
    char output_data = 'E';

    UARTCharGetAndLog(COM_UART_BASE); // char_prefix, unused
    address_regular = UARTCharGetAndLog(COM_UART_BASE);
    len_data = readi32_uart(COM_UART_BASE);
    g_checksum_uart = 0;

    if (address_regular == UART_ADDRESS) { // For the local transmission
        UARTHandlerDigital();
        checksum_recv = g_checksum_uart;
        checksum_uart = g_checksum_uart;
        g_checksum_uart = 0;
    } else { // For other regular modules
        UARTClear(RACK_UART_BASE);
        UART9BitAddrSend(RACK_UART_BASE, address_regular);
        for (i_data = 0; i_data < len_data; i_data++) {
            int32_t current_byte = UARTCharGetAndLog(COM_UART_BASE);
            checksum_uart += (uint8_t)(current_byte);
            UARTCharPut(RACK_UART_BASE,current_byte);
        }
        checksum_recv = readi32_uart(RACK_UART_BASE);
        g_checksum_uart = 0;
    }
    if ( checksum_recv < 0) { // T for timeout error, which returns a -1
        output_data = 'T';
    } else if ( checksum_recv == checksum_uart ) { // K if the transmission was good, signifying it is OK
        output_data = 'K';
    }
    // compute output data
    UARTCharPut(COM_UART_BASE,output_data);
}

void GPIOIntHandler(void)
{
    int to_start = GPIOPinRead( START_BASE, START_PIN );
    GPIOIntClear( CLK_BASE,0x1FF ); // Clear all interrupts
    if (!GPIOPinRead(CLK_BASE, CLK_PIN)) {  return; } // Proceed only within clock cycle

#ifdef LEADER_MODE
    if ( g_t_start <= 0 ) {
        read_absl_val(&g_dig_val_cache[I_SLOT]);
        write_dig_val(I_SLOT);
    } else if ( g_t_start < FRAMES_TO_SKIP_LEADER ) {
        g_t_start--;
        if( g_t_start == 1) {
            while (GPIOPinRead(CLK_BASE, CLK_PIN));
            GPIOPinWrite( START_BASE, START_PIN, START_PIN ); // Set the start pin to get everybody going
        }
    }
#endif

#ifdef FOLLOW_MODE
    if ( to_start == 0 ) {
        g_t_start = FRAMES_TO_SKIP_FOLLOW;
    } else if ( g_t_start > 0) {
        g_t_start--;
    }
    if( !g_t_start ) {
        read_absl_val(&g_dig_val_cache[I_SLOT]);
        write_dig_val(I_SLOT);
    } else {
        clear_pos(I_SLOT);
    }

#endif

#ifdef REGULAR_MODE
    if (UARTCharsAvail(RACK_UART_BASE))
        UARTHandlerDigital();
#endif 
#ifdef REPEATER_MODE
    if (UARTCharsAvail(COM_UART_BASE))
        UARTHandlerCOM();
#endif
}

int
main( void )
{
    uint32_t clock_rate;

    // Set the system clock to run at 80Mhz off PLL with external crystal as reference.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);

    SysCtlDelay(3);
    IntMasterDisable();

    // Enable the peripherals we will use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
#ifdef USE_INTERNAL_CLOCK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
#endif

    clock_rate = SysCtlClockGet();

    // Configure pins
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, 0xF0);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0x0F);
    GPIOPinTypeGPIOInput(CLK_BASE, CLK_PIN);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // Configure start pin based on mode
#ifdef LEADER_MODE
    GPIOPinTypeGPIOOutput(START_BASE, START_PIN);
#endif
#ifdef FOLLOW_MODE
    GPIOPinTypeGPIOInput(START_BASE, START_PIN);
#endif

    // Enable the UART Pins
    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_RX_PIN);
    GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_TX_PIN);
    // Enable COM UART Pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(COM_UART_PORT, COM_UART_RX_PIN);
    GPIOPinTypeUART(COM_UART_PORT, COM_UART_TX_PIN);

#ifdef USE_INTERNAL_CLOCK
    // Enable PWM Pins
    GPIOPinTypePWM(INTERNAL_CLOCK_BASE, INTERNAL_CLOCK_PIN);
    GPIOPadConfigSet(GPIO_PORTD_BASE,  GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinConfigure(GPIO_PD1_M0PWM7);

    // Enable PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, clock_rate/PWM_CLOCK_RATE);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, clock_rate/(2*PWM_CLOCK_RATE));
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
#else
    (void)clock_rate; /* Suppress unused variable warning */
#endif

    // Rack UART setup
#ifdef REGULAR_MODE
    UART9BitAddrSet(RACK_UART_BASE, UART_ADDRESS, 0xFF);
    UART9BitEnable(RACK_UART_BASE);
#endif

    UARTConfigSetExpClk(RACK_UART_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_ZERO));
    // COM UART setup
    UARTConfigSetExpClk(COM_UART_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_NONE));
#ifdef LEADER_MODE
    GPIOPinWrite(START_BASE,START_PIN,0);
#endif

    // Set up interrupts
    GPIOIntEnable(CLK_BASE,CLK_PIN);
    GPIOIntTypeSet(CLK_BASE,CLK_PIN,GPIO_RISING_EDGE);
    IntEnable(INT_GPIOD);

    // 9 Bit Interrupt Stuff
    UARTFIFOLevelSet(RACK_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntEnable(RACK_UART_BASE, UART_INT_9BIT);
    UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));
    // Interrupt Stuff
    UARTFIFOLevelSet(COM_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntEnable(COM_UART_BASE, UART_INT_9BIT);
    UARTIntClear(COM_UART_BASE, UARTIntStatus(COM_UART_BASE, true));

    SysCtlDelay(INIT_WAIT);
    while(UARTCharsAvail(RACK_UART_BASE)) UARTCharGet(RACK_UART_BASE);
    while(UARTCharsAvail(COM_UART_BASE)) UARTCharGet(COM_UART_BASE);

    IntMasterEnable();

    while(true)
    {
    }
}
