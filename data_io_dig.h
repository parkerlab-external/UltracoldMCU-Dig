#include "config_dig.h"
#include "config_device.h"
// #include "config_device_leadrepeat.h"

#ifdef REGULAR_MODE
#define DIG_INPUT_UART_BASE RACK_UART_BASE
#endif
#ifdef REPEATER_MODE
#define DIG_INPUT_UART_BASE COM_UART_BASE
#endif

// Global Variable Declaration
uint32_t g_frame_pos[N_SLOT] = {0}; // Counting index
uint32_t g_chkpt_pos[N_SLOT]   = {0}; // Array index
uint32_t g_checksum_uart = 0; // UART error detection

uint32_t g_dig_val_cache[N_SLOT] = {0};
// A list of durations (in units of cycles) for increment addition
uint32_t g_table_poly_dur[N_SLOT][MAX_POS] = { { 0 } };
// During each duration, the corresponding value is written to the pins
uint32_t g_table_dig_val[N_SLOT][MAX_POS] = { { 0x0000 } };

uint32_t g_n_chkpt[N_SLOT] = {4}; // Effective array length

int32_t g_t_start = 0;

// Local

inline void modify_checksum(int32_t acc) {
#ifdef REGULAR_MODE
    g_checksum_uart += acc;
#endif
}

inline void clear_pos(int i_SLOT) {
    g_chkpt_pos[i_SLOT]   = 0;    
    g_frame_pos[i_SLOT] = 0;
}

inline void advance_chkpt_pos(int i_SLOT) {
    g_chkpt_pos[i_SLOT]  += 1;    
    g_frame_pos[i_SLOT] = 0;
}

inline void recede_chkpt_pos(int i_SLOT) {
    if ( g_chkpt_pos[i_SLOT] >= g_n_chkpt[i_SLOT] )
        g_chkpt_pos[i_SLOT] = g_n_chkpt[i_SLOT] - 1;
}

inline void advance_frame_pos(int i_SLOT) {
    g_frame_pos[i_SLOT] += 1;
    // Move to next command in the cycle
    if ( g_frame_pos[i_SLOT] >= g_table_poly_dur[i_SLOT][g_chkpt_pos[i_SLOT]] ) {
        g_chkpt_pos[i_SLOT] += 1;
        g_frame_pos[i_SLOT] = 0;
    }
    if ( g_chkpt_pos[i_SLOT] >= g_n_chkpt[i_SLOT] ) { // Restart the loop if we go out of range, may be changed in future  
#ifdef LEADER_MODE
        g_chkpt_pos[i_SLOT] = 0;
        g_t_start = FRAMES_TO_SKIP_LEADER;
        GPIOPinWrite( START_BASE, START_PIN, 0 ); // Clear the start pin
#endif
#ifdef FOLLOW_MODE
        g_chkpt_pos[i_SLOT] -= 1;
#endif
    }
}

// Output

void write_dig_val(int i_SLOT)
{
    GPIOPinWrite( GPIO_PORTB_BASE, 0xFF, (g_dig_val_cache[i_SLOT] >> 8) );
    GPIOPinWrite( GPIO_PORTE_BASE, 0x0F, g_dig_val_cache[i_SLOT] );
    GPIOPinWrite( GPIO_PORTA_BASE, 0xF0, g_dig_val_cache[i_SLOT] );
}

// UART checksum
inline void write_checksum_uart() {
    int i_tx;
    for(i_tx = 0; i_tx < 4; i_tx++)
        UARTCharPut(RACK_UART_BASE, g_checksum_uart >> ((3 - i_tx) * 8));
}

inline int32_t readi8_uart(uint32_t uart_base) {
    int32_t val1 = UARTCharGetWithWait(uart_base);
    modify_checksum(val1);
    return ( val1 );
}

inline int32_t readi16_uart(uint32_t uart_base){ // make a 32 bit number from 8 bit reads
    int32_t val1, val2;
    val1 = UARTCharGetWithWait(uart_base);
    if ( val1 < 0) { return( val1 ); }
    val2 = UARTCharGetWithWait(uart_base);
    if ( val2 < 0 ) { return ( val2 ); }
    modify_checksum(val1 + val2);
    return ( val1 << 8| val2 );
}

inline int32_t readi32_uart(uint32_t uart_base) { // make a 32 bit number from 8 bit reads
    int32_t val1, val2, val3, val4;
    val1 = UARTCharGetWithWait(uart_base);
    if ( val1 < 0) { return( val1 ); }
    val2 = UARTCharGetWithWait(uart_base);
    if ( val2 < 0 ) { return ( val2 ); }
    val3 = UARTCharGetWithWait(uart_base);
    if ( val3 < 0 ) { return ( val3 ); }
    val4 = UARTCharGetWithWait(uart_base);
    if ( val4 < 0 ) { return( val4 ); }
    modify_checksum(val1 + val2 + val3 + val4);
    return ( val1 << 24| val2 << 16 | val3 << 8 | val4 );
}
