/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides interface routines to the ATmega8515.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 * Note: this version was modified from LPC21xx to ATmega8515.
 *****************************************************************************/
#include "uart.h"
#include <avr/io.h>

#if UART0_SUPPORT
uint8_t uart0_rx_buffer[UART0_RX_BUFFER_SIZE];
volatile uint8_t uart0_rx_insert_idx, uart0_rx_extract_idx;

uint8_t uart0_tx_buffer[UART0_TX_BUFFER_SIZE];
volatile uint8_t uart0_tx_insert_idx, uart0_tx_extract_idx;
volatile uint8_t uart0_tx_running;

#endif

#if UART1_SUPPORT
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
volatile uint8_t uart1_rx_insert_idx, uart0_rx_extract_idx;

uint8_t uart1_tx_buffer[UART1_TX_BUFFER_SIZE];
volatile uint8_t uart1_tx_insert_idx, uart0_tx_extract_idx;
volatile uint8_t uart1_tx_running;

#endif


#if UART0_SUPPORT

ISR(USART0_RX_vect)
{
	
	static uint8_t tmp;
	
	//do {
		
		// calc next insert index & store character
		tmp = (uart0_rx_insert_idx + 1) % UART0_RX_BUFFER_SIZE;
		uart0_rx_buffer[uart0_rx_insert_idx] = UDR0;

		// check for more room in queue
		if (tmp != uart0_rx_extract_idx)
			uart0_rx_insert_idx = tmp; // update insert index
	
	//} while (UCSR0A & (1u << RXC0));
	
}

ISR(USART0_UDRE_vect)
{
	
	while (UCSR0A & (1u << UDRE0)) {
		
		// check if more data to send
		if (uart0_tx_insert_idx != uart0_tx_extract_idx) {
			
			UDR0 = uart0_tx_buffer[uart0_tx_extract_idx++];
			uart0_tx_extract_idx %= UART0_TX_BUFFER_SIZE;
			
		} else {
			
			// No: Disable interrupt
			UCSR0B &= ~(1u << UDRIE0);
			uart0_tx_running = 0; // clear running 
			break;
			
		}
	}
	
}

/******************************************************************************
 *
 * Function Name: uart0Init()
 *
 * Description:  
 *    This function initializes the UART for async mode
 *
 * Calling Sequence: 
 *    baudrate divisor - use UART_BAUD macro
 *
 * Returns:
 *    void
 *
 * NOTE: uart0Init(UART_BAUD(9600));
 *
 *****************************************************************************/
void uart0Init(const uint16_t baud)
{
	
	UCSR0B = 0x00;           // Disable all functionality
	UCSR0A = (1u << TXC0);   // Clear complete tx interrupt flag

	// set the baud rate
	UBRR0H = ((uint8_t) ((baud >> 8) & 0xFFu)) & ~UCSRC_SELECT;
	UBRR0L = (uint8_t) (baud & 0xFFu);

	// set the number of characters and other
	// user specified operating parameters
	UCSR0C = UCSRC_SELECT | (3u << UCSZ00); // 8N1

	// initialize the transmit data queue
	uart0_tx_extract_idx = uart0_tx_insert_idx = 0;
	uart0_tx_running = 0;

	// initialize the receive data queue
	uart0_rx_extract_idx = uart0_rx_insert_idx = 0;

	// enable USART & interrupts
	UCSR0B = (1u << RXCIE0) | (1u << UDRIE0) | (1u << RXEN0) | (1u << TXEN0);
	
}

/******************************************************************************
 *
 * Function Name: uart0Putch()
 *
 * Description:  
 *    This function puts a character into the UART output queue for
 *    transmission.
 *
 * Calling Sequence: 
 *    character to be transmitted
 *
 * Returns:
 *    ch on success, -1 on error (queue full)
 *
 *****************************************************************************/
const int16_t uart0Putch(const uint8_t ch) {

	uint16_t temp;

	temp = (uart0_tx_insert_idx + 1) % UART0_TX_BUFFER_SIZE;

	if (temp == uart0_tx_extract_idx)
		return -1; // no room

	// check if in process of sending data
	if (uart0_tx_running) {
		
		// add to queue
		uart0_tx_buffer[uart0_tx_insert_idx] = ch;
		uart0_tx_insert_idx = temp;
		
	} else {
		
		// set running flag and write to output register
		uart0_tx_running = 1;
		UCSR0B |= (1u << UDRIE0); // enable data ready interrupts
		UDR0 = ch;
	}

	

	return ch;
}

/******************************************************************************
 *
 * Function Name: uart0Space()
 *
 * Description:  
 *    This function gets the available space in the transmit queue
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    available space in the transmit queue
 *
 *****************************************************************************/
const uint16_t uart0Space(void) {
	
	int space;

	if ((space = (uart0_tx_extract_idx - uart0_tx_insert_idx)) <= 0)
		space += UART0_TX_BUFFER_SIZE;

	return (uint16_t) (space - 1);

}

/******************************************************************************
 *
 * Function Name: uart0Puts()
 *
 * Description:  
 *    This function writes a NULL terminated 'string' to the UART output
 *    queue.
 *
 * Calling Sequence: 
 *    address of the string
 *
 *
 *****************************************************************************/
void uart0Puts(const char* string) {
	
	register uint8_t ch = 0xFFu;
	
	while (ch = *string) {
		
		while (uart0Putch(ch) < 0);
			
		string++;
		
	}

}

/******************************************************************************
 *
 * Function Name: uart0Write()
 *
 * Description:  
 *    This function writes 'count' characters from 'buffer' to the UART
 *    output queue.
 *
 *
 *****************************************************************************/
void uart0Write(const char * buffer, uint16_t count) {

	while (count) {
		
		while (uart0Putch(*buffer) < 0);
			
		buffer++;
		count--;
			
	}

}

/******************************************************************************
 *
 * Function Name: uart0Getch()
 *
 * Description:  
 *    This function gets a character from the UART receive queue
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    character on success, -1 if no character is available
 *
 *****************************************************************************/
const int16_t uart0Getch(void) {
	
	uint8_t ch;
	uint8_t tmp = (uart0_rx_extract_idx + 1) % UART0_RX_BUFFER_SIZE;

	if (uart0_rx_insert_idx == uart0_rx_extract_idx) // check if character is available
		return -1;

	ch = uart0_rx_buffer[uart0_rx_extract_idx]; // get character, bump pointer
	uart0_rx_extract_idx = tmp; // limit the pointer
	
	return ch;

}

#endif