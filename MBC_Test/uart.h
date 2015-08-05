/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides the interface definitions for for uart.c
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 * Note: this version was modified from LPC21xx to ATmega8515.
 *****************************************************************************/
#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"

///////////////////////////////////////////////////////////////////////////////
// ATMEL REDEFINES
///////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega8515__)

	// Remap defines
	#define USART0_RX_vect   USART_RX_vect
	#define USART0_TX_vect   USART_TX_vect
	#define USART0_UDRE_vect USART_UDRE_vect
	
	#define UDR0           UDR
	
	#define UCSR0A         UCSRA
	#define MPCM0          MPCM
	#define U2X0           U2X
	#define UPE0           PE
	#define DOR0           DOR
	#define FE0            FE
	#define UDRE0          UDRE
	#define TXC0           TXC
	#define RXC0           RXC
	
	#define UCSR0B         UCSRB
	#define TXB80          TXB8
	#define RXB80          RXB8
	#define UCSZ02         UCSZ2
	#define TXEN0          TXEN
	#define RXEN0          RXEN
	#define UDRIE0         UDRIE
	#define TXCIE0         TXCIE
	#define RXCIE0         RXCIE
	
	#define UCSR0C         UCSRC
	#define UCPOL0         UCPOL
	#define UCSZ00         UCSZ0
	#define UCSZ01         UCSZ1
	#define USBS0          USBS
	#define UPM00          UPM0
	#define UPM01          UPM1
	#define UMSEL00        UMSEL
	#define UCSRC_SELECT   (1u << URSEL)
	
	#define UBRR0H         UBRRH
	#define UBRR0L         UBRRL
	
	
#elif defined(__AVR_ATmega644P__)

	// Device is supported.
	#define UCSRC_SELECT     (0)
	
#else

	#error "Device not supported."
	
#endif

///////////////////////////////////////////////////////////////////////////////
#define UART0_SUPPORT    (1)            // non-zero to enable UART0 code
#define UART1_SUPPORT    (0)            // non-zero to enable UART1 code
///////////////////////////////////////////////////////////////////////////////
// code is optimized for power of 2 buffer sizes (16, 32, 64, 128, ...)
#define UART0_RX_BUFFER_SIZE 16         // UART0 receive buffer size
#define UART0_TX_BUFFER_SIZE 16         // UART0 transmit buffer size
#define UART1_RX_BUFFER_SIZE 16         // UART1 receive buffer size
#define UART1_TX_BUFFER_SIZE 16         // UART1 transmit buffer size
///////////////////////////////////////////////////////////////////////////////
// use the following macros to determine the 'baud' parameter values
// for uart0Init() and uart1Init()
// CAUTION - 'baud' SHOULD ALWAYS BE A CONSTANT or
// a lot of code will be generated.
#define UART_BAUD(baud) (uint16_t)(PCLK / 16 / baud - 1)

///////////////////////////////////////////////////////////////////////////////
// Definitions for typical UART 'baud' settings
#define B1200         UART_BAUD(1200)
#define B9600         UART_BAUD(9600)
#define B19200        UART_BAUD(19200)
#define B38400        UART_BAUD(38400)
#define B57600        UART_BAUD(57600)
#define B115200       UART_BAUD(115200)

#ifdef UART0_SUPPORT

ISR(USART0_RX_vect);
ISR(USART0_UDRE_vect);

void uart0Init(const uint16_t baud);

const int16_t uart0Putch(const uint8_t ch);
const uint16_t uart0Space(void);
void uart0Puts(const char* string);
void uart0Write(const char * buffer, uint16_t count);
const int16_t uart0Getch(void);

#endif

#endif /* UART_H_ */