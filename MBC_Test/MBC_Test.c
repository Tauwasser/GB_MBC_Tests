#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "pal.h"
#include "uart.h"
#include "delay_.h"

void write_usart_hex(const uint8_t num) {
	
	const char alpha[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	
	while (uart0Putch(alpha[(num >> 4)]) < 0);
	while (uart0Putch(alpha[(num & 0x0Fu)]) < 0);
	
}

const char str0[] PROGMEM = "Hello, World.\n";

PGM_P const str_tbl[] PROGMEM = {
	str0,
};

void uart0Puts_p(const uint8_t ix) {
	
	register const char* ptr = (PGM_P) pgm_read_word(&str_tbl[ix]);
	register uint8_t ch = 0xFFu;
	
	while (ch = pgm_read_byte(ptr)) {
		while (uart0Putch(ch) < 0);
		ptr++;
	}
	
}

int main(void)
{
	
	int16_t command;
	
	// PU
	IOPORTA->out = 0xFFu;
	IOPORTA->dir = 0x00u;
	
	// PU all pins, except LED
	IOPORTB->out = ~(1u << PB0);
	IOPORTB->dir = (1u << PB0);
	
	// PU
	IOPORTC->out = 0xFFu;
	IOPORTC->dir = 0x00u;
	
	// PU
	IOPORTD->out = 0xFFu;
	IOPORTD->dir = 0x00u;
	
	// PU
	IOPORTE->out = 0xFFu;
	IOPORTE->dir = 0x00u;
	
	uart0Init(UART_BAUD(125000u));
	
	sei();
	
	uart0Puts("MBC1 Test\n");
	
    while(1)
    {
		
		while ((command = uart0Getch()) == -1);
		
		switch (command) {
		
		case 'h':
		
			uart0Puts_p(0);
			break;
		
		case 'L':
			IOPORTB->out ^= (1u << PB0);
			break;
		default:
			uart0Putch(command);
			break;
		}
		
    }
}