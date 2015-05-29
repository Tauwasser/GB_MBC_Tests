/*           7         6        5       4       3       2       1       0 
 * Port A:  A15       A14      A8              D3      D2      D1      D0 
 * Port B:                                                             LED
 * Port C:  #ROM_CS #RESET     A7      A6     RA17    RA16    RA15    RA14
 * Port D:  A5        A4       A3      A2      A1      A0     TXD     RXD 
 * Port E:                                            nWR     nRD     nCS 
 *
 */

#define A_HI_MASK  (1u << PA7 | 1u << PA6 | 1u << PA5)
#define A_HI_SHIFT PA5

#define D_MASK  (1u << PA3 | 1u << PA2 | 1u << PA1 | 1u << PA0)
#define D_SHIFT PA0

#define RA_MASK  (1u << PC3 | 1u << PC2 | 1u << PC1 | 1u << PC0)
#define RA_SHIFT PC0
#define A_MID_MASK  (1u << PC5 | 1u << PC4)
#define A_MID_SHIFT PC4

#define nROM_CS  (1u << PC7)
#define nRST     (1u << PC6)

#define nWR (1u << PE2)
#define nRD (1u << PE1)
#define nCS (1u << PE0)

#define A_LO_MASK  (1u << PD7 | 1u << PD6 | 1u << PD5 | 1u << PD4 | 1u << PD3 | 1u << PD2)
#define A_LO_SHIFT PD2

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "pal.h"
#include "uart.h"
#include "delay_.h"

inline void assertCS(void) {
	IOPORTE->out &= ~(nCS);
}

inline void deassertCS(void) {
	IOPORTE->out |= nCS;
}

inline void assertWR(void) {
	IOPORTE->out &= ~(nWR);
}

inline void deassertWR(void) {
	IOPORTE->out |= nWR;
}

inline void assertRD(void) {
	IOPORTE->out &= ~(nRD);
}

inline void deassertRD(void) {
	IOPORTE->out |= nRD;
}

inline void assertRST(void) {
	IOPORTC->out &= ~nRST;
	IOPORTC->dir |= nRST;
}

inline void deassertRST(void) {
	IOPORTC->dir &= ~nRST;
	IOPORTC->out |= nRST;
}

inline void putAddr(uint16_t addr) {
	uint8_t addr_tmp = (addr >> 13) & 0x06;
	if (addr & 0x0100)
		addr_tmp |= 0x01u;
	
	IOPORTD->out = ((addr << A_LO_SHIFT) & A_LO_MASK) | (IOPORTD->out & ~A_LO_MASK);
	IOPORTC->out &= ~A_MID_MASK;
	IOPORTC->out |= ((addr >> 6) << A_MID_SHIFT) & A_MID_MASK;
	IOPORTA->out = (IOPORTA->out & D_MASK) | ((addr_tmp << A_HI_SHIFT) & A_HI_MASK);
}

inline void putAddrCS(uint16_t addr) {
	putAddr(addr);
	deassertCS();
	if (addr >= 0xA000u && addr <= 0xFF00u)
		assertCS();
}

inline void putData(uint8_t data) {
	IOPORTA->out = ((data << D_SHIFT) & D_MASK) | (IOPORTA->out & A_HI_MASK);
	IOPORTA->dir = D_MASK | A_HI_MASK;
};

inline void floatData(void) {
	IOPORTA->out = D_MASK | (IOPORTA->out & A_HI_MASK);
	IOPORTA->dir = A_HI_MASK;
}

inline uint8_t getRA(void) {
	return (IOPORTC->in & RA_MASK) >> RA_SHIFT;
}

inline uint8_t getnROM_CS(void) {
	return (IOPORTC->in & nROM_CS);
}

inline uint8_t getData(void) {
	return (IOPORTA->in & D_MASK) >> D_SHIFT;
}

void writeMBC2(uint16_t addr, uint8_t data) {
	
	putAddrCS(addr);
	putData(data);
	
	assertWR();
	delay_us(1);
	deassertWR();
	deassertCS();
	floatData();
	
}

uint8_t readMBC2(uint16_t addr) {
	
	uint8_t tmp;
	
	putAddrCS(addr);
	floatData();
	
	assertRD();
	delay_us(1);
	tmp = getData();
	deassertRD();
	deassertCS();
	floatData();
	
	return tmp;
	
}

void write_usart_hex(const uint8_t num) {
	
	const char alpha[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	
	while (uart0Putch(alpha[(num >> 4)]) < 0);
	while (uart0Putch(alpha[(num & 0x0Fu)]) < 0);
	
}

void printMBC2Status(void) {
	
	uart0Puts(" RA: 0x");
	write_usart_hex(getRA());
	uart0Puts("\n");
}

void printnROM_CS(void){
	uart0Puts(" /// nROM_CS: ");
	getnROM_CS() ? uart0Puts("1") : uart0Puts("0");
}

void printCS_Data(void(*fun)(void)) {
	
	uint16_t addr;
	
	for (addr = 0x0000u; ; addr += 0x2000u) {
				
		putAddr(addr);
		uart0Puts("0x");
		write_usart_hex(addr >> 8);
		write_usart_hex(addr & 0xFFu);
		uart0Puts(" /// nRD: ");
		(IOPORTE->out & nRD) ? uart0Puts("1") : uart0Puts("0");
		uart0Puts(" nWR: ");
		(IOPORTE->out & nWR) ? uart0Puts("1") : uart0Puts("0");
		uart0Puts(" nCS: ");
		(IOPORTE->out & nCS) ? uart0Puts("1") : uart0Puts("0");
		fun();
		uart0Puts("\n");
				
		if (addr == 0xE000u)
			break;
				
	}
	
	uart0Puts("//////////////////\n");
	
}

const char str0[] PROGMEM = "Drive nROM_CS '0'.\n";
const char str1[] PROGMEM = "Float nROM_CS.\n";
const char str2[] PROGMEM = "Init RAM.\n";
const char str3[] PROGMEM = "Print RAM.\n";

PGM_P const str_tbl[] PROGMEM = {
	str0,
	str1,
	str2,
	str3,
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
	uint8_t i;
	uint8_t j;
	uint16_t addr;
	
	uint8_t perm[8] = {
		nWR | nRD | nCS,
		nWR | nRD |   0,
		nWR |   0 | nCS,
		nWR |   0 |   0,
		  0 | nRD | nCS,
		  0 | nRD |   0,
		  0 |   0 | nCS,
		  0 |   0 |   0,
	};
	
	// PU unused, PU on D3..D0, drive A15..A14, A8
	IOPORTA->out = D_MASK | (1u << PA4);
	IOPORTA->dir = A_HI_MASK;
	
	// PU all pins, except LED
	IOPORTB->out = ~(1u << PB0);
	IOPORTB->dir = (1u << PB0);
	
	// PU on nROM_CS, #RESET, RA17..RA14, drive A7..A6
	IOPORTC->out = RA_MASK | nRST | nROM_CS;
	IOPORTC->dir = A_MID_MASK;
	
	// drive all pins
	IOPORTD->out = 0xFFu;
	IOPORTD->dir = 0xFFu;
	
	// Drive nWR, nRD, nCS
	IOPORTE->out = nWR | nRD | nCS;
	IOPORTE->dir = nWR | nRD | nCS;
	
	uart0Init(UART_BAUD(125000u));
	
	sei();
	
	uart0Puts("MBC2 Test\n");
	
    while(1)
    {
		
		while ((command = uart0Getch()) == -1);
		
		switch (command) {
		
		case 'u':
			// Unknown Pins drive test
			IOPORTC->out &= ~nROM_CS;
			IOPORTC->dir |= nROM_CS;
			uart0Puts_p(0);
			while (uart0Getch() == -1);
			
			IOPORTC->out |= nROM_CS;
			IOPORTC->dir &= ~(nROM_CS);
			uart0Puts_p(1);
			break;
		
		case 't':
			
			assertRST();
			delay_us(1);
			deassertRST();
			
			for (i = 0x00u; i < 0x20u; i++) {
				
				writeMBC2(0x2100u, i);
				
				putAddrCS(0x0000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB0: ");
				printMBC2Status();
				
				putAddrCS(0x4000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB1: ");
				printMBC2Status();
				
			}
			
			deassertRD();
			deassertWR();
			deassertCS();
			
			break;
			
		case 'a':
			// Test nROM_CS
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&printnROM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			break;
			
		case 'b':
		
			// Test nROM_CS while RST assert
			assertRST();
			for (i = 0; i < sizeof(perm); i++) {
								
				IOPORTE->out = perm[i];
				
				printCS_Data(&printnROM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			break;

		case 'c':
	
			// test RAM
			uart0Puts_p(2);
	
			assertRST();
			delay_us(1);
			deassertRST();
			
			// Enable SRAM
			writeMBC2(0x0000u, 0x0Au);
			j = 0x01;
			for (addr = 0x0000; addr < 0x0200; addr <<= 1, j++) {
				
				writeMBC2(0xA000 | addr, j);
				
				if (!addr)
					addr = 0x0001u;
			}
			
			// Disable SRAM
			writeMBC2(0x0000u, 0x00u);
			break;

		case 'd':
		
			// read RAM
			uart0Puts_p(3);
			
			assertRST();
			delay_us(1);
			deassertRST();

			// Enable SRAM
			writeMBC2(0x0000u, 0x0Au);
			
			for (i = 0x00; i < 0x20u; i++) {
			
				for (j = 0x00; j < 0x10u; j++) {
					write_usart_hex(readMBC2(0xA000 | (i << 4) | (j)));
					uart0Puts(" ");
				}
				uart0Puts("\n");
				
			}
			
			// Disable SRAM
			writeMBC2(0x0000u, 0x00u);
			
			break;
		
		case 'e':
				
			// test RAM
			uart0Puts_p(2);
				
			assertRST();
			delay_us(1);
			deassertRST();
				
			// Enable SRAM
			writeMBC2(0x0000u, 0x0Au);
				
			for (addr = 0x0000; addr < 0x0200; addr++) {
					
				writeMBC2(0xA000 | addr, 0x0F);
					
			}
				
			// Disable SRAM
			writeMBC2(0x0000u, 0x00u);
			break;
		
		case 'f':
			
			// Enable SRAM
			writeMBC2(0x0000u, 0x0Au);
			
			while ((int8_t)(i = uart0Getch()) == -1);
			while ((int8_t)(j = uart0Getch()) == -1);
			while ((int16_t)(addr = uart0Getch()) == -1);
			
			writeMBC2(0xA000 | ((i & 0x03) << 8) | j, addr & 0xFFu);
			
			// Disable SRAM
			writeMBC2(0x0000u, 0x00u);
			break;
		
		case 'g':
		
			// Test which bits influence RB0, RB1 switch
			assertRST();
			delay_us(1);
			deassertRST();
			
			writeMBC2(0x0100u, 0x15u);
			printCS_Data(&printMBC2Status);
			break;
			
		case 'h':
			
			for (addr = 0x0000u; ; addr += 0x4000u) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				uart0Puts("0x");
				write_usart_hex(addr >> 8);
				write_usart_hex(addr & 0xFFu);
				uart0Puts(" /// ");
				
				putAddr(0x0000u);
				writeMBC2(addr, 0x15u);
				printMBC2Status();
				
				putAddr(0x4000u);
				writeMBC2(addr, 0x15u);
				printMBC2Status();
				
				addr |= 0x0100u;
				
				uart0Puts("0x");
				write_usart_hex(addr >> 8);
				write_usart_hex(addr & 0xFFu);
				uart0Puts(" /// ");
				
				putAddr(0x0000u);
				writeMBC2(addr, 0x15u);
				printMBC2Status();
			
				putAddr(0x4000u);
				writeMBC2(addr, 0x15u);
				printMBC2Status();
				
				if (addr == 0xC100)
					break;
					
				addr &= ~(0x0100u);
				
			}
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