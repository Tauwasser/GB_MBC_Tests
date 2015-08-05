#define A_MASK  (1u << PA2 | 1u << PA1 | 1u << PA0)
#define A_SHIFT PA0

#define D_MASK  (1u << PA7 | 1u << PA6 | 1u << PA5 | 1u << PA4 | 1u << PA3)
#define D_SHIFT PA3

#define RA_MASK  (1u << PC4 | 1u << PC3 | 1u << PC2 | 1u << PC1 | 1u << PC0)
#define RA_SHIFT PC0
#define AA_MASK  (1u << PC6 | 1u << PC5)
#define AA_SHIFT PC5
#define nRAM_CS  (1u << PC7)

#define nWR (1u << PE2)
#define nRD (1u << PE1)
#define nCS (1u << PE0)

#define nRST    (1u << PD5)
#define nROM_CS (1u << PD6)
#define RAM_CS  (1u << PD7)

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
	IOPORTD->out &= ~nRST;
	IOPORTD->dir |= nRST;
}

inline void deassertRST(void) {
	IOPORTD->dir &= ~nRST;
	IOPORTD->out |= nRST;
}

inline void putAddr(uint16_t addr) {
	IOPORTA->out = (IOPORTA->out & D_MASK) | ((addr >> 13) & A_MASK);
}

inline void putAddrCS(uint16_t addr) {
	IOPORTA->out = (IOPORTA->out & D_MASK) | ((addr >> 13) & A_MASK);
	deassertCS();
	if (addr >= 0xA000u && addr <= 0xFF00u)
		assertCS();
}

inline void putData(uint8_t data) {
	IOPORTA->out = ((data << D_SHIFT) & D_MASK) | (IOPORTA->out & A_MASK);
	IOPORTA->dir = D_MASK | A_MASK;
};

inline void floatData(void) {
	IOPORTA->out = D_MASK | (IOPORTA->out & A_MASK);
	IOPORTA->dir = A_MASK;
}

inline uint8_t getRA(void) {
	return (IOPORTC->in & RA_MASK) >> RA_SHIFT;
}

inline uint8_t getAA(void) {
	return (IOPORTC->in & AA_MASK) >> AA_SHIFT;
}

inline uint8_t getnRAM_CS(void) {
	return (IOPORTC->in & nRAM_CS);
}

inline uint8_t getRAM_CS(void) {
	return (IOPORTD->in & RAM_CS);
}

inline uint8_t getnROM_CS(void) {
	return (IOPORTD->in & nROM_CS);
}

void writeMBC1(uint16_t addr, uint8_t data) {
	
	putAddrCS(addr);
	putData(data);
	
	assertWR();
	delay_us(1);
	deassertWR();
	deassertCS();
	floatData();
	
}

void write_usart_hex(const uint8_t num) {
	
	const char alpha[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	
	while (uart0Putch(alpha[(num >> 4)]) < 0);
	while (uart0Putch(alpha[(num & 0x0Fu)]) < 0);
	
}

void printMBC1Status(void) {
	
	uart0Puts("AA: 0x");
	write_usart_hex(getAA());
	uart0Puts(" RA: 0x");
	write_usart_hex(getRA());
	uart0Puts("\n");
}

void printnROM_CS(void){
	uart0Puts(" /// nROM_CS: ");
	getnROM_CS() ? uart0Puts("1") : uart0Puts("0");
}

void print_bothRAM_CS(void){
	uart0Puts(" /// nRAM_CS: ");
	getnRAM_CS() ? uart0Puts("1") : uart0Puts("0");
	uart0Puts(" RAM_CS: ");
	getRAM_CS() ? uart0Puts("1") : uart0Puts("0");
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

const char str0[] PROGMEM = "Drive nROM_CS '0', RAM_CS '1'.\n";
const char str1[] PROGMEM = "Float RAM_CS, nROM_CS.\n";
const char str2[] PROGMEM = "Disable RAM, RAM Bank 0x01, Mode 0 (16Mb/8kB)\n";
const char str3[] PROGMEM = "Enable RAM, RAM Bank 0x01, Mode 0 (16Mb/8kB)\n";
const char str4[] PROGMEM = "Disable RAM, RAM Bank 0x01, Mode 1 (4Mb/32kB)\n";
const char str5[] PROGMEM = "Enable RAM, RAM Bank 0x01, Mode 1 (4Mb/32kB)\n";
const char str6[] PROGMEM = "Latch Test\n";
const char str7[] PROGMEM = "Reg vs Latch\n";

PGM_P const str_tbl[] PROGMEM = {
	str0,
	str1,
	str2,
	str3,
	str4,
	str5,
	str6,
	str7,
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
	
	// PU on D4..D0, drive A15..A13
	IOPORTA->out = D_MASK;
	IOPORTA->dir = A_MASK;
	
	// PU all pins, except LED
	IOPORTB->out = ~(1u << PB0);
	IOPORTB->dir = (1u << PB0);
	
	// PU on nRAM_CS, AA14..AA13, RA18..RA14
	IOPORTC->out = 0xFFu;
	IOPORTC->dir = 0x00u;
	
	// PU nRST, PU unused, PU RXD
	IOPORTD->out = nRST | ~(RAM_CS | nROM_CS) | (1u << PD0);
	IOPORTD->dir = (1u << PD1);
	
	// Drive nWR, nRD, nCS
	IOPORTE->out = nWR | nRD | nCS;
	IOPORTE->dir = nWR | nRD | nCS;
	
	uart0Init(UART_BAUD(125000u));
	
	sei();
	
	uart0Puts("MBC1 Test\n");
	
    while(1)
    {
		
		while ((command = uart0Getch()) == -1);
		
		switch (command) {
		
		case 'u':
			// Unknown Pins drive test
			IOPORTD->out |= RAM_CS;
			IOPORTD->dir |= RAM_CS | nROM_CS;
			uart0Puts_p(0);
			while (uart0Getch() == -1);
			
			IOPORTD->out &= ~RAM_CS;
			IOPORTD->dir &= ~(RAM_CS | nROM_CS);
			uart0Puts_p(1);
			break;
		
		case 't':
			
			assertRST();
			delay_us(1);
			deassertRST();
			
			for (i = 0x00u; i < 0x20u; i++) {
				
				writeMBC1(0x2000u, i);
				
				putAddrCS(0x0000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB0: ");
				printMBC1Status();
				
				putAddrCS(0x4000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB1: ");
				printMBC1Status();
				
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
			// Test (n)RAM_CS
			uart0Puts_p(2);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMBC1(0x4000u, 0x01u);
				writeMBC1(0x0000u, 0x00u);
				writeMBC1(0x6000u, 0x00u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(3);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMBC1(0x4000u, 0x01u);
				writeMBC1(0x0000u, 0x0Au);
				writeMBC1(0x6000u, 0x00u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(4);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMBC1(0x4000u, 0x01u);
				writeMBC1(0x0000u, 0x00u);
				writeMBC1(0x6000u, 0x01u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(5);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMBC1(0x4000u, 0x01u);
				writeMBC1(0x0000u, 0x0Au);
				writeMBC1(0x6000u, 0x01u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			break;
		
		case 'c':
		
			uart0Puts_p(6);
			
			assertRST();
			deassertRST();
			
			// Mode 4M/256k
			writeMBC1(0x6000u, 0x01u);
			
			putAddr(0x4000u);
			putData(0x02u);
			printMBC1Status();
			assertWR();
			printMBC1Status();
			deassertWR();
			printMBC1Status();
			floatData();
			break;
		
		case 'd':
		
			uart0Puts_p(7);
			
			assertRST();
			deassertRST();
			
			// Mode 4M/256k
			writeMBC1(0x6000u, 0x01u);
			
			putAddr(0x4000u);
			putData(0x02u);
			printMBC1Status();
			assertWR();
			printMBC1Status();
			putAddr(0xC000u);
			printMBC1Status();
			floatData();
			deassertWR();
			printMBC1Status();
			break;
			
		case 'e':
		
			for (addr = 0x0000u; ; addr += 0x2000) {
				
				assertRST();
				deassertRST();
				
				writeMBC1(addr, 0x15u);
				putAddr(0x0000);
				printMBC1Status();
				putAddr(0x4000);
				printMBC1Status();
				
				if (addr == 0xE000u)
					break;
				
			}
			
			break;
			
		case 'f':
		
			// Test nROM_CS while RST assert
			assertRST();
			for (i = 0; i < sizeof(perm); i++) {
								
				IOPORTE->out = perm[i];
				
				printCS_Data(&printMBC1Status);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			break;
		
		case 'g':
		
			assertRST();
			writeMBC1(0x4000u, 0x01u);
			writeMBC1(0x0000u, 0x0Au);
			writeMBC1(0x6000u, 0x00u);
			
			for (i = 0; i < sizeof(perm); i++) {
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
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