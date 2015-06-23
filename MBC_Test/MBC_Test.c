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
	IOPORTE->out &= ~(nCS); // disable PU
	IOPORTE->dir |= nCS;    // drive '0'
}

inline void deassertCS(void) {
	IOPORTE->out |= nCS;    // drive '1'
	IOPORTE->dir &= ~(nCS); // enable PU
}

inline void assertWR(void) {
	IOPORTE->out &= ~(nWR); // disable PU
	IOPORTE->dir |= nWR;    // drive '0'
}

inline void deassertWR(void) {
	IOPORTE->out |= nWR;    // drive '1'
	IOPORTE->dir &= ~(nWR); // enable PU
}

inline void assertRD(void) {
	IOPORTE->out &= ~(nRD); // disable PU
	IOPORTE->dir |= nRD;    // drive '0'
}

inline void deassertRD(void) {
	IOPORTE->out |= nRD;    // drive '1'
	IOPORTE->dir &= ~(nRD); // enable PU
}

inline void assertRST(void) {
	IOPORTC->out &= ~nRST;  // disable PU
	IOPORTC->dir |= nRST;   // drive '0'
}

inline void deassertRST(void) {
	IOPORTC->dir &= ~nRST;  // float
	IOPORTC->out |= nRST;   // enable PU
}

inline void putAddr(uint16_t addr) {
	uint8_t addr_tmp = (addr >> 13) & 0x06; // mask A15 A14
	if (addr & 0x0100)
		addr_tmp |= 0x01u; // or A8
	
	uint8_t addr_l = (addr << A_LO_SHIFT) & A_LO_MASK;
	uint8_t addr_m = ((addr >> 6) << A_MID_SHIFT) & A_MID_MASK;
	uint8_t addr_h = (addr_tmp << A_HI_SHIFT) & A_HI_MASK;
	
	// addr_l
	IOPORTD->out = addr_l | (IOPORTD->out & ~A_LO_MASK); // put PUs in place
	IOPORTD->dir |= A_LO_MASK;                           // help bus up
	IOPORTD->dir = ~addr_l;                              // only drive '0' bits
	
	// addr_m
	IOPORTC->out = addr_m | (IOPORTC->out & ~A_MID_MASK); // put PUs in place
	IOPORTC->dir |= A_MID_MASK;                           // help bus up
	IOPORTC->dir &= ~addr_m;                              // drive '0' bits
	
	// addr_h
	IOPORTA->out = addr_h | (IOPORTA->out & ~A_HI_MASK); // put PUs in place
	IOPORTA->dir |= A_HI_MASK;                           // help bus up
	IOPORTA->dir &= ~addr_h;                             // drive '0' bits
	
}

inline void putAddrCS(uint16_t addr) {
	putAddr(addr);
	deassertCS();
	if (addr >= 0xA000u && addr <= 0xFF00u)
		assertCS();
}

inline void putData(uint8_t data) {
	uint8_t d = (data << D_SHIFT) & D_MASK;
	IOPORTA->out = d | (IOPORTA->out & A_HI_MASK); // put PUs in place
	IOPORTA->dir |= D_MASK;                        // help bus up
	IOPORTA->dir &= ~d;                            // drive '0' bits
};

inline void floatData(void) {
	IOPORTA->out |= D_MASK;  // PU
	IOPORTA->dir &= ~D_MASK; // float
}

inline void floatAddr(void) {
	IOPORTA->out |= A_HI_MASK;  // PU
	IOPORTA->dir &= ~A_HI_MASK; // float
	
	IOPORTC->out |= A_MID_MASK;  // PU
	IOPORTC->dir &= ~A_MID_MASK; // float
	
	IOPORTD->out |= A_LO_MASK;  // PU
	IOPORTD->dir &= ~A_LO_MASK; // float
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
	
	// PU unused, PU on D3..D0, PU A15..A14, A8
	IOPORTA->out = D_MASK | (1u << PA4) | A_HI_MASK;
	IOPORTA->dir = 0x00;
	
	// PU all pins, except LED
	IOPORTB->out = ~(1u << PB0);
	IOPORTB->dir = (1u << PB0);
	
	// PU on nROM_CS, #RESET, RA17..RA14, PU A7..A6
	IOPORTC->out = RA_MASK | nRST | nROM_CS | A_MID_MASK;
	IOPORTC->dir = 0x00u;
	
	// PU all pins
	IOPORTD->out = 0xFFu;
	IOPORTD->dir = 0x00u;
	
	// PU nWR, nRD, nCS
	IOPORTE->out = nWR | nRD | nCS;
	IOPORTE->dir = 0x00u;
	
	uart0Init(UART_BAUD(125000u));
	
	sei();
	
	uart0Puts("MBC2 Test\n");
	
    while(1)
    {
		
		while ((command = uart0Getch()) == -1);
		
		switch (command) {
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
				
				IOPORTE->out = perm[i];  // PU
				IOPORTE->dir = ~perm[i]; // drive '0'
				
				printCS_Data(&printnROM_CS);
				
				IOPORTE->out = nWR | nRD | nCS; // drive '1'
				IOPORTE->dir = 0x00u;           // float
				
			}
			
			break;
			
		case 'b':
		
			// Test nROM_CS while RST assert
			assertRST();
			for (i = 0; i < sizeof(perm); i++) {
								
				IOPORTE->out = perm[i];  // PU
				IOPORTE->dir = ~perm[i]; // drive '0'
				
				printCS_Data(&printnROM_CS);
				
				IOPORTE->out = nWR | nRD | nCS; // drive '1'
				IOPORTE->dir = 0x00u;           // float
				
			}
			break;

		case 'c':
	
			// test RAM
			// write IDs to power-of-two locations
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
			
			// init RAM to all 0x0F
			
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
			
			// Write to RAM
			// f [hi] [lo] [data]
			// hi & lo = 0000 - 0x3FF (allow trying to write out of bounds)
			
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
			
			// Test which Address bits affect RB0 / RB1 mapping
			// include A8
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
			
		case 'k':
		
			// float all pins
			IOPORTA->dir &= ~A_HI_MASK & ~D_MASK;
			IOPORTA->out &= ~A_HI_MASK & ~D_MASK;
			
			IOPORTC->dir &= ~RA_MASK & ~A_MID_MASK & ~nROM_CS & ~nRST;
			IOPORTC->out &= ~RA_MASK & ~A_MID_MASK & ~nROM_CS & ~nRST;
			
			IOPORTD->dir &= ~A_LO_MASK;
			IOPORTD->out &= ~A_LO_MASK;
			
			IOPORTE->dir &= ~nCS & ~nRD & ~nWR;
			IOPORTE->out &= ~nCS & ~nRD & ~nWR;
			
			break;
		
		case 'l':
		
			// all signals back to default
			IOPORTA->out |= A_HI_MASK | D_MASK;
			IOPORTA->dir &= ~A_HI_MASK & ~D_MASK;
			
			deassertRST();
			IOPORTC->out |= RA_MASK | A_MID_MASK | nROM_CS;
			IOPORTC->dir |= ~RA_MASK & ~A_MID_MASK & ~nROM_CS;
		
			IOPORTD->out |= A_LO_MASK;
			IOPORTD->dir &= ~A_LO_MASK;
			
			IOPORTE->out |= nCS | nRD | nWR;
			IOPORTE->dir &= ~nCS & ~nRD & ~nWR;
		
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