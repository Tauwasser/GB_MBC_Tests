/*           7          6        5       4       3       2       1       0 
 * Port A:  #RESET     D6       D5      D4      D3      D2      D1      D0
 * Port B:             RA22    RA21    AA16    AA15    AA14    AA13     LED
 * Port C:             RA20    RA19    RA18    RA17    RA16    RA15    RA14
 * Port D:  A15        A14     A13     RAM_CS #RAM_CS #ROM_CS
 * Port E:                                             nWR     nRD     nCS 
 *
 */

#define A_MASK  (1u << PD7 | 1u << PD6 | 1u << PD5)
#define A_SHIFT PD5

#define D_MASK  (1u << PA6 | 1u << PA5 | 1u << PA4 | 1u << PA3| 1u << PA2 | 1u << PA1 | 1u << PA0)
#define D_SHIFT PA0

#define RA_MASK  (1u << PC6 | 1u << PC5 | 1u << PC4 | 1u << PC3 | 1u << PC2 | 1u << PC1 | 1u << PC0)
#define RA_SHIFT PC0
#define RA_HI_MASK  (1u << PB6 | 1u << PB5)
#define RA_HI_SHIFT PB5
#define AA_MASK  (1u << PB4 | 1u << PB3 | 1u << PB2 | 1u << PB1)
#define AA_SHIFT PB1

#define nWR (1u << PE2)
#define nRD (1u << PE1)
#define nCS (1u << PE0)

#define nRST    (1u << PA7)
#define nROM_CS (1u << PD2)
#define nRAM_CS (1u << PD3)
#define RAM_CS  (1u << PD4)

#include <stdarg.h>

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
	IOPORTA->out &= ~nRST;
	IOPORTA->dir |= nRST;
}

inline void deassertRST(void) {
	IOPORTA->dir &= ~nRST;
	IOPORTA->out |= nRST;
}

inline void putAddr(uint16_t addr) {
	IOPORTD->out = (IOPORTD->out & ~A_MASK) | ((addr >> 8) & A_MASK);
}

inline void putAddrCS(uint16_t addr) {
	IOPORTD->out = (IOPORTD->out & ~A_MASK) | ((addr >> 8) & A_MASK);
	if (addr >= 0xA000u && addr <= 0xFF00u)
		assertCS();
	else
		deassertCS();
}

inline void putData(uint8_t data) {
	IOPORTA->out = ((data << D_SHIFT) & D_MASK) | (IOPORTA->out & ~D_MASK);
	IOPORTA->dir |= D_MASK;
};

inline void floatData(void) {
	IOPORTA->dir &= ~D_MASK;
}

inline uint16_t getRA(void) {
	return ((IOPORTC->in & RA_MASK) >> RA_SHIFT) | (((IOPORTB->in & RA_HI_MASK) >> RA_HI_SHIFT) << 7);
}

inline uint8_t getAA(void) {
	return (IOPORTB->in & AA_MASK) >> AA_SHIFT;
}

inline uint8_t getnRAM_CS(void) {
	return (IOPORTD->in & nRAM_CS);
}

inline uint8_t getRAM_CS(void) {
	return (IOPORTD->in & RAM_CS);
}

inline uint8_t getnROM_CS(void) {
	return (IOPORTD->in & nROM_CS);
}

void writeMMM01(uint16_t addr, uint8_t data) {
	
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

void printMMM01Status(void) {
	
	uint16_t ra = getRA();
	
	uart0Puts("AA: 0x");
	write_usart_hex(getAA());
	uart0Puts(" RA: 0x");
	write_usart_hex((ra >> 8) & 0xFFu);
	write_usart_hex(ra & 0xFFu);
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

const char str0[] PROGMEM = "> Pin Status\n";
const char str1[] PROGMEM = "Float RAM_CS, nROM_CS.\n";
const char str2[] PROGMEM = "Disable RAM, RAM Bank 0x01, Mode 0 (16Mb/8kB)\n";
const char str3[] PROGMEM = "Enable RAM, RAM Bank 0x01, Mode 0 (16Mb/8kB)\n";
const char str4[] PROGMEM = "Disable RAM, RAM Bank 0x01, Mode 1 (4Mb/32kB)\n";
const char str5[] PROGMEM = "Enable RAM, RAM Bank 0x01, Mode 1 (4Mb/32kB)\n";
const char str6[] PROGMEM = "Latch Test\n";
const char str7[] PROGMEM = "Reg vs Latch\n";
const char str8[] PROGMEM = "> Enable SRAM\n";
const char str9[] PROGMEM = "> Disable SRAM\n";
const char str10[] PROGMEM = "> Assert RESET\n";
const char str11[] PROGMEM = "> Deassert RESET\n";
const char str12[] PROGMEM = "> MMM01 Settings\n R0 0x%h\n R1 0x%h\n R2 0x%h\n R3 0x%h\n";
const char str13[] PROGMEM = "> 0x%H <= %h\n";
const char str14[] PROGMEM = "> 0x%H: AA 0x%h RA 0x%H expect AA 0x%h RA 0x%H\n";

PGM_P const str_tbl[] PROGMEM = {
	str0,
	str1,
	str2,
	str3,
	str4,
	str5,
	str6,
	str7,
	str8,
	str9,
	str10,
	str11,
	str12,
	str13,
	str14
};

void uart0Puts_p(const uint8_t ix) {
	
	register const char* ptr = (PGM_P) pgm_read_word(&str_tbl[ix]);
	register uint8_t ch = 0xFFu;
	
	while (ch = pgm_read_byte(ptr)) {
		while (uart0Putch(ch) < 0);
		ptr++;
	}
	
}

void uart0Printf(const char *ptr, ...) {
	
	register uint8_t ch = 0xFFu;
	uint16_t tmp;
	va_list ap;
	va_start(ap, ptr);
	
	while (ch = *ptr) {
		
		if (ch == '%') {
			ptr++;
			ch = pgm_read_byte(ptr++);
			
			switch (ch) {
				case 'h':
				tmp = va_arg(ap, int);
				write_usart_hex(tmp);
				break;
				case 'H':
				tmp = va_arg(ap, int);
				write_usart_hex(tmp >> 8);
				write_usart_hex(tmp & 0xFFu);
				break;
				case 'b':
				tmp = va_arg(ap, int);
				if (tmp)
				while (uart0Putch('1') < 0);
				else
				while (uart0Putch('0') < 0);
				break;
				case '%':
				while (uart0Putch('%') < 0);
				break;
				default:
				while (uart0Putch('%') < 0);
				while (uart0Putch(ch) < 0);
				if (ch == '0')
				ptr--;
				break;
			}
			
			continue;
		}
		
		while (uart0Putch(ch) < 0);
		ptr++;
	}
	va_end(ap);
	
}

void uart0Printf_p(const uint8_t ix, ...) {
	
	register const char* ptr = (PGM_P) pgm_read_word(&str_tbl[ix]);
	register uint8_t ch = 0xFFu;
	uint16_t tmp;
	va_list ap;
	va_start(ap, ix);
	
	while (ch = pgm_read_byte(ptr)) {
		
		if (ch == '%') {
			ptr++;
			ch = pgm_read_byte(ptr++);
			
			switch (ch) {
			case 'h':
				tmp = va_arg(ap, int);
				write_usart_hex(tmp);
				break;
			case 'H':
				tmp = va_arg(ap, int);
				write_usart_hex(tmp >> 8);
				write_usart_hex(tmp & 0xFFu);
				break;
			case 'b':
				tmp = va_arg(ap, int);
				if (tmp)
					while (uart0Putch('1') < 0);
				else
					while (uart0Putch('0') < 0);
				break;
			case '%':
				while (uart0Putch('%') < 0);
				break;
			default:
				while (uart0Putch('%') < 0);
				while (uart0Putch(ch) < 0);
				if (ch == '0')
					ptr--;
				break;
			}
			
			continue;
		}
		
		while (uart0Putch(ch) < 0);
		ptr++;
	}
	va_end(ap);
	
}

struct mmm01_settings {
	/* R0 */
	uint8_t ram_lock  : 4;
	uint8_t ramb_mask : 2;
	uint8_t latch     : 1;
	uint8_t zero3     : 1;
	/* R1 */
	uint8_t romb      : 5;
	uint8_t romb_mid  : 2;
	uint8_t zero2     : 1;
	/* R2 */
	uint8_t ramb      : 2;
	uint8_t ramb_hi   : 2;
	uint8_t romb_hi   : 2;
	uint8_t mode_lock : 1;
	uint8_t zero1     : 1;
	/* R3 */
	uint8_t mode      : 1;
	uint8_t unk       : 1;
	uint8_t romb_mask : 4;
	uint8_t mux       : 1;
	uint8_t zero0     : 1;
};

void print_MMM01_Settings(struct mmm01_settings *s) {
	uint8_t *p = (uint8_t*) s;
	uart0Printf_p(12, p[0], p[1], p[2], p[3]);
}

uint16_t mmm01_check_getRA(struct mmm01_settings *s, uint16_t addr, uint8_t romb, uint8_t ramb) {
	
	/* MBC1 Mode                     4Mbit: keep                16Mbit: zero */
	uint8_t ramb_masked = (s->mode ? (ramb & 0x03u) & ~(s->ramb_mask) : 0x00u);
	uint8_t romb_masked = (addr & (1 << 14) ? (romb & 0x1Fu) & ~(s->romb_mask << 1) : 0x00u);
	/*                       Mux: RA22..RA21 |                                    AA14..AA13 | ROMBASE (pre-map ROMB & ROMB_MASK)   Reg:  RA22..RA21      |       RA20..RA19 | ROMBASE (pre-map ROMB & ROMB_MASK) */
	uint16_t ra = (s->mux ? (s->romb_hi << 7 | ((s->ramb & s->ramb_mask) | ramb_masked) << 5 | (s->romb & s->romb_mask << 1))          : (s->romb_hi << 7 | s->romb_mid << 5 | (s->romb & s->romb_mask << 1)));
	return ra | romb_masked;
}

uint8_t mmm01_check_getAA(struct mmm01_settings *s, uint16_t addr, uint8_t romb, uint8_t ramb) {
	
	/* MBC1 Mode                     4Mbit: keep                16Mbit: zero */
	uint8_t ramb_masked = (s->mode ? (ramb & 0x03u) & ~(s->ramb_mask) : 0x00u);
	/*                  Mux:     AA16..AA15 | RA20..RA19  Reg:        AA16..AA15 | RAMBASE | AA14..AA13 */
	uint8_t aa = (s->mux ? (s->ramb_hi << 2 | s->romb_mid)   :  (s->ramb_hi << 2 | s->ramb | ramb_masked));
	return aa;
}

int mmm01_check_map(struct mmm01_settings *s) {
	
	uint8_t aa;
	uint8_t ext_aa;
	uint16_t ra;
	uint16_t ext_ra;
	uint8_t err;
	
	/* check initial settings after map */
	putAddrCS(0x0000);
	err = 0x00u;
	ext_aa = getAA();
	ext_ra = getRA();
	aa = mmm01_check_getAA(s, 0x0000u, s->romb, s->ramb);
	ra = mmm01_check_getRA(s, 0x0000u, s->romb, s->ramb);
	
	uart0Printf_p(14, 0x0000, ext_aa, ext_ra, aa, ra);
		
	err |= ext_aa != aa;
	err |= ext_ra != ra;
		
	if (err)
		return err;
		
	return 0;
	
}

int main(void)
{
	
	int16_t command;
	int16_t i;
	int16_t j;
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
	
	// no PU on D6..D0 (MMM01 has PD), drive nRST
	IOPORTA->out = nRST;
	IOPORTA->dir = nRST;
	
	// PU all pins, except LED
	IOPORTB->out = ~(1u << PB0);
	IOPORTB->dir = (1u << PB0);
	
	// PU on all pins
	IOPORTC->out = 0xFFu;
	IOPORTC->dir = 0x00u;
	
	// PU unused, PU RXD, drive TXD, A15..A13
	IOPORTD->out = 0xFE | (1u << PD0);
	IOPORTD->dir = (1u << PD1) | A_MASK;
	
	// Drive nWR, nRD, nCS
	IOPORTE->out = nWR | nRD | nCS;
	IOPORTE->dir = nWR | nRD | nCS;
	
	uart0Init(UART_BAUD(125000u));
	
	sei();
	
	uart0Puts("MMM01 Test\n");
	
    while(1)
    {
		
		while ((command = uart0Getch()) == -1);
		
		switch (command) {
		
		case 'T':
			
			// Get Pin Status
			uart0Puts_p(0);
			
			for (addr = 0x0000u; addr <= 0xE000; addr+= 0x2000u) {
		
				putAddrCS(addr);
				uart0Puts("0x");
				write_usart_hex((addr >> 8) & 0xFFu);
				write_usart_hex(addr & 0xFFu);
				uart0Puts(": ");
				printMMM01Status();
				printnROM_CS();
				print_bothRAM_CS();
				
				uart0Puts("\n");
				
				if (addr == 0xE000)
					break;
			}
			break;
			
		case 'A':
		
			while ((i = uart0Getch()) == -1);
			putAddrCS(i << 8);
			break;
			
		case 'U':
		
			uart0Puts_p(8);
			writeMMM01(0x0000, 0x0A);
			break;
			
		case 'u':
		
			uart0Puts_p(9);
			writeMMM01(0x0000, 0x00);
			break;
		
		case 'R':
		
			assertRD();
			break;
			
		case 'r':
		
			deassertRD();
			break;
		
		case 'S':
		
			uart0Puts_p(10);
			assertRST();
			break;
		
		case 's':
		
			uart0Puts_p(11);
			deassertRST();
			break;
		
		case 'p':
		
			while ((i = uart0Getch()) == -1);
			while ((j = uart0Getch()) == -1);
			addr = i << 8;
			
			uart0Printf_p(13, addr, j);
			writeMMM01(addr, j);
			break;
			
		case 't':
			
			assertRST();
			delay_us(1);
			deassertRST();
			
			for (i = 0x00u; i < 0x20u; i++) {
				
				writeMMM01(0x2000u, i);
				
				putAddrCS(0x0000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB0: ");
				printMMM01Status();
				uart0Puts("\n");
				
				putAddrCS(0x4000u);
				uart0Puts("RB 0x");
				write_usart_hex(i);
				uart0Puts(" RB1: ");
				printMMM01Status();
				uart0Puts("\n");
				
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
				
				writeMMM01(0x4000u, 0x01u);
				writeMMM01(0x0000u, 0x00u);
				writeMMM01(0x6000u, 0x00u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(3);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMMM01(0x4000u, 0x01u);
				writeMMM01(0x0000u, 0x0Au);
				writeMMM01(0x6000u, 0x00u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(4);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMMM01(0x4000u, 0x01u);
				writeMMM01(0x0000u, 0x00u);
				writeMMM01(0x6000u, 0x01u);
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			
			uart0Puts_p(5);
			for (i = 0; i < sizeof(perm); i++) {
				
				assertRST();
				delay_us(1);
				deassertRST();
				
				writeMMM01(0x4000u, 0x01u);
				writeMMM01(0x0000u, 0x0Au);
				writeMMM01(0x6000u, 0x01u);
				
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
			writeMMM01(0x6000u, 0x01u);
			
			putAddr(0x4000u);
			putData(0x02u);
			printMMM01Status();
			assertWR();
			printMMM01Status();
			deassertWR();
			printMMM01Status();
			floatData();
			break;
		
		case 'd':
		
			uart0Puts_p(7);
			
			assertRST();
			deassertRST();
			
			// Mode 4M/256k
			writeMMM01(0x6000u, 0x01u);
			
			putAddr(0x4000u);
			putData(0x02u);
			printMMM01Status();
			assertWR();
			printMMM01Status();
			putAddr(0xC000u);
			printMMM01Status();
			floatData();
			deassertWR();
			printMMM01Status();
			break;
			
		case 'e':
		
			for (addr = 0x0000u; ; addr += 0x2000) {
				
				assertRST();
				deassertRST();
				
				writeMMM01(addr, 0x15u);
				putAddr(0x0000);
				printMMM01Status();
				putAddr(0x4000);
				printMMM01Status();
				
				if (addr == 0xE000u)
					break;
				
			}
			
			break;
			
		case 'f':
		
			// Test nROM_CS while RST assert
			assertRST();
			for (i = 0; i < sizeof(perm); i++) {
								
				IOPORTE->out = perm[i];
				
				printCS_Data(&printMMM01Status);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
			break;
		
		case 'g':
		
			assertRST();
			writeMMM01(0x4000u, 0x01u);
			writeMMM01(0x0000u, 0x0Au);
			writeMMM01(0x6000u, 0x00u);
			
			for (i = 0; i < sizeof(perm); i++) {
				
				IOPORTE->out = perm[i];
				
				printCS_Data(&print_bothRAM_CS);
				
				IOPORTE->out = nWR | nRD | nCS;
				
			}
		
			break;
		
		case 'x':
			{
				struct mmm01_settings settings;
				memset((void*)&settings, 0x00, sizeof(settings));
				settings.ram_lock = 0x0Au;
				settings.latch = 0x01u;
				settings.ramb_mask = 0x02u;
				
				settings.romb = 0x15u;
				settings.romb_mid = 0x02u;
				
				settings.ramb = 0x03u;
				settings.ramb_hi = 0x01u;
				settings.romb_hi = 0x01u;
				settings.mode_lock = 0x00u;
				
				settings.mode = 0x01u;
				settings.romb_mask = 0x08u;
				settings.mux = 0x00u;
				
				uint8_t *ptr = (uint8_t*) &settings;
				print_MMM01_Settings(&settings);
				
				writeMMM01(0x2000, ptr[1]);
				writeMMM01(0x6000, ptr[3]);
				writeMMM01(0x4000, ptr[2]);
				writeMMM01(0x0000, ptr[0]);
				
				if (mmm01_check_map(&settings))
					uart0Puts("Fail");
				else
					uart0Puts("Pass");
				
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