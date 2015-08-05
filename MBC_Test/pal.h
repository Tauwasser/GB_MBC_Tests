#ifndef REGISTERS_H_
#define REGISTERS_H_

#include <stddef.h>
#include <inttypes.h>

struct avr_gpio {

	uint8_t in;
	uint8_t dir;
	uint8_t out;
	
};

#define IOPORTA ((volatile struct avr_gpio*)&PINA)
#define IOPORTB ((volatile struct avr_gpio*)&PINB)
#define IOPORTC ((volatile struct avr_gpio*)&PINC)
#define IOPORTD ((volatile struct avr_gpio*)&PIND)
#define IOPORTE ((volatile struct avr_gpio*)&PINE)

//#define pal_od_set_mask(port, mask) (port->dir &= ~mask)
//#define pal_od_reset_mask(port, mask) (port->dir |= mask)

#define pal_od_set_mask(port, mask) \
	{ \
	volatile struct avr_gpio* tmp_ptr = port; \
	asm volatile ( \
		"adiw %A0, %1 \r\n" \
		"ld __tmp_reg__, %a0 \r\n" \
		"and __tmp_reg__, %2 \r\n" \
		"st %a0, __tmp_reg__ \r\n" \
		: "+e" (tmp_ptr) \
		: "I" (offsetof(struct avr_gpio, dir)) , "r" ((uint8_t) ~mask) \
		: "cc" \
		); \
	}

#define pal_od_reset_mask(port, mask) \
	{ \
	volatile struct avr_gpio* tmp_ptr = port; \
	asm volatile ( \
		"adiw %A0, %1 \r\n" \
		"ld __tmp_reg__, %a0 \r\n" \
		"or __tmp_reg__, %2 \r\n" \
		"st %a0, __tmp_reg__ \r\n" \
		: "+e" (tmp_ptr) \
		: "I" (offsetof(struct avr_gpio, dir)) , "r" (mask) \
		: "cc" \
		); \
	}

#define pal_od_write_mask(port, mask, bit) \
	pal_od_set_mask(port, mask); \
	if (!bit) \
		pal_od_reset_mask(port, mask);

#endif /* REGISTERS_H_ */