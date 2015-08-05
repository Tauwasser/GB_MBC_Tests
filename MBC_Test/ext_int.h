#ifndef EXT_INT_H_
#define EXT_INT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define EXTINT_LEVEL        (0u)
#define EXTINT_ANY_EDGE     (1u)
#define EXTINT_FALLING_EDGE (2u)
#define EXTINT_RISING_EDGE  (3u)

#define DEF_ISR_INT(_NUM_, _VAR_) \
        ISR(INT##_NUM_##_vect) \
        { \
                _VAR_ = 0xFFu; \
        }
		
#define ISC0  ISC00
#define ISC1  ISC10
#define ISC20 ISC2

#define DEF_INIT_INT(_NUM_, _EDGE_, _VAR_) \
void initExtInt##_NUM_(void) \
{ \
        _VAR_ = 0x00u; \
        GICR &= ~(1u << INT##_NUM_); \
		if (_NUM_ != 2) \
			MCUCR = (MCUCR & ~(3u << ISC##_NUM_##0)) | (_EDGE_ << ISC##_NUM_##0); \
		else \
			EMCUCR = (EMCUCR & ~(1u << ISC##_NUM_)) | ((_EDGE_ & 1u) << ISC##_NUM_); \
        GIFR |= (1u << INTF##_NUM_); \
        GICR |= (1u << INT##_NUM_); \
}

#define CREATE_INT(_NUM_, _EDGE_, _VAR_) \
        DEF_ISR_INT(_NUM_, _VAR_) \
        DEF_INIT_INT(_NUM_, _EDGE_, _VAR_)

#endif /* EXT_INT_H_ */