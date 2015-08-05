#ifndef DELAY_H_
#define DELAY_H_

#include "config.h"

#define delay_ms(_TIME_MS_) \
	{\
		volatile uint64_t dly = _TIME_MS_ * (F_CPU / 1000ul); \
		for (;dly > 0; dly--); \
	}

#define delay_us(_TIME_US_) \
{\
	volatile uint64_t dly = _TIME_US_ * (F_CPU / 1000000ul); \
	for (;dly > 0; dly--); \
}

#endif /* DELAY_H_ */