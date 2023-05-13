/*
 * 2017, Aurelio Colosimo <aurelio@aureliocolosimo.it>
 * This file is part of fatfs-stm32f4:
 * https://github.com/colosimo/fatfs-stm32
 * MIT License
 */

#include <basic.h>
#include <gpio.h>
#include <log.h>


extern uint32_t k_ticks(void);
extern uint32_t k_ticks_freq(void);

static inline uint32_t k_elapsed(uint32_t tprev)
{
	return k_ticks() - tprev;
}

static inline uint32_t ms_to_ticks(uint32_t ms)
{
	return (k_ticks_freq() * ms) / 1000;
}

static inline uint32_t ticks_to_ms(uint32_t tk)
{
	return (tk * 1000) / k_ticks_freq();
}

void k_delay(const uint32_t ms)
{
	uint32_t s = k_ticks();
	while (k_elapsed(s) < ms_to_ticks(ms));
}

//static void byte_swap(uint8_t *dest, uint32_t src)
//{
//	int i;
//	for (i = 0; i < 4; i ++)
//		dest[i] = src >> (24 - 8 * i);
//}
