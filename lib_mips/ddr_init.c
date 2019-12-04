#include <common.h>
#include <rt_mmap.h>

#define DRAMC_BASE 0xBE005000

#define MPLL_IN_LBK	1
#define MEMPLL_CLK_200	1

#define udelay_a(count) \
        do {    \
           register unsigned int delay;        \
           asm volatile (	\
           				 "move %0, %1\n\t"      \
                         "1:\n\t"              \
                         "subu %0, %0, 1\n\t" \
                         "bgtz %0, 1b\n\t"          \
                         "nop\n\t"	\
                         : "+r" (delay)        \
                         : "r" (count)         \
                         : "cc"); \
        } while (0)

int ddr_initialize(void)
{
	int oneusec = 25;	/* 1/((1/50)*2) */


	if ((RALINK_REG(0xBE000010)>>4)&0x1)
	{
#include "ddr2.h"
	}
	else
	{
#include "ddr3.h"
	}
}

int mempll_init()
{
	int oneusec = 25;
#include "mpll40Mhz.h"
	return 0;
}
