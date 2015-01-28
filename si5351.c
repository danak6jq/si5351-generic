/*
 * si5351.c - Si5351 library for avr-gcc
 *
 * Copyright (C) 2014 Jason Milldrum <milldrum@gmail.com>
 * Copyright 2015, Dana H. Myers <k6jq@comcast.net>
 *
 * Extensive re-factoring 25 Jan 2015:
 *  - Merged multisynth_calc() into a single function that takes a PLL
 *    frequency parameter and an integer division flag. Single function
 *    can calculate maximum precision, reduced jitter, or re-calculate.
 *
 *  - si5351_init() takes reference correction and calculates corrected
 *    reference once. Also take load capacitance parameter. Initialization
 *    simulates power-on reset by clearing all registers.
 *
 *  - si5351_calibration_mode() switches pass-through of reference to CLK0
 *    output to permit direct measurement of reference
 *
 *  - Various enhancement; eliminate floating-point use, do_div() macro
 *    malloc() use, eliminates unused headers
 *
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * rational_best_approximation() derived from lib/rational.c in
 * the Linux kernel.
 * Copyright (C) 2009 emlix GmbH, Oskar Schirmer <oskar@scara.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdint.h>
#include "si5351.h"

#define RFRAC_DENOM (1L << 24)

static void rational_best_approximation(
        uint32_t, uint32_t,
        uint32_t, uint32_t,
        uint32_t *, uint32_t *);
static uint32_t pll_calc(uint32_t, struct Si5351RegSet *);
static uint32_t multisynth_calc(uint32_t, uint32_t, enum si5351_ms_mode,
    struct Si5351RegSet *);

uint32_t	plla_freq, pllb_freq;
uint32_t    freq_ref;

/*
 * n_si5351_init
 */
void
si5351_init(uint32_t freq_xtal, int32_t xtal_correction, int load_cap)
{
    int clock;

    /*
     * Wait for system init completion - occurs after power-up
     */
    while (SI5351_READ_BYTE(SI5351_DEVICE_STATUS) & SI5351_STATUS_SYS_INIT)
        ;
    
    /*
     * Reset the Si5351 to power-on state
     */
    SI5351_WRITE_XFER(SI5351_DEVICE_STATUS, NULL,
      SI5351_FANOUT_ENABLE - SI5351_DEVICE_STATUS + 1);
    SI5351_WRITE_BYTE(SI5351_PLL_RESET,
      SI5351_PLL_RESET_B | SI5351_PLL_RESET_A);
    SI5351_WRITE_BYTE(SI5351_OUTPUT_ENABLE_CTRL, 0xFF);
    /* Disable state - select high-Z */
    SI5351_WRITE_BYTE(SI5351_CLK3_0_DISABLE_STATE,
      SI5351_CLK_DISABLE_STATE_FLOAT |
      SI5351_CLK_DISABLE_STATE_FLOAT << 2 |
      SI5351_CLK_DISABLE_STATE_FLOAT << 4 |
      SI5351_CLK_DISABLE_STATE_FLOAT << 6);
    SI5351_WRITE_BYTE(SI5351_CLK7_4_DISABLE_STATE,
      SI5351_CLK_DISABLE_STATE_FLOAT |
      SI5351_CLK_DISABLE_STATE_FLOAT << 2 |
      SI5351_CLK_DISABLE_STATE_FLOAT << 4 |
      SI5351_CLK_DISABLE_STATE_FLOAT << 6);
    SI5351_WRITE_BYTE(SI5351_CRYSTAL_LOAD, load_cap);
    
    /* Disable and power-down clock outputs */
    for (clock = SI5351_CLK0; clock <= SI5351_CLK7; clock++)
        si5351_clock_enable(clock, 0);

	/* Calculated corrected reference frequency */
	/* xtal_correction is Measured in parts-per-ten million */
    freq_ref = freq_xtal + (int32_t)((((((int64_t)xtal_correction) << 31) /
      10000000LL) * freq_xtal) >> 31);
}

// get frequency

/*
 * Calibration mode for the crystal
 * enabled: XTAL output appears on CLK0 output
 * disable: multisynth output appears on CLK0 output
 * note that CLK0 must be enabled, too
 */
void
si5351_calibration_mode(int enable)
{
    
    if (enable) {
        SI5351_WRITE_BYTE(SI5351_FANOUT_ENABLE,
          SI5351_READ_BYTE(SI5351_FANOUT_ENABLE) | SI5351_XTAL_ENABLE);
        SI5351_WRITE_BYTE(SI5351_CLK0_CTRL,
          (SI5351_READ_BYTE(SI5351_CLK0_CTRL) & ~SI5351_CLK_INPUT_MASK) |
          SI5351_CLK_INPUT_XTAL);
    } else {
        SI5351_WRITE_BYTE(SI5351_CLK0_CTRL,
          (SI5351_READ_BYTE(SI5351_CLK0_CTRL) & ~SI5351_CLK_INPUT_MASK) |
          SI5351_CLK_INPUT_MULTISYNTH_N);
        SI5351_WRITE_BYTE(SI5351_FANOUT_ENABLE,
          SI5351_READ_BYTE(SI5351_FANOUT_ENABLE) & ~SI5351_XTAL_ENABLE);
    }
}

/*
 * si5351_clock_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen clock
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void
si5351_clock_enable(enum si5351_clock clk, int enable)
{
	uint8_t reg_val;

	reg_val = SI5351_READ_BYTE(SI5351_OUTPUT_ENABLE_CTRL) |
      (1 << (uint8_t)clk);

	if (enable) {
		reg_val &= ~(1<<(uint8_t)clk);
	}
	SI5351_WRITE_BYTE(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
    
	reg_val = SI5351_READ_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk) |
      SI5351_CLK_POWERDOWN;
    
    if (enable) {
        reg_val &= ~SI5351_CLK_POWERDOWN;
    }
	SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void
si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
{
	uint8_t reg_val;

	reg_val = SI5351_READ_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk) &
      ~SI5351_CLK_DRIVE_STRENGTH_MASK;

	switch(drive) {
	case SI5351_DRIVE_2MA:
		reg_val |= SI5351_CLK_DRIVE_STRENGTH_2MA;
		break;
	case SI5351_DRIVE_4MA:
		reg_val |= SI5351_CLK_DRIVE_STRENGTH_4MA;
		break;
	case SI5351_DRIVE_6MA:
		reg_val |= SI5351_CLK_DRIVE_STRENGTH_6MA;
		break;
	case SI5351_DRIVE_8MA:
		reg_val |= SI5351_CLK_DRIVE_STRENGTH_8MA;
		break;
	}

	SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

static void
si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll,
  enum si5351_ms_mode ms_mode)
{
	uint8_t reg_val;

	reg_val = SI5351_READ_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk);
    reg_val |= SI5351_CLK_INPUT_MULTISYNTH_N;
    reg_val &= ~(SI5351_CLK_POWERDOWN | SI5351_CLK_INTEGER_MODE);

	if(pll == SI5351_PLLA) {
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	} else if (pll == SI5351_PLLB) {
		reg_val |= SI5351_CLK_PLL_SELECT;
	}
    
    if (ms_mode == SI5351_MS_MODE_EVEN_INT) {
        reg_val |= SI5351_CLK_INTEGER_MODE;
    }
    
	SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

static void
si5351_set_pll(uint32_t pll_freq, enum si5351_pll pll_prog)
{
   	struct Si5351RegSet pllreg;
    int addr;
    char regbuf[8];
    
    /* keep track of PLL programming */
    switch (pll_prog) {
    case SI5351_PLLA:
        plla_freq = pll_freq;
        addr = SI5351_PLLA_PARAMETERS;
        break;
    case SI5351_PLLB:
        pllb_freq = pll_freq;
        addr = SI5351_PLLB_PARAMETERS;
        break;
    }
    
    /* XXX: using fixed correction factor here */
	pll_calc(pll_freq, &pllreg);
	
	/* Registers 26-27 */
	regbuf[0] = (uint8_t) (pllreg.p3 >> 8);
	regbuf[1] = (uint8_t) pllreg.p3;

	/* Register 28 */
	regbuf[2] = (uint8_t) ((pllreg.p1 >> 16) & 0x03);

	/* Registers 29-30 */
	regbuf[3] = (uint8_t) (pllreg.p1 >> 8);
	regbuf[4] = (uint8_t) pllreg.p1;

	/* Register 31 */
	regbuf[5] = ((uint8_t) ((pllreg.p3 >> 12) & 0xF0)) |
      ((uint8_t) ((pllreg.p2 >> 16) & 0x0F));

	/* Registers 32-33 */
	regbuf[6] = (uint8_t) (pllreg.p2 >> 8);
	regbuf[7] = (uint8_t) pllreg.p2;

	/* Write the parameters */
	SI5351_WRITE_XFER(addr, (uint8_t *) regbuf, 8);
    /* Reset the PLL */
    // XXX:
}

static void
si5351_set_multisynth(struct Si5351RegSet *msreg, enum si5351_clock clock,
  enum si5351_pll pll_prog, enum si5351_ms_mode ms_mode)
{
    int addr = -1;
    char regbuf[8];
    
	/* Registers 42-43 */
	regbuf[0] = (uint8_t) (msreg->p3 >> 8);
	regbuf[1] = (uint8_t) msreg->p3;

	/* Register 44 */
	/* TODO: add code for output divider */
	regbuf[2] = (uint8_t) ((msreg->p1 >> 16) & 0x03);

	/* Registers 45-46 */
	regbuf[3] = (uint8_t) (msreg->p1 >> 8);
	regbuf[4] = (uint8_t) msreg->p1;

	/* Register 47 */
	regbuf[5] = (uint8_t) ((msreg->p3 >> 12) & 0xF0) |
      (uint8_t) ((msreg->p2 >> 16) & 0x0F);

	/* Registers 48-49 */
	regbuf[6] = (uint8_t) (msreg->p2 >> 8);
	regbuf[7] = (uint8_t) msreg->p2;

   	switch (clock) {
	case SI5351_CLK0:
		addr = SI5351_CLK0_PARAMETERS;
		break;
	case SI5351_CLK1:
		addr = SI5351_CLK1_PARAMETERS;
		break;
	case SI5351_CLK2:
		addr = SI5351_CLK2_PARAMETERS;
		break;
	case SI5351_CLK3:
		addr = SI5351_CLK3_PARAMETERS;
		break;
	case SI5351_CLK4:
		addr = SI5351_CLK4_PARAMETERS;
		break;
	case SI5351_CLK5:
		addr = SI5351_CLK5_PARAMETERS;
		break;
	case SI5351_CLK6:
		addr = SI5351_CLK6_PARAMETERS;
		break;
	case SI5351_CLK7:
		addr = SI5351_CLK7_PARAMETERS;
		break;
	}

    if (addr < 0)
        return;
    
	SI5351_WRITE_XFER(addr, (uint8_t *) regbuf, 8);
    si5351_set_ms_source(clock, pll_prog, ms_mode);
}

/*
 * 
 */
uint32_t
si5351_set_frequency(uint32_t freq, uint32_t pll_freq,
  enum si5351_clock clock, enum si5351_ms_mode ms_mode)
{
	struct Si5351RegSet msreg;
	int pll_prog;
    int program_pll = 0;
	
	/*
	 * if pll_freq == 0, let multisynth_calc() pick the PLL frequency
	 */
    pll_freq = multisynth_calc(freq, pll_freq, ms_mode, &msreg);
	 
	 /*
	  * select PLL
	  */
	switch (clock) {
	case SI5351_CLK0:
        pll_prog = SI5351_PLLA;
        program_pll = (plla_freq != pll_freq);
		break;
	/* CLK1 and CLK2 share PLLB; first programmed sets PLL freq */
	case SI5351_CLK1:
	case SI5351_CLK2:
		pll_prog = SI5351_PLLB;
		if (pllb_freq != 0) {
			multisynth_calc(freq, pllb_freq, SI5351_MS_MODE_FRAC, &msreg);
		} else {
            program_pll = 1;
		}
		break;
	default:
		/* XXX: */
		break;
	}

    if (program_pll)
        si5351_set_pll(pll_freq, pll_prog);

    si5351_set_multisynth(&msreg, clock, pll_prog, ms_mode);
    return (pll_freq);
}


/*
 * Calculate best rational approximation for a given fraction
 * taking into account restricted register size, e.g. to find
 * appropriate values for a pll with 5 bit denominator and
 * 8 bit numerator register fields, trying to set up with a
 * frequency ratio of 3.1415, one would say:
 *
 * rational_best_approximation(31415, 10000,
 *              (1 << 8) - 1, (1 << 5) - 1, &n, &d);
 *
 * you may look at given_numerator as a fixed point number,
 * with the fractional part size described in given_denominator.
 *
 * for theoretical background, see:
 * http://en.wikipedia.org/wiki/Continued_fraction
 */

static void
rational_best_approximation(
        uint32_t given_numerator, uint32_t given_denominator,
        uint32_t max_numerator, uint32_t max_denominator,
        uint32_t *best_numerator, uint32_t *best_denominator)
{
	unsigned long n, d, n0, d0, n1, d1;
	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;
	for (;;) {
		unsigned long t, a;
		if ((n1 > max_numerator) || (d1 > max_denominator)) {
			n1 = n0;
			d1 = d0;
			break;
		}
		if (d == 0)
			break;
		t = d;
		a = n / d;
		d = n % d;
		n = t;
		t = n0 + a * n1;
		n0 = n1;
		n1 = t;
		t = d0 + a * d1;
		d0 = d1;
		d1 = t;
	}
	*best_numerator = n1;
	*best_denominator = d1;
}

static uint32_t
pll_calc(uint32_t freq, struct Si5351RegSet *reg)
{
	uint32_t rfrac, a, b, c;

	/* PLL bounds checking */
	if (freq < SI5351_PLL_VCO_MIN) {
		freq = SI5351_PLL_VCO_MIN;
    } else if (freq > SI5351_PLL_VCO_MAX) {
		freq = SI5351_PLL_VCO_MAX;
    }

	/* Determine integer part of feedback equation */
	a = freq / freq_ref;

	if (a < SI5351_PLL_A_MIN) {
		freq = freq_ref * SI5351_PLL_A_MIN;
    } else if (a > SI5351_PLL_A_MAX) {
		freq = freq_ref * SI5351_PLL_A_MAX;
    }

	/* find best approximation for b/c = fVCO mod fIN */
    b = 0;
    c = 1;
    rfrac = (((uint64_t)(freq % freq_ref)) * RFRAC_DENOM) / freq_ref;
	if (rfrac)
		rational_best_approximation(rfrac, RFRAC_DENOM,
				    SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);

	/* calculate parameters */
	reg->p3  = c;
	reg->p2  = (128 * b) % c;
	reg->p1  = 128 * a + (128 * b / c) - 512;

	/* recalculate rate by fIN * (a + b/c) */
    freq = (uint32_t) ((((uint64_t) freq_ref) * ((uint64_t) b)) / c) +
      freq_ref * a;

	return (freq);
}

static uint32_t
multisynth_calc(uint32_t freq, uint32_t pll_freq,
  enum si5351_ms_mode ms_mode, struct Si5351RegSet *reg)
{
	uint32_t rfrac, a, b, c;

	/* Multisynth bounds checking */
    /* XXX: actually sort of pointless without an error indicationm */
    if (freq > SI5351_MULTISYNTH_MAX_FREQ) {
        freq = SI5351_MULTISYNTH_MAX_FREQ;
    } else if (freq < SI5351_MULTISYNTH_MIN_FREQ) {
		freq = SI5351_MULTISYNTH_MIN_FREQ;
    }

	/* Calculate parameters */
	if (freq > SI5351_MULTISYNTH_DIVBY4_FREQ) {
        a = 4;
        b = 0;
        c = 1;
        pll_freq = a * freq;

		reg->p3 = 1;
		reg->p2 = 0;
		reg->p1 = 0;
    } else {
        if (pll_freq == 0) {
            pll_freq = SI5351_PLL_VCO_MAX;
            /* arrange for integer MS divider */
            if (ms_mode == SI5351_MS_MODE_INT ||
              ms_mode == SI5351_MS_MODE_EVEN_INT) {
                a = pll_freq / freq;
                if (ms_mode == SI5351_MS_MODE_EVEN_INT) {
                    a &= ~1;
                }
                pll_freq = a * freq;
            }
        }
        a = pll_freq / freq;
        b = 0;
        c = 1;
        rfrac = (uint32_t) ((((uint64_t) (pll_freq % freq)) *
          RFRAC_DENOM) / freq);
        
	    if (rfrac) {
            rational_best_approximation(rfrac, RFRAC_DENOM,
			  SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX, &b, &c);
        } else {
            pll_freq = a * freq;
        }
        
		reg->p3  = c;
		reg->p2  = (128 * b) % c;
		reg->p1  = 128 * a + (128 * b / c) - 512;
	}

    return (pll_freq);
}
