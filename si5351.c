/*
 * si5351.c - Generic Si5351A3 library
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

#define RFRAC_DENOM ((1L << 20) - 1)

static int32_t pll_calc(int32_t, uint8_t *);
static int32_t multisynth_calc(int32_t, int32_t, uint8_t *, uint8_t *);

int32_t	plla_freq, pllb_freq;
int32_t    freq_ref;
uint8_t clock_control[8];

/*
 * n_si5351_init
 */
void
si5351_init(int32_t freq_xtal, int32_t xtal_correction, int load_cap)
{
    int clock;

    /*
     * Wait for system init completion - occurs after power-up
     */
    while (SI5351_READ_BYTE(SI5351_DEVICE_STATUS) & SI5351_STATUS_SYS_INIT)
        ;
    
    SI5351_WRITE_BYTE(SI5351_OUTPUT_ENABLE_CTRL, 0xFF);
    SI5351_WRITE_BYTE(SI5351_CRYSTAL_LOAD, load_cap | 0x12);
    
    /* Disable and power-down clock outputs */
    clock_control[0] = SI5351_CLK_INPUT_MULTISYNTH_N;
    clock_control[1] = SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_PLL_SELECT;
    clock_control[2] = SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_PLL_SELECT;
    
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
    
    if (enable) {
        clock_control[clk] &= ~SI5351_CLK_POWERDOWN;
    } else {
        clock_control[clk] |= SI5351_CLK_POWERDOWN;
    }
    
    SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk, clock_control[clk]);
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

    clock_control[clk] &= ~SI5351_CLK_DRIVE_STRENGTH_MASK;

    switch(drive) {
    case SI5351_DRIVE_2MA:
        clock_control[clk] |= SI5351_CLK_DRIVE_STRENGTH_2MA;
        break;
    case SI5351_DRIVE_4MA:
        clock_control[clk] |= SI5351_CLK_DRIVE_STRENGTH_4MA;
        break;
    case SI5351_DRIVE_6MA:
        clock_control[clk] |= SI5351_CLK_DRIVE_STRENGTH_6MA;
        break;
    case SI5351_DRIVE_8MA:
        clock_control[clk] |= SI5351_CLK_DRIVE_STRENGTH_8MA;
        break;
    }

    SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + (uint8_t)clk, clock_control[clk]);
}

/*
 * 
 */
int32_t
si5351_set_frequency(int32_t freq, int32_t pll_freq,
  enum si5351_clock clock, int reset)
{
    uint8_t clk_ctrl, pllregs[8], msregs[8];
    int addr, pll_prog;
    int program_pll = 0;
    
    /*
     * if pll_freq == 0, let multisynth_calc() pick the PLL frequency
     */
    pll_freq = multisynth_calc(freq, pll_freq, &clk_ctrl, msregs);
    
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
            pll_freq = multisynth_calc(freq, pllb_freq, &clk_ctrl, msregs);
        } else {
            program_pll = 1;
        }
        break;
    default:
        /* XXX: */
        break;
    }

    if (pll_freq < 0) {
        return (pll_freq);
    }
    
    if (program_pll) {
        pll_freq = pll_calc(pll_freq, pllregs);
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

        if (pll_freq < 0) {
            return (pll_freq);
        }
    }
    
    clock_control[clock] = (clock_control[clock] &
      ~SI5351_CLK_INTEGER_MODE) | clk_ctrl;
    SI5351_WRITE_BYTE(SI5351_CLK0_CTRL + clock, clock_control[clock]);
    if (program_pll) {
        SI5351_WRITE_XFER(addr, (uint8_t *) pllregs, 8);
    }
    SI5351_WRITE_XFER(SI5351_CLK0_PARAMETERS + clock * 8, (uint8_t *) msregs, 8);
    if (reset) {
        SI5351_WRITE_BYTE(177, 0xac);
    }
    
    return (pll_freq);
}

static int32_t
pll_calc(int32_t freq, uint8_t *regs)
{
    uint32_t a, b, c;
    uint32_t p1, p2, p3;

    /* Determine integer part of feedback equation */
    a = freq / freq_ref;
    if ((a < SI5351_PLL_A_MIN) || (a > SI5351_PLL_A_MAX)) {
        return (-1);
    }

    b = (((uint64_t)(freq % freq_ref)) * RFRAC_DENOM) / freq_ref;
    c = b ? RFRAC_DENOM : 0;
#if 0
    if ((b > 0) && (a == SI5351_PLL_A_MAX)) {
        return (-1);
    }
#endif

    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;
    
    /* Registers 26-27 */
    regs[0] = (uint8_t) (p3 >> 8);
    regs[1] = (uint8_t) p3;

    /* Register 28 */
    regs[2] = (uint8_t) ((p1 >> 16) & 0x03);

    /* Registers 29-30 */
    regs[3] = (uint8_t) (p1 >> 8);
    regs[4] = (uint8_t) p1;

    /* Register 31 */
    regs[5] = ((uint8_t) ((p3 >> 12) & 0xF0)) |
      ((uint8_t) ((p2 >> 16) & 0x0F));

    /* Registers 32-33 */
    regs[6] = (uint8_t) (p2 >> 8);
    regs[7] = (uint8_t) p2;

    
    /* avoid loss of precision here */
    freq = a * freq_ref + (b * freq_ref) / c;
#if 0
    if ((freq < SI5351_PLL_VCO_MIN) || (freq > SI5351_PLL_VCO_MAX)) {
        freq = -1;
    }
#endif
    
    return (freq);
}

static int32_t
multisynth_calc(int32_t freq, int32_t pll_freq, uint8_t *clk_ctrl, uint8_t *regs)
{
    int32_t a, b, c;
    uint32_t p1, p2, p3;
    int r0, divby4, ms_int;
    int32_t target_pll_freq;

    /* Calculate parameters */
    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ) {
        pll_freq = 4 * freq;
        p1 = 0;
        p2 = 0;
        p3 = 1;
        ms_int = 1;
        divby4 = 3;
        r0 = 0;
    } else{
        target_pll_freq = pll_freq ? pll_freq : SI5351_PLL_VCO_MAX;        
        r0 = 0;
        for (;;) {
            a = target_pll_freq / (freq << r0);
            b = target_pll_freq % (freq << r0);
                
            if ((a >= SI5351_MULTISYNTH_A_MIN) &&
              ((a < SI5351_MULTISYNTH_A_MAX) ||
              ((a == SI5351_MULTISYNTH_A_MAX) && (b == 0)))) {
                if (pll_freq == 0) {
                    /* make even and remain in range */
                    if (a & 1) {
                        a--;
                    }
                    target_pll_freq = a * (freq << r0);
                }
                break;
            }
                
            if ((a > SI5351_MULTISYNTH_A_MAX) ||
              ((a == SI5351_MULTISYNTH_A_MAX) && (b != 0))) {
                /* too high a value of A */
                if (++r0 > 7) {
                    /* R0 out of range; error  */
                    return (-1);
                }
                continue;
            }
            
            /* too low a value of A */
            /* XXX: think this is always an error */
            return (-1);
        }
        
        if (pll_freq == 0)
            pll_freq = target_pll_freq;
        /* PLL chosen */

        a = pll_freq / (freq << r0);
        b = ((uint64_t)(pll_freq % (freq << r0)) * RFRAC_DENOM) /
          (freq << r0);
        ms_int = ((a % 2) == 0) && (b == 0);
        c = b ? RFRAC_DENOM : 1;
        
        p1 = 128 * a + ((128 * b) / c) - 512;
        p2 = 128 * b - c * ((128 * b) / c);
        p3 = c;
        divby4 = 0;

    }
    
    *clk_ctrl = ms_int ? SI5351_CLK_INTEGER_MODE : 0;
    
    /* Registers 42-43 */
    regs[0] = (uint8_t) (p3 >> 8);
    regs[1] = (uint8_t) p3;

    /* Register 44 */
    regs[2] = (uint8_t) ((p1 >> 16) & 0x03) |
      (r0 << 4) | (divby4 << 2);

    /* Registers 45-46 */
    regs[3] = (uint8_t) (p1 >> 8);
    regs[4] = (uint8_t) p1;

    /* Register 47 */
    regs[5] = (uint8_t) ((p3 >> 12) & 0xF0) |
      ((p2 >> 16) & 0x0F);

    /* Registers 48-49 */
    regs[6] = (uint8_t) (p2 >> 8);
    regs[7] = (uint8_t) p2;

    return (pll_freq);
}
