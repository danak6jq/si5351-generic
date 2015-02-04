# si5351-generic
Generic Si5351A3 control functions

Dana H. Myers  K6JQ

General notes:

Development of these functions is currently on a PSoC 5LP freeSoC mini,
which contains an ARM Cortex-M0. However, si5351.c and si5351.h should easily integrated into other C-language environments, including avr-gcc.

These functions depend on being fed sane parameters; there is only
minimal error checking if you ask the Si5351 to do something it is
not capable of, or if you ask it to do something conflicting (in
particular, pay attention to output frequencies above 150MHz on
CLK1 and CLK2 outputs (see below)).

I strongly suggest using only a single output of the
Si5351 for RF applications, since there is a relatively
large amount of cross-talk between the clock outputs.

The MultiSynth mode parameter can be used to force the
clock divider into integer mode, which likely improves RF
phase noise performance.

I2C functional interface macros (in si5351.h):

```
SI5351_WRITE_XFER(reg, datap, count)
SI5351_READ_XFER(reg, datap, count)
SI5351_WRITE_BYTE(reg, val)
SI5351_READ_BYTE(reg)
```

where:
```
	uint8_t	reg:		register address in Si5351
	uint8_t * datap:	pointer to data buffer for read/write
	uint8_t count:		count of bytes to transer
	uint8_t	val:	value to write
```

Note:
	If datap == NULL, writes must be 0

```
void
si5351_init(uint32_t freq_xtal, int32_t xtal_correction, int load_cap)
```

Arguments:
```
	freq_xtal:	reference frequency source in Hz
	xtal_correct:	correction factor in parts-per-10MHz for reference
	load_cap:	load capacitance:
				SI5351_CRYSTAL_LOAD_6PF
				SI5351_CRYSTAL_LOAD_8PF
				SI5351_CRYSTAL_LOAD_10PF
```

Busy-waits for Si5351A to complete power-on reset, initializes
Si5351A3; disables and powers-down all clock outputs. Generates corrected
reference frequency using the reference frequency source and correction
factor.


```
void
si5351_calibration_mode(int enable)
```

Argument:
```
	enable:		0 disables calibration mode, !0 enables
```

Configures CLK0 to pass-through the crystal oscillator according to
enable argument.

```
void
si5351_clock_enable(enum si5351_clock clk, int enable)
```

Arguments:
```
	clk:		clock output, one of:
				SI5351_CLK0
				SI5351_CLK1
				SI5351_CLK2
	enable:		0 disables, !0 enables
```

Enables the selected clock output. This function also powers-down
disabled clock outputs.

```
void
si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
```

Arguments:
```
	clk:		clock output, one of:
				SI5351_CLK0
				SI5351_CLK1
				SI5351_CLK2
	drive:		one of:
				SI5351_DRIVE_2MA
				SI5351_DRIVE_4MA
				SI5351_DRIVE_6MA
				SI5351_DRIVE_8MA
```

Sets the drive level for the selected clock output.

```
int32_t
si5351_set_frequency(int32_t freq, int32_t pll_freq,
  enum si5351_clock clock, int reset)
```

Arguments:
```
	freq:		desired frequency in Hz
	pll_freq:	PLL frequency in Hz; 0 to allow selection
	clock:		clock output to program, one of:
				SI5351_CLK0
				SI5351_CLK1
				SI5351_CLK2
	reset:		Force PLL reset after frequency change; required
			when changing above or below 150MHz.
```

Programs the selected clock output to the chosen frequency 'freq'.
If pll_freq == 0, the PLL is programmed to the highest usable frequency;
if not 0, the PLL is programmed to this frequency. The PLL is programmed
with a fractional divisor. If the chosen frequency 'freq' is between 150
and 160MHz, the PLL is fixed at 4 * freq. Invalid frequency selection is
indicated with a return of -1, otherwise the PLL frequency used is returned.

Outputs CLK1 and CLK2 share the same PLL; the first output programmed
determines the PLL frequency and the second output programmed will use
this PLL frequency regardless of pll_freq. Further, ms_mode will be
forced to MODE_FRAC for the second output programmed. Since a single
PLL is shared, only one of CLK1 and CLK2 can generate a frequency
above 150MHz and must be the first output programmed.

