/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>

#include "si5351.h"

#define SI5351_I2C_ADDR 0x060

static void
si5351_start_xfer(uint8_t reg)
{
    (void) I2C_1_MasterSendStart(SI5351_I2C_ADDR, I2C_1_WRITE_XFER_MODE);
    (void) I2C_1_MasterWriteByte(reg); /* write register address */
}

void
si5351_write_xfer(uint8_t reg, uint8_t *data, int count)
{
    /* if data is NULL, write 0x00 instead */
#if 1
    si5351_start_xfer(reg);
    while (count-- > 0)
        (void) I2C_1_MasterWriteByte(data ? *data++ : 0x00);
    I2C_1_MasterSendStop();
#else
    while (count-- > 0) {
        si5351_start_xfer(reg++);
        (void) I2C_1_MasterWriteByte(data ? *data++ : 0x00);
        I2C_1_MasterSendStop();
    }
#endif
}

void
si5351_read_xfer(uint8_t reg, uint8_t *data, int count)
{
    si5351_start_xfer(reg);
    I2C_1_MasterSendStop();
    (void) I2C_1_MasterSendStart(SI5351_I2C_ADDR, I2C_1_READ_XFER_MODE);
    
    while (count > 1) {
        *data++ = I2C_1_MasterReadByte(I2C_1_ACK_DATA);
        count--;
    }
    
    if (count == 1) {
        *data++ = I2C_1_MasterReadByte(I2C_1_NAK_DATA);
    }
    I2C_1_MasterSendStop();
}

void
si5351_write_byte(uint8_t reg, uint8_t val)
{
    si5351_write_xfer(reg, &val, 1);
}

uint8_t
si5351_read_byte(uint8_t reg)
{
    uint8_t data;
    
    si5351_read_xfer(reg, &data, 1);
    return (data);
}

int main()
{
    int32_t freq, pll_freq, count = 0;
    int a = 0;
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    CyGlobalIntEnable;  /* Uncomment this line to enable global interrupts. */
    
    I2C_1_Start();
    
    si5351_init(27000000, 1612, SI5351_CRYSTAL_LOAD_10PF);
    si5351_clock_enable(SI5351_CLK0, 1);
    si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA);
   // pll_freq = si5351_set_frequency(190000000, 0, SI5351_CLK0);
    
  //  pll_freq = si5351_set_frequency(155000000, 0, SI5351_CLK0);
  //  pll_freq = si5351_set_frequency(156000000, 0, SI5351_CLK0);
    pll_freq = si5351_set_frequency(155000000, 0, SI5351_CLK0, 1);
    pll_freq = si5351_set_frequency(11700000, 0, SI5351_CLK0, 1);
    
#if 0
    si5351_calibration_mode(1);
    // while (1) ;
#endif

#if 0
    si5351_clock_enable(SI5351_CLK1, 1);
    si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
    pll_freq = si5351_set_frequency(155050000, 0, SI5351_CLK1, SI5351_MS_MODE_EVEN_INT);
#endif
    
#if 0
    si5351_clock_enable(SI5351_CLK1, 1);
    si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
    si5351_set_frequency(156050000, 0, SI5351_CLK1);
    si5351_clock_enable(SI5351_CLK2, 1);
    si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_6MA);
    si5351_set_frequency(155050000, 0, SI5351_CLK2);
#endif

    pll_freq = 0;
    while (1) //  ;
    {
        //Control_Reg_1_Write(a++ & 1);
        for (freq = 11700000; freq < 11702000; freq += 1) {
            int rv = si5351_set_frequency(freq, pll_freq, SI5351_CLK0, 0);
        }
    }
        
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
