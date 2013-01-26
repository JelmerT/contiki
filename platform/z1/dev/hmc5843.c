/*
 * Copyright (c) 2013, Jelmer Tiete
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Device drivers for hmc5843 magnetometer sensor on Zolertia Z1.
 * \author
 *         Jelmer Tiete, VUB <jelmer@tiete.be>
 *         based on arduino library from Fabio Varesano <fvaresano@yahoo.it>
 *         Enric M. Calvo, Zolertia <ecalvo@zolertia.com>
 *         Marcus Lundén, SICS <mlunden@sics.se>
 */


#include <stdio.h>
#include "contiki.h"
#include "i2cmaster.h"
#include "hmc5843.h"

float x_scale,y_scale,z_scale,x_max,y_max,z_max;

#define PRINTFDEBUG(...) printf(__VA_ARGS__)
#define MIN(a,b) ((a) < (b)? (a): (b))
#define true 1
#define false 0

  const int counts_per_milligauss[8]={  
    1620,
    1300,
     970,
     780,
     530,
     460,
     390,
     280
  };

/*---------------------------------------------------------------------------*/
/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/

void
hmc5843_point_reg(uint8_t reg) {
  uint8_t tx_buf[] = {reg};

  i2c_transmitinit(HMC5843_ADDR);
  while (i2c_busy());
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(1, tx_buf);
  while (i2c_busy());
  PRINTFDEBUG("POINT_REG @ reg 0x%02X\n", reg);
}
/*---------------------------------------------------------------------------*/
/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/

void
hmc5843_write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx_buf[] = {reg, val};

  i2c_transmitinit(HMC5843_ADDR);
  while (i2c_busy());
  //PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while (i2c_busy());
  //PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}

void 
hmc5843_set_gain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  hmc5843_write_reg(HMC5843_CONFIG_B, gain << 5);
}

void 
hmc5843_set_mode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  
  hmc5843_write_reg(HMC5843_MODE, mode);
  //delay(100);
}

/*!
    \brief Calibrate using the self test operation.
  
    Average the values using bias mode to obtain the scale factors.

    \param gain [in] Gain setting for the sensor. See data sheet.
    \param n_samples [in] Number of samples to average together while applying the positive and negative bias.
    \return Returns false if any of the following occurs:
        # Invalid input parameters. (gain>7 or n_samples=0).
        # Id registers are wrong for the compiled device. Unfortunately, we can't distinguish between HMC5843 and HMC5883L.
        # Calibration saturates during the positive or negative bias on any of the readings.
        # Readings are outside of the expected range for bias current. 
*/
uint8_t
hmc5843_calibrate(unsigned char gain,uint16_t n_samples) 
{
    int xyz[3];                     // 16 bit integer values for each axis.
    long int xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
    uint8_t bret=true;                 // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    char id[3];                     // Three identification registers should return 'H43'.
    long int low_limit, high_limit;                                    
    /*
        Make sure we are talking to the correct device.
        Hard to believe Honeywell didn't change the identifier.
    */
    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        hmc5843_get_id(id);
        if (('H' == id[0]) && ('4' == id[1]) && ('3' == id[2]))
        {   /*
                Use the positive bias current to impose a known field on each axis.
                This field depends on the device and the axis.
            */
            hmc5843_write_reg(HMC5843_CONFIG_A, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
            /*
                Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
                The new gain setting is effective from the second measurement and on.
            */
            hmc5843_set_gain(gain);                      
            hmc5843_set_mode(1);                         // Change to single measurement mode.
            hmc5843_get_raw(&xyz[0],&xyz[1],&xyz[2]);    // Get the raw values and ignore since this reading may use previous gain.
int i;
            for (i=0; i<n_samples; i++) 
            { 
                hmc5843_set_mode(1);
                hmc5843_get_raw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged rather than taking the max.
                */
                xyz_total[0]+=xyz[0];
                xyz_total[1]+=xyz[1];
                xyz_total[2]+=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= MIN(xyz[0],MIN(xyz[1],xyz[2])))
                {
                    PRINTFDEBUG("HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Apply the negative bias. (Same gain)
            */
            hmc5843_write_reg(HMC5843_CONFIG_A, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
            for (i=0; i<n_samples; i++) 
            { 
                hmc5843_set_mode(1);
                hmc5843_get_raw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged.
                */
                xyz_total[0]-=xyz[0];
                xyz_total[1]-=xyz[1];
                xyz_total[2]-=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= MIN(xyz[0],MIN(xyz[1],xyz[2])))
                {
                    PRINTFDEBUG("HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Compare the values against the expected self test bias gauss.
                Notice, the same limits are applied to all axis.
            */
            low_limit =SELF_TEST_LOW_LIMIT *counts_per_milligauss[gain]*2*n_samples;
            high_limit=SELF_TEST_HIGH_LIMIT*counts_per_milligauss[gain]*2*n_samples;

            if ((true==bret) && 
                (low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
                (low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
                (low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2]) )
            {   /*
                    Successful calibration.
                    Normalize the scale factors so all axis return the same range of values for the bias field.
                    Factor of 2 is from summation of total of n_samples from both positive and negative bias.
                */
                x_scale=(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
                y_scale=(counts_per_milligauss[gain]*(HMC58X3_Y_SELF_TEST_GAUSS*2))/(xyz_total[1]/n_samples);
                z_scale=(counts_per_milligauss[gain]*(HMC58X3_Z_SELF_TEST_GAUSS*2))/(xyz_total[2]/n_samples);
            }else
            {
                PRINTFDEBUG("HMC58x3 Self test out of range.");
                bret=false;
            }
            hmc5843_write_reg(HMC5843_CONFIG_A, 0x010); // set RegA/DOR back to default.
        }else
        {
            PRINTFDEBUG("HMC5843 failed id check.");
            bret=false;
        }
    }else
    {   /*
            Bad input parameters.
        */
        PRINTFDEBUG("HMC5843 Bad parameters.");
        bret=false;
    }
    return(bret);
}

/*---------------------------------------------------------------------------*/
/* Init the magneto sensor: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */
void
hmc5843_init (void)
{
  x_scale=1.0F;
  y_scale=1.0F;
  z_scale=1.0F;

  i2c_enable ();

  //make sure you have at least 8.3 milli-second brake after power-up! 

  hmc5843_write_reg(HMC5843_CONFIG_A, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  hmc5843_write_reg(HMC5843_CONFIG_B, 0xA0);
  hmc5843_write_reg(HMC5843_MODE, 0x00);

//  hmc5843_write_reg(HMC5843_MODE, 0x00); // set continues mode (0x00) in mode register (0x02)
 // hmc5843_point_reg(HMC5843_X_MSB);
}

void hmc5843_get_values(float *x,float *y,float *z) {
  int16_t xr,yr,zr;
  
  hmc5843_get_raw(&xr, &yr, &zr);
  *x = ((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = ((float) zr) / z_scale;
}

/*---------------------------------------------------------------------------*/
/* Read heading in raw format
    args:
	x
	y
	z
*/

void
hmc5843_get_raw(int16_t *x,int16_t *y,int16_t *z)
{
  // maintain 100 mili-second delay between queries!

  uint8_t buf[7];

  // receive the data
  i2c_receiveinit (HMC5843_ADDR); //address pointer should be at 0x03, so just start reading
  while (i2c_busy ());
  i2c_receive_n (7, &buf[0]);
  while (i2c_busy ());
  //address pointer wraps around back to 0x03 after status reg is read

  *x = (int16_t) (buf[0] << 8 | (buf[1]));
  *y = (int16_t) (buf[2] << 8 | (buf[3]));
  *z = (int16_t) (buf[4] << 8 | (buf[5]));
  
  //PRINTFDEBUG ("x:%i y:%i z:%i\n", x, y, z);
  //PRINTFDEBUG ("x1:0x%02X x2:0x%02X y1:0x%02X y2:0x%02X z1:0x%02X z2:0x%02X s1:0x%02X\n", buf[0],buf[1], buf[2],buf[3], buf[4],buf[5], buf[6]);
}

/*! 
    \brief Retrieve the value of the three ID registers.    

    Note:  Both the HMC5843 and HMC5883L have the same 'H43' identification register values. (Looks like someone at Honeywell screwed up.)

    \param id [out] Returns the three id register values.
*/
void
hmc5843_get_id(char id[3]) 
{
  hmc5843_point_reg(HMC5843_ID_A);
  
  // receive the ID's
  i2c_receiveinit (HMC5843_ADDR);
  while (i2c_busy ());
  i2c_receive_n (3, &id[0]);
  while (i2c_busy ());

  hmc5843_point_reg(HMC5843_X_MSB); //point back to measurements

}

