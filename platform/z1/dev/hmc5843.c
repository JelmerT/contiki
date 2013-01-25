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
 *         Enric M. Calvo, Zolertia <ecalvo@zolertia.com>
 *         Marcus Lundén, SICS <mlunden@sics.se>
 */


#include <stdio.h>
#include "contiki.h"
#include "i2cmaster.h"
#include "hmc5843.h"

#define PRINTFDEBUG(...) printf(__VA_ARGS__)
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
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while (i2c_busy());
  PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}

/*---------------------------------------------------------------------------*/
/* Init the magneto sensor: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */

void
hmc5843_init (void)
{
  i2c_enable ();

  //make sure you have at least 8.3 milli-second brake after power-up! 


  hmc5843_write_reg(HMC5843_MODE, 0x00); // set continues mode (0x00) in mode register (0x02)
 // hmc5843_point_reg(HMC5843_X_MSB);
}


/*---------------------------------------------------------------------------*/
/* Get Values.
    args:
    returns the value of the read register type uint16_t
*/

uint16_t
hmc5843_get_values ()
{
  // maintain 100 mili-second delay between queries!

  uint8_t buf[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int16_t retVal,x,y,z,h = 0;

  // receive the data
  i2c_receiveinit (HMC5843_ADDR); //address pointer should be at 0x03, so just start reading
  while (i2c_busy ());
  i2c_receive_n (7, &buf[0]);
  while (i2c_busy ());

  // transmit the register to read //must reach 0x09 to go back to 0x03


  x = (int16_t) (buf[0] << 8 | (buf[1]));
  y = (int16_t) (buf[2] << 8 | (buf[3]));
  z = (int16_t) (buf[4] << 8 | (buf[5]));
  h = y/x;
  
  PRINTFDEBUG ("x:%i y:%i z:%i heading:%i\n", x, y, z,h);
  //PRINTFDEBUG ("x1:0x%02X x2:0x%02X y1:0x%02X y2:0x%02X z1:0x%02X z2:0x%02X s1:0x%02X\n", buf[0],buf[1], buf[2],buf[3], buf[4],buf[5], buf[6]);

  return retVal;
}
