/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         Device driver for LSM303DLM 3-axis accelerometer and 3-axis 
 *         magnetometer on Zolertia Z1.
 *         Datasheet: 
 *         http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF251902
 * \author
 *         Jelmer Tiete, VUB <jelmer@tiete.be>

 */


#include <stdio.h>
#include "contiki.h"
#include "lsm303.h"
#include "i2cmaster.h"

/*---------------------------------------------------------------------------*/
/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/

void
lsm303_accm_write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx_buf[] = {reg, val};

  i2c_transmitinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while (i2c_busy());
  PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}
/*---------------------------------------------------------------------------*/
/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from

  First byte in stream must be the register address to begin writing to.
  The data is then written from second byte and increasing. */

void
lsm303_accm_write_stream(uint8_t len, uint8_t *data) {
  i2c_transmitinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  PRINTFDEBUG("I2C Ready to TX(stream)\n");

  i2c_transmit_n(len, data);	// start tx and send conf reg 
  while (i2c_busy());
  PRINTFDEBUG("WRITE_STR %u B to 0x%02X\n", len, data[0]);
}

/*---------------------------------------------------------------------------*/
/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/

uint8_t
lsm303_accm_read_reg(uint8_t reg) {
  uint8_t retVal = 0;
  uint8_t rtx = reg;
  PRINTFDEBUG("READ_REG 0x%02X\n", reg);

  /* transmit the register to read */
  i2c_transmitinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &rtx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  i2c_receive_n(1, &retVal);
  while (i2c_busy());

  return retVal;
}

/*---------------------------------------------------------------------------*/
/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/

void
lsm303_accm_read_stream(uint8_t reg, uint8_t len, uint8_t *whereto) {
  uint8_t rtx = reg;
  PRINTFDEBUG("READ_STR %u B from 0x%02X\n", len, reg);

  /* transmit the register to start reading from */
  i2c_transmitinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &rtx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_ACC_ADDR);
  while (i2c_busy());
  i2c_receive_n(len, whereto);
  while (i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/

void
lsm303_magn_write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx_buf[] = {reg, val};

  i2c_transmitinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while (i2c_busy());
  PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}
/*---------------------------------------------------------------------------*/
/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from

  First byte in stream must be the register address to begin writing to.
  The data is then written from second byte and increasing. */

void
lsm303_magn_write_stream(uint8_t len, uint8_t *data) {
  i2c_transmitinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  PRINTFDEBUG("I2C Ready to TX(stream)\n");

  i2c_transmit_n(len, data);	// start tx and send conf reg 
  while (i2c_busy());
  PRINTFDEBUG("WRITE_STR %u B to 0x%02X\n", len, data[0]);
}

/*---------------------------------------------------------------------------*/
/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/

uint8_t
lsm303_magn_read_reg(uint8_t reg) {
  uint8_t retVal = 0;
  uint8_t rtx = reg;
  PRINTFDEBUG("READ_REG 0x%02X\n", reg);

  /* transmit the register to read */
  i2c_transmitinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &rtx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  i2c_receive_n(1, &retVal);
  while (i2c_busy());

  return retVal;
}

/*---------------------------------------------------------------------------*/
/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/

void
lsm303_magn_read_stream(uint8_t reg, uint8_t len, uint8_t *whereto) {
  uint8_t rtx = reg;
  PRINTFDEBUG("READ_STR %u B from 0x%02X\n", len, reg);

  /* transmit the register to start reading from */
  i2c_transmitinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &rtx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_MAG_ADDR);
  while (i2c_busy());
  i2c_receive_n(len, whereto);
  while (i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* Init the accelerometer: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */

void
lsm303_init(void) {
  /* Set up ports and pins for I2C communication */
  i2c_enable();

  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  lsm303_accm_write_reg(LSM303_CTRL_REG1_A, 0x27);

  // Enable Magnetometer
  // 0x00 = 0b00000000
  // Continuous conversion mode
  lsm303_magn_write_reg(LSM303_MR_REG_M, 0x00);  

}

/*---------------------------------------------------------------------------*/
/* Reads the 3 magnetometer channels and stores them in vector m
*/

void 
lsm303_magn_read(void)
{
  int16_t xm,ym,zm = 0;
  uint8_t xzym[6];
  lsm303_magn_read_stream(LSM303_OUT_X_H_M,6,&xzym[0]);

  xm = (int16_t)(xzym[1] | (xzym[0]<<8));
  zm = (int16_t)(xzym[3] | (xzym[2]<<8));
  ym = (int16_t)(xzym[5] | (xzym[4]<<8));

  printf("Magneto: x- %i y- %i z- %i \n",xm,ym,zm);
}
