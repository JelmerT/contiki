/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
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
 *         Datasheet: http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF251902
 * \author
 *         Jelmer Tiete, VUB <jelmer@tiete.be>
 */


#include <stdio.h>
#include "contiki.h"
#include "lsm303.h"
#include "i2cmaster.h"
#include "math.h"


/*---------------------------------------------------------------------------*/
/* Write to a register.
 *  args:
 *    reg       register to write to
 *    val       value to write
 */

void
lsm303_accm_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx_buf[] = { reg, val };

  i2c_transmitinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while(i2c_busy());
  PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}
/*---------------------------------------------------------------------------*/
/* Write several registers from a stream.
 *  args:
 *    len       number of bytes to read
 *    data      pointer to where the data is read from
 *
 * First byte in stream must be the register address to begin writing to.
 * The data is then written from second byte and increasing.
 */

void
lsm303_accm_write_stream(uint8_t len, uint8_t * data)
{
  i2c_transmitinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  PRINTFDEBUG("I2C Ready to TX(stream)\n");

  i2c_transmit_n(len, data);    // start tx and send conf reg 
  while(i2c_busy());
  PRINTFDEBUG("WRITE_STR %u B to 0x%02X\n", len, data[0]);
}

/*---------------------------------------------------------------------------*/
/* Read one register.
 *  args:
 *    reg       what register to read
 *  returns the value of the read register
 */

uint8_t
lsm303_accm_read_reg(uint8_t reg)
{
  uint8_t retVal = 0;
  uint8_t rtx = reg;

  PRINTFDEBUG("READ_REG 0x%02X\n", reg);

  /* transmit the register to read */
  i2c_transmitinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  i2c_transmit_n(1, &rtx);
  while(i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  i2c_receive_n(1, &retVal);
  while(i2c_busy());

  return retVal;
}

/*---------------------------------------------------------------------------*/
/* Read several registers in a stream.
 *  args:
 *    reg       what register to start reading from
 *    len       number of bytes to read
 *    whereto   pointer to where the data is saved
 */

void
lsm303_accm_read_stream(uint8_t reg, uint8_t len, uint8_t * whereto)
{
  uint8_t rtx = reg;

  PRINTFDEBUG("READ_STR %u B from 0x%02X\n", len, reg);

  /* transmit the register to start reading from */
  i2c_transmitinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  i2c_transmit_n(1, &rtx);
  while(i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_ACC_ADDR);
  while(i2c_busy());
  i2c_receive_n(len, whereto);
  while(i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* Write to a register.
 *  args:
 *    reg       register to write to
 *    val       value to write
 */

void
lsm303_magn_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx_buf[] = { reg, val };

  i2c_transmitinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  PRINTFDEBUG("I2C Ready to TX\n");

  i2c_transmit_n(2, tx_buf);
  while(i2c_busy());
  PRINTFDEBUG("WRITE_REG 0x%02X @ reg 0x%02X\n", val, reg);
}
/*---------------------------------------------------------------------------*/
/* Write several registers from a stream.
 *  args:
 *    len       number of bytes to read
 *    data      pointer to where the data is read from
 *
 * First byte in stream must be the register address to begin writing to.
 * The data is then written from second byte and increasing. 
 */

void
lsm303_magn_write_stream(uint8_t len, uint8_t * data)
{
  i2c_transmitinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  PRINTFDEBUG("I2C Ready to TX(stream)\n");

  i2c_transmit_n(len, data);    // start tx and send conf reg 
  while(i2c_busy());
  PRINTFDEBUG("WRITE_STR %u B to 0x%02X\n", len, data[0]);
}

/*---------------------------------------------------------------------------*/
/* Read one register.
 *  args:
 *    reg       what register to read
 *  returns the value of the read register
 */

uint8_t
lsm303_magn_read_reg(uint8_t reg)
{
  uint8_t retVal = 0;
  uint8_t rtx = reg;

  PRINTFDEBUG("READ_REG 0x%02X\n", reg);

  /* transmit the register to read */
  i2c_transmitinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  i2c_transmit_n(1, &rtx);
  while(i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  i2c_receive_n(1, &retVal);
  while(i2c_busy());

  return retVal;
}

/*---------------------------------------------------------------------------*/
/* Read several registers in a stream.
 *  args:
 *    reg       what register to start reading from
 *    len       number of bytes to read
 *    whereto   pointer to where the data is saved
 */

void
lsm303_magn_read_stream(uint8_t reg, uint8_t len, uint8_t * whereto)
{
  uint8_t rtx = reg;

  PRINTFDEBUG("READ_STR %u B from 0x%02X\n", len, reg);

  /* transmit the register to start reading from */
  i2c_transmitinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  i2c_transmit_n(1, &rtx);
  while(i2c_busy());

  /* receive the data */
  i2c_receiveinit(LSM303_MAG_ADDR);
  while(i2c_busy());
  i2c_receive_n(len, whereto);
  while(i2c_busy());
}
/*---------------------------------------------------------------------------*/
/* Init the accelerometer: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */

void
lsm303_init(void)
{
  /* Set up ports and pins for I2C communication */
  i2c_enable();

  /* Enable Accelerometer
   * 0x27 = 0b00100111
   * Normal power mode, all axes enabled
   */
  lsm303_accm_write_reg(LSM303_CTRL_REG1_A, 0x27);

  /* Enable Magnetometer
   * 0x00 = 0b00000000
   * Continuous conversion mode
   */
  lsm303_magn_write_reg(LSM303_MR_REG_M, 0x00);

}

/*---------------------------------------------------------------------------*/
/* Reads all magnetometer channels and stores them in vector m.
 */

void
lsm303_magn_read(void)
{
  uint8_t tmp[6];

  lsm303_magn_read_stream(LSM303_OUT_X_H_M, 6, &tmp[0]);

  m.x = (int16_t) (tmp[0] << 8 | tmp[1]);
  m.y = (int16_t) (tmp[4] << 8 | tmp[5]);
  m.z = (int16_t) (tmp[2] << 8 | tmp[3]);
}

/*---------------------------------------------------------------------------*/
/* Reads all accelerometer channels and stores them in vector a.
 */

void
lsm303_accm_read(void)
{

  uint8_t tmp[6];

  lsm303_accm_read_stream((LSM303_OUT_X_L_A | (1 << 7)), 6, &tmp[0]);

  /* combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)*/
  a.x = ((int16_t) (tmp[1] << 8 | tmp[0])) >> 4;
  a.y = ((int16_t) (tmp[3] << 8 | tmp[2])) >> 4;
  a.z = ((int16_t) (tmp[5] << 8 | tmp[4])) >> 4;
}



// Returns the number of degrees from the -Y axis that it
// is pointing.
//int lsm303_heading(void)
//{
//  return heading((vector){0,-1,0});
//}

/* Returns the number of degrees from the From vector projected into
 * the horizontal plane is away from north.
 *
 * Description of heading algorithm:
 * Shift and scale the magnetic reading based on calibration data to
 * to find the North vector. Use the acceleration readings to
 * determine the Down vector. The cross product of North and Down
 * vectors is East. The vectors East and North form a basis for the
 * horizontal plane. The From vector is projected into the horizontal
 * plane and the angle between the projected vector and north is
 * returned.
 */
int
lsm303_heading(vector from)
{

  /* These are just some values for a particular unit; it is recommended that
   * a calibration be done for your particular unit.
   */
  m_max.x = +540;
  m_max.y = +500;
  m_max.z = 180;
  m_min.x = -520;
  m_min.y = -570;
  m_min.z = -770;

  /* shift and scale*/
  m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
  m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
  m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

  vector temp_a = a;

  /* normalize*/
  lsm303_vector_normalize(&temp_a);

  /* compute E and N*/
  vector E;
  vector No;

  lsm303_vector_cross(&m, &temp_a, &E);
  lsm303_vector_normalize(&E);
  lsm303_vector_cross(&temp_a, &E, &No);

  /* compute heading*/
  int heading =/*round */(atan2(lsm303_vector_dot(&E, &from), lsm303_vector_dot(&No, &from)) * 180 /
   M_PI);
  if(heading < 0)
 {
    heading += 360;
}
  return heading;
}

void
lsm303_vector_cross(const vector * a, const vector * b, vector * out)
{
  out->x = a->y * b->z - a->z * b->y;
  out->y = a->z * b->x - a->x * b->z;
  out->z = a->x * b->y - a->y * b->x;
}

float
lsm303_vector_dot(const vector * a, const vector * b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void
lsm303_vector_normalize(vector * a)
{
  float mag = sqrtf(lsm303_vector_dot(a, a));

  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}
