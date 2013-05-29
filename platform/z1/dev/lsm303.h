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
 *         Device drivers header file for LSM303DLM 3-axis accelerometer and 
 *         3-axis magnetometer on Zolertia Z1.
 *         Datasheet: http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF251902
 * \author
 *         Jelmer Tiete, VUB <jelmer@tiete.be>
 */

#ifndef __LSM303_H__
#define __LSM303_H__
#include <stdio.h>
#include "dev/i2cmaster.h"

/* Used in lsm303_xxx_read_axis(), eg lsm303_accm_read_axis(X_AXIS);*/
enum LSM303_AXIS {
  X_AXIS = 0,
  Y_AXIS = 2,
  Z_AXIS = 4,
};

typedef struct vector {
  float x, y, z;
} vector;

vector a;                       // accelerometer readings
vector m;                       // magnetometer readings
vector m_max;                   // maximum magnetometer values, used for calibration
vector m_min;                   // minimum magnetometer values, used for calibration

/* -------------------------------------------------------------------------- */
/* Init the accelerometer: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */
void lsm303_init(void);

/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
void lsm303_accm_write_reg(uint8_t reg, uint8_t val);

/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from
  First byte in stream must be the register address to begin writing to.
  The data is then written from the second byte and increasing. The address byte
  is not included in length len. */
void lsm303_accm_write_stream(uint8_t len, uint8_t * data);

/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/
uint8_t lsm303_accm_read_reg(uint8_t reg);

/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/
void lsm303_accm_read_stream(uint8_t reg, uint8_t len, uint8_t * whereto);

/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
void lsm303_magn_write_reg(uint8_t reg, uint8_t val);

/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from
  First byte in stream must be the register address to begin writing to.
  The data is then written from the second byte and increasing. The address byte
  is not included in length len. */
void lsm303_magn_write_stream(uint8_t len, uint8_t * data);

/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/
uint8_t lsm303_magn_read_reg(uint8_t reg);

/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/
void lsm303_magn_read_stream(uint8_t reg, uint8_t len, uint8_t * whereto);

/* Read an axis of the accelerometer (x, y or z). Return value is a signed 10 bit int.
  The resolution of the acceleration measurement can be increased up to 13 bit, but
  will change the data format of this read out. Refer to the data sheet if so is
  wanted/needed. */
void lsm303_magn_read(void);
void lsm303_accm_read(void);

//int lsm303_heading(void);
int lsm303_heading(vector from);

void vector_cross(const vector * a, const vector * b, vector * out);
float vector_dot(const vector * a, const vector * b);
void vector_normalize(vector * a);


/* -------------------------------------------------------------------------- */
/* Application definitions, change if required by application. 
 */


/* -------------------------------------------------------------------------- */
/* Reference definitions, should not be changed */

/* lsm303 slave address */
#define LSM303_ACC_ADDR            0x18
#define LSM303_MAG_ADDR            0x1E

/* lsm303 registers */
// register addresses

#define LSM303_CTRL_REG1_A       0x20
#define LSM303_CTRL_REG2_A       0x21
#define LSM303_CTRL_REG3_A       0x22
#define LSM303_CTRL_REG4_A       0x23
#define LSM303_CTRL_REG5_A       0x24
#define LSM303_HP_FILTER_RESET_A 0x25
#define LSM303_REFERENCE_A       0x26
#define LSM303_STATUS_REG_A      0x27

#define LSM303_OUT_X_L_A         0x28
#define LSM303_OUT_X_H_A         0x29
#define LSM303_OUT_Y_L_A         0x2A
#define LSM303_OUT_Y_H_A         0x2B
#define LSM303_OUT_Z_L_A         0x2C
#define LSM303_OUT_Z_H_A         0x2D

#define LSM303_INT1_CFG_A        0x30
#define LSM303_INT1_SRC_A        0x31
#define LSM303_INT1_THS_A        0x32
#define LSM303_INT1_DURATION_A   0x33
#define LSM303_INT2_CFG_A        0x34
#define LSM303_INT2_SRC_A        0x35
#define LSM303_INT2_THS_A        0x36
#define LSM303_INT2_DURATION_A   0x37

#define LSM303_CRA_REG_M         0x00
#define LSM303_CRB_REG_M         0x01
#define LSM303_MR_REG_M          0x02

#define LSM303_OUT_X_H_M         0x03
#define LSM303_OUT_X_L_M         0x04
#define LSM303_OUT_Y_H_M         0x07
#define LSM303_OUT_Y_L_M         0x08
#define LSM303_OUT_Z_H_M         0x05
#define LSM303_OUT_Z_L_M         0x06

#define LSM303_SR_REG_M          0x09
#define LSM303_IRA_REG_M         0x0A
#define LSM303_IRB_REG_M         0x0B
#define LSM303_IRC_REG_M         0x0C

#define LSM303_WHO_AM_I_M        0x0F

/* -------------------------------------------------------------------------- */
#endif /* ifndef __LSM303_H__ */
