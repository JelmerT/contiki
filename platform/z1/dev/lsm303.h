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


/* -------------------------------------------------------------------------- */
/* Init the accelerometer: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */
void    lsm303_init(void);

/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
void    lsm303_accm_write_reg(uint8_t reg, uint8_t val);

/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from
  First byte in stream must be the register address to begin writing to.
  The data is then written from the second byte and increasing. The address byte
  is not included in length len. */
void    lsm303_accm_write_stream(uint8_t len, uint8_t *data);

/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/
uint8_t    lsm303_accm_read_reg(uint8_t reg);

/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/
void    lsm303_accm_read_stream(uint8_t reg, uint8_t len, uint8_t *whereto);

/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
void    lsm303_magn_write_reg(uint8_t reg, uint8_t val);

/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from
  First byte in stream must be the register address to begin writing to.
  The data is then written from the second byte and increasing. The address byte
  is not included in length len. */
void    lsm303_magn_write_stream(uint8_t len, uint8_t *data);

/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/
uint8_t    lsm303_magn_read_reg(uint8_t reg);

/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/
void    lsm303_magn_read_stream(uint8_t reg, uint8_t len, uint8_t *whereto);

/* Read an axis of the accelerometer (x, y or z). Return value is a signed 10 bit int.
  The resolution of the acceleration measurement can be increased up to 13 bit, but
  will change the data format of this read out. Refer to the data sheet if so is
  wanted/needed. */
void    lsm303_magn_read(void);

/* -------------------------------------------------------------------------- */
/* Application definitions, change if required by application. */

/* Interrupt suppress periods */
/*
// XXX Not used yet.
#define ADXL345_INT_OVERRUN_BACKOFF     CLOCK_SECOND/8
#define ADXL345_INT_WATERMARK_BACKOFF   CLOCK_SECOND/8
#define ADXL345_INT_FREEFALL_BACKOFF    CLOCK_SECOND/8
#define ADXL345_INT_INACTIVITY_BACKOFF  CLOCK_SECOND/8
#define ADXL345_INT_ACTIVITY_BACKOFF    CLOCK_SECOND/8
#define ADXL345_INT_DOUBLETAP_BACKOFF   CLOCK_SECOND/8
#define ADXL345_INT_TAP_BACKOFF         CLOCK_SECOND/8
#define ADXL345_INT_DATAREADY_BACKOFF   CLOCK_SECOND/8
*/
/* Time after an interrupt that subsequent interrupts are suppressed. Should later
  be turned into one specific time per type of interrupt (tap, freefall etc) */
#define SUPPRESS_TIME_INT1    CLOCK_SECOND/4
#define SUPPRESS_TIME_INT2    CLOCK_SECOND/4

/* Suggested defaults according to the data sheet etc */
#define ADXL345_THRESH_TAP_DEFAULT      0x48    // 4.5g (0x30 == 3.0g) (datasheet: 3g++)
#define ADXL345_OFSX_DEFAULT            0x00    // for individual units calibration purposes
#define ADXL345_OFSY_DEFAULT            0x00
#define ADXL345_OFSZ_DEFAULT            0x00
#define ADXL345_DUR_DEFAULT             0x20    // 20 ms (datasheet: 10ms++)
#define ADXL345_LATENT_DEFAULT          0x50    // 100 ms (datasheet: 20ms++)
#define ADXL345_WINDOW_DEFAULT          0xFF    // 320 ms (datasheet: 80ms++)
#define ADXL345_THRESH_ACT_DEFAULT      0x15    // 1.3g (62.5 mg/LSB)
#define ADXL345_THRESH_INACT_DEFAULT    0x08    // 0.5g (62.5 mg/LSB)
#define ADXL345_TIME_INACT_DEFAULT      0x02    // 2 s (1 s/LSB)
#define ADXL345_ACT_INACT_CTL_DEFAULT   0xFF    // all axis involved, ac-coupled
#define ADXL345_THRESH_FF_DEFAULT       0x09    // 563 mg
#define ADXL345_TIME_FF_DEFAULT         0x20    // 160 ms
#define ADXL345_TAP_AXES_DEFAULT        0x07    // all axis, no suppression

#define ADXL345_BW_RATE_DEFAULT         (0x00|ADXL345_SRATE_100)   // 100 Hz, normal operation
#define ADXL345_POWER_CTL_DEFAULT       0x28	  // link bit set, no autosleep, start normal measuring
#define ADXL345_INT_ENABLE_DEFAULT      0x00    // no interrupts enabled
#define ADXL345_INT_MAP_DEFAULT         0x00    // all mapped to int_1

/* XXX NB: In the data format register, data format of axis readings is chosen
  between left or right justify. This affects the position of the MSB/LSB and is
  different depending on g-range and resolution. If changed, make sure this is
  reflected in the _read_axis() function. Also, the resolution can be increased
  from 10 bit to at most 13 bit, but this also changes position of MSB etc on data
  format so check this in read_axis() too. */
#define ADXL345_DATA_FORMAT_DEFAULT     (0x00|ADXL345_RANGE_2G)    // right-justify, 2g, 10-bit mode, int is active high
#define ADXL345_FIFO_CTL_DEFAULT        0x00    // FIFO bypass mode

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

/* Accelerometer hardware ports, pins and registers on the msp430 µC */
#define ADXL345_DIR        P1DIR
#define ADXL345_PIN        P1PIN
#define ADXL345_REN        P1REN
#define ADXL345_SEL        P1SEL
#define ADXL345_SEL2       P1SEL2
#define ADXL345_INT1_PIN   (1<<6)            // P1.6
#define ADXL345_INT2_PIN   (1<<7)            // P1.7
#define ADXL345_IES        P1IES
#define ADXL345_IE         P1IE
#define ADXL345_IFG        P1IFG
#define ADXL345_VECTOR     PORT1_VECTOR

/* g-range for DATA_FORMAT register */
#define ADXL345_RANGE_2G    0x00
#define ADXL345_RANGE_4G    0x01
#define ADXL345_RANGE_8G    0x02
#define ADXL345_RANGE_16G   0x03


/* The adxl345 has programmable sample rates, but unexpected results may occur if the wrong 
  rate and I2C bus speed is used (see datasheet p 17). Sample rates in Hz. This
  setting does not change the internal sampling rate, just how often it is piped
  to the output registers (ie the interrupt features use the full sample rate
  internally).

  Example use:
    adxl345_set_reg(ADXL345_BW_RATE, ((_ADXL345_STATUS & LOW_POWER) | ADXL345_SRATE_50));
  */
#define ADXL345_SRATE_3200      0x0F    // XXX NB don't use at all as I2C data rate<= 400kHz (see datasheet)
#define ADXL345_SRATE_1600      0x0E    // XXX NB don't use at all as I2C data rate<= 400kHz (see datasheet)
#define ADXL345_SRATE_800       0x0D    // when I2C data rate == 400 kHz
#define ADXL345_SRATE_400       0x0C    // when I2C data rate == 400 kHz
#define ADXL345_SRATE_200       0x0B    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_100       0x0A    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_50        0x09    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_25        0x08    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_12_5      0x07    // 12.5 Hz, when I2C data rate >= 100 kHz
#define ADXL345_SRATE_6_25      0x06    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_3_13      0x05    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_1_56      0x04    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_0_78      0x03    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_0_39      0x02    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_0_20      0x01    // when I2C data rate >= 100 kHz
#define ADXL345_SRATE_0_10      0x00    // 0.10 Hz, when I2C data rate >= 100 kHz


/* -------------------------------------------------------------------------- */
#endif /* ifndef __ADXL345_H__ */
