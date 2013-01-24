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
 *         Device drivers header file for hmc5843 magnetometer sensor on Zolertia Z1.
 * \author
 *         Jelmer Tiete, VUB <jelmer@tiete.be>
 *         Enric M. Calvo, Zolertia <ecalvo@zolertia.com>
 *         Marcus Lundén, SICS <mlunden@sics.se>
 */

#ifndef __HMC5843_H__
#define __HMC5843_H__
#include <stdio.h>
#include "i2cmaster.h"

/* -------------------------------------------------------------------------- */
/* Init the temperature sensor: ports, pins, I2C, interrupts (XXX none so far),
*/
void  hmc5843_init(void);

/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
void    hmc5843_write_reg(uint8_t reg, uint8_t val);

/* Read heading in raw format
    no args needed
*/
uint16_t hmc5843_get_values();

/* -------------------------------------------------------------------------- */
/* Reference definitions */
/* hmc5843 slave address */
#define HMC5843_ADDR           0x1E // this is the real slave address 0x3C for write 0x3D for read

/* hmc5843 registers */
#define HMC5843_CONFIG_A       0x00
#define HMC5843_CONFIG_B       0x01
#define HMC5843_MODE           0x02
#define HMC5843_X_MSB          0x03
#define HMC5843_X_LSB          0x04
#define HMC5843_Y_MSB          0x05
#define HMC5843_Y_LSB          0x06
#define HMC5843_Z_MSB          0x07
#define HMC5843_Z_LSB          0x08
#define HMC5843_STATUS         0x09
#define HMC5843_ID_A           0x10
#define HMC5843_ID_B           0x11
#define HMC5843_ID_C           0x12

/* -------------------------------------------------------------------------- */
#endif /* ifndef __HMC5843_H__ */



