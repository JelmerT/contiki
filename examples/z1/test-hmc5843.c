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
 *         A quick program for testing the HMC5843 driver on the Z1 platform
 * \author
 *         Jelmer Tiete <jelmer@tiete.be>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"
#include "dev/hmc5843.h"


#if 1
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#if 1
#define PRINTFDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTFDEBUG(...)
#endif


#define HMC5843_READ_INTERVAL (CLOCK_SECOND)

PROCESS(magn_process, "Test Magnetometer process");
AUTOSTART_PROCESSES(&magn_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;

PROCESS_THREAD(magn_process, ev, data)
{
  PROCESS_BEGIN();

  char id[3];
  int16_t x,y,z;
  float fx,fy,fz;

  hmc5843_init(); //init i2c and hmc5843

  hmc5843_get_id(id); //read last 3 registers for ID
  PRINTF("HMC5843 ID:0x%02X 0x%02X 0x%02X\n",id[0],id[1],id[2]);

  hmc5843_calibrate(1, 64);
  hmc5843_set_mode(0);

  while(1) {
    etimer_set(&et, HMC5843_READ_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    //hmc5843_get_raw(&x, &y, &z);
    PRINTF("x=%i y=%i z=%i\n",x,y,z);
    hmc5843_get_values(&fx, &fy, &fz);
    PRINTF("float: x=%i y=%i z=%i\n",(int16_t)fx,(int16_t)fy,(int16_t)fz); //printf doesn't do floats

    //Calculate heading
    float heading = atan2(fy, fx);
    if(heading < 0) {// Correct for when signs are reversed.
      heading += 2 * 3.14159265359;
    }
    PRINTF("heading: %i\n",(int16_t)(heading*(180/3.14159265359)));
  }
  PROCESS_END();
}
