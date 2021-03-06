/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         Implementation of the energy estimation module
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "sys/energest.h"
#include "contiki-conf.h"



#if ENERGEST_CONF_ON

int energest_total_count;
energest_t energest_total_time[ENERGEST_TYPE_MAX];
rtimer_clock_t energest_current_time[ENERGEST_TYPE_MAX];
#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
energest_t energest_leveldevice_current_leveltime[ENERGEST_CONF_LEVELDEVICE_LEVELS];
#endif
unsigned char energest_current_mode[ENERGEST_TYPE_MAX];

/*---------------------------------------------------------------------------*/
void
energest_init(void)
{
  int i;
  for(i = 0; i < ENERGEST_TYPE_MAX; ++i) {
    energest_total_time[i].current = energest_current_time[i] = 0;
    energest_current_mode[i] = 0;
  }
#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
  for(i = 0; i < ENERGEST_CONF_LEVELDEVICE_LEVELS; ++i) {
    energest_leveldevice_current_leveltime[i].current = 0;
  }
#endif
}
/*---------------------------------------------------------------------------*/
unsigned long
energest_type_time(int type)
{
  /* Note: does not support ENERGEST_CONF_LEVELDEVICE_LEVELS! */
#ifndef ENERGEST_CONF_LEVELDEVICE_LEVELS
  if(energest_current_mode[type]) {
    rtimer_clock_t now = RTIMER_NOW();
    energest_total_time[type].current += (rtimer_clock_t)
      (now - energest_current_time[type]);
    energest_current_time[type] = now;
  }
#endif /* ENERGEST_CONF_LEVELDEVICE_LEVELS */
  return energest_total_time[type].current;
}
/*---------------------------------------------------------------------------*/
unsigned long
energest_leveldevice_leveltime(int powerlevel)
{
#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
  return energest_leveldevice_current_leveltime[powerlevel].current;
#else
  return 0;
#endif
}
/*---------------------------------------------------------------------------*/
void
energest_type_set(int type, unsigned long val)
{
  energest_total_time[type].current = val;
}
/*---------------------------------------------------------------------------*/
/* Note: does not support ENERGEST_CONF_LEVELDEVICE_LEVELS! */
void
energest_flush(void)
{
  rtimer_clock_t now;
  int i;
  for(i = 0; i < ENERGEST_TYPE_MAX; i++) {
    if(energest_current_mode[i]) {
      now = RTIMER_NOW();
      energest_total_time[i].current += (rtimer_clock_t)
	(now - energest_current_time[i]);
      energest_current_time[i] = now;
    }
  }
}

uint16_t get_total_energy_consumption()
{
  SENSORS_ACTIVATE(battery_sensor);
  uint32_t energy_consumed = 0;
  uint32_t cpu, lpm, transmit, listen;
  uint32_t  time;

  energest_flush();

  cpu = energest_type_time(ENERGEST_TYPE_CPU);// - last_cpu;
  lpm = energest_type_time(ENERGEST_TYPE_LPM);// - last_lmp;
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);// - last_transmit;
  listen = energest_type_time(ENERGEST_TYPE_LISTEN);// - last_listen;
  SENSORS_DEACTIVATE(battery_sensor);

  time = cpu + lpm;

  energy_consumed = 3L * ((cpu/RTIMER_ARCH_SECOND) * 2 + (lpm/RTIMER_ARCH_SECOND) / 10 + (transmit/RTIMER_ARCH_SECOND) * 20 + (listen/RTIMER_ARCH_SECOND) * 22);
  
  if (energy_consumed == 0)
  {
    return 1;
  }
   // printf("Node's power consummed(mJ) : %lu\n", energy_consumed);
  // printf("radio-ontime %lu %%\n", (100*(transmit+listen))/time);
  return energy_consumed; 
}

/*---------------------------------------------------------------------------*/
#else /* ENERGEST_CONF_ON */
void energest_type_set(int type, unsigned long val) {}
void energest_init(void) {}
unsigned long energest_type_time(int type) { return 0; }
void energest_flush(void) {}
#endif /* ENERGEST_CONF_ON */
