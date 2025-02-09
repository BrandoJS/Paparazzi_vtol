/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009-2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*
 *\brief STM32 timing functions
 *
 */

#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#include "sys_time.h"

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "std.h"
#ifdef SYS_TIME_LED
#include "led.h"
#endif

#define InitSysTimePeriodic()

extern void sys_time_init( void );
extern void sys_tick_irq_handler(void);

extern volatile bool_t sys_time_period_elapsed;
extern uint32_t cpu_time_ticks;

#define SYS_TICS_OF_SEC(s)        (uint32_t)((s) * AHB_CLK + 0.5)
#define SIGNED_SYS_TICS_OF_SEC(s)  (int32_t)((s) * AHB_CLK + 0.5)

/* Generic timer macros */
//FIXME: dirty temporary hack!!!
#define SysTimeTimerStart(_t) { _t = SYS_TICS_OF_SEC(cpu_time_sec) + cpu_time_ticks; }
#define SysTimeTimer(_t) ((uint32_t)(SYS_TICS_OF_SEC(cpu_time_sec) + cpu_time_ticks - _t))
#define SysTimeTimerStop(_t) { _t = (SYS_TICS_OF_SEC(cpu_time_sec) + cpu_time_ticks - _t); }

static inline bool_t sys_time_periodic( void ) {
  if (sys_time_period_elapsed) {
    sys_time_period_elapsed = FALSE;
    cpu_time_ticks += PERIODIC_TASK_PERIOD;
    if (cpu_time_ticks >= TIME_TICKS_PER_SEC) {
      cpu_time_ticks -= TIME_TICKS_PER_SEC;
      cpu_time_sec++;
#ifdef SYS_TIME_LED
      LED_TOGGLE(SYS_TIME_LED);
#endif
    }
    return TRUE;
  }
  else
    return FALSE;
}

/** Busy wait, in microseconds */
/* for now empty shell */
static inline void sys_time_usleep(uint32_t us) {

}

#endif /* SYS_TIME_HW_H */
