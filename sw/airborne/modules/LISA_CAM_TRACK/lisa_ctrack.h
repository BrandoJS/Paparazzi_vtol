/*
 * $Id: demo_module.h 3079 2009-03-11 16:55:42Z gautier $
 *
 * Copyright (C) 2009  Gautier Hattenberger
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

/** lisa_ctrack.h
 *  Simple Camera tracker
 *
 * 
 */

#ifndef LCTRACK_MODULE_H
#define LCTRACK_MODULE_H

#include <inttypes.h>

#ifdef USE_SERVOS_7AND8
#define ACTUATORS_PWM_NB 8
#else
#define ACTUATORS_PWM_NB 6
#endif

extern void l_actuators_pwm_arch_init(void);
extern void l_actuators_pwm_commit(void);

#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(_x)  actuators_pwm_values[_x]
#define SERVOS_TICS_OF_USEC(_v) (_v)



extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

extern uint8_t track_mode;

struct ServoCommands {
	int32_t p;
	int32_t y;
};

extern struct ServoCommands m;

extern int32_t d_level;
extern int32_t x_star;
extern float point;


void init_lisa_ctrack(void);
void periodic_lisa_ctrack(void);
void level(float theta, float d_level);





#endif
