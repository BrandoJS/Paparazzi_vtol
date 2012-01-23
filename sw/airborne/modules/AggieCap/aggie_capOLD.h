/*
 * $Id: rcam.h
 *
 * Copyright (C) 2011 Brandon Stark
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

/** rcam.h
 *  Simple Camera Controller using LEDs as output
 *
 * 
 */

#ifndef RCAM_MODULE_H
#define RCAM_MODULE_H

#ifdef USE_SERVOS_7AND8
#define ACTUATORS_PWM_NB 8
#else
#define ACTUATORS_PWM_NB 6
#endif

#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(_x)  actuators_pwm_values[_x]
#define SERVOS_TICS_OF_USEC(_v) (_v)


#include <inttypes.h>
#include "mcu_periph/uart.h"


extern uint8_t cam_control_mode;
extern uint8_t camera_mode;
extern int32_t image_count;
extern int32_t image_gps_lat;
extern int32_t image_gps_lon;
extern int32_t image_gps_alt;
extern int32_t image_phi;
extern int32_t image_the;
extern int32_t image_psi;



extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

extern int32_t c_psi;
extern int32_t c_the;



/* Init and Periodic Functions*/
void init_aggie_cap(void);
void periodic_aggie_cap(void);


/* Trigger Functions */

void send_int(int32_t);
void click(void);


/* Camera Targeting Functions*/

extern void l_actuators_pwm_arch_init(void);
extern void l_actuators_pwm_commit(void);

void level(float theta, float d_level);






#endif
