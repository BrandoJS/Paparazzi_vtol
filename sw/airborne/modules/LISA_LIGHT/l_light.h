/*
 * $Id: l_light.h 
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

/** l_light.h
 *  Light Controller for the Lisa Board using the Power Switches
 *  Requires changes to arch/stm32/led_hw.h and led.h
 * 
 */

#ifndef LISA_LIGHT_MODULE_H
#define LISA_LIGHT_MODULE_H

#include <inttypes.h>


extern uint8_t light_1_mode;
extern uint8_t light_2_mode;

void init_light(void);
void periodic_light(void);

#endif

