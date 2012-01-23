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

#include <inttypes.h>



extern uint8_t power_mode;
extern uint8_t camera_mode;
extern int32_t image_count;
extern int32_t image_gps_lat;
extern int32_t image_gps_lon;
extern int32_t image_gps_alt;
extern int32_t image_phi;
extern int32_t image_the;
extern int32_t image_psi;

void init_rcam(void);
void periodic_rcam(void);




#endif
