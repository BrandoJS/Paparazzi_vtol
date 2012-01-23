/*
 * $Id: rcam.c
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

#include "rcam.h"
#include "led.h"
#include "booz_gps.h"
#include "subsystems/ahrs.h"

uint8_t power_mode;
uint8_t camera_mode;
int32_t image_count;
int32_t image_gps_lat;
int32_t image_gps_lon;
int32_t image_gps_alt;
int32_t image_phi;
int32_t image_the;
int32_t image_psi;

#ifndef POWER_MODE_DEFAULT
#define POWER_MODE_DEFAULT 0
#endif

#ifndef CAMERA_MODE_DEFAULT
#define CAMERA_MODE_DEFAULT 0
#endif


void init_rcam(void) {
  //Initialize Switches
  CM_INIT();
  CM1_OFF();
  CM2_OFF();


  power_mode = POWER_MODE_DEFAULT;
  camera_mode = CAMERA_MODE_DEFAULT;
  image_count = 0;

}

void periodic_rcam(void)
{
  static uint8_t counter_sw1 = 0;
  static uint8_t counter_sw2 = 0;
  

  switch (power_mode)
  {
    default:	// Always off/Waiting state
	CM1_OFF();
	break;
    case 1:	// ON
        CM1_ON();
	break;
    case 2:
	if (counter_sw1 == 2)
	{
		CM1_OFF();
		
	}
	else if (counter_sw1 >= 4)
	{
		CM1_ON();
		counter_sw1 = 0;
	}
	break;
  }


  switch (camera_mode)
  {
    default:	// Always off
	CM2_OFF();
	break;
    case 1:	// Toggle
	if (counter_sw2 == 2)
	{
		CM2_OFF();
		camera_mode = 0;
	}
	else if (counter_sw2 >= 10)
	{
		CM2_ON();
		counter_sw2 = 0;
		image_count++;
	}
	break;
    case 2:  	//Automatic 
	if (counter_sw2 == 2)
	{
		CM2_OFF();
		
	}
	else if (counter_sw2 >= 10)
	{
		CM2_ON();
		counter_sw2 = 0;
		image_count++;
		image_gps_lat = booz_gps_state.lla_pos.lat;
		image_gps_lon = booz_gps_state.lla_pos.lon;
		image_gps_alt = booz_gps_state.lla_pos.alt;
		image_phi = 0;
		image_the = 0;
		image_psi = 0;
	}
	break;
    
	
  }
  

  counter_sw1++;
  counter_sw2++;
}

