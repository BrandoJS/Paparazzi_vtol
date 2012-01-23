/*
 * $Id: demo_module.c 3079 2009-03-11 16:55:42Z gautier $
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

#include "l_light.h"
#include "led.h"

uint8_t light_1_mode;
uint8_t light_2_mode;

#ifndef LIGHT_1_MODE_DEFAULT
#define LIGHT_1_MODE_DEFAULT 5
#endif

#ifndef LIGHT_2_MODE_DEFAULT
#define LIGHT_2_MODE_DEFAULT 0
#endif


void init_light(void) {
  //Initialize Switches
  SW_INIT();
  SW1_OFF();
  SW2_OFF();

  light_1_mode = LIGHT_1_MODE_DEFAULT;
  light_2_mode = LIGHT_2_MODE_DEFAULT;

}

void periodic_light(void)
{
  static uint8_t counter_1 = 0;
  static uint8_t counter_2 = 0;


  switch (light_1_mode)
  {
    default:	// Always off
	SW1_OFF();
	break;
    case 1:	// Always on
	SW1_ON();
	break;
    case 2:	// Blink
    case 3:
    case 4:
	if (counter_1 == (light_1_mode*5 - 4))
	{
		SW1_OFF()
	}
	else if (counter_1 >= 20)
	{
		SW1_ON()
		counter_1 = 0;
	}
	break;
     case 5:	// Complex Blinking
	if (counter_1 == 3)
	{
		SW1_OFF();
	}
	else if (counter_1 == 4)
	{
		SW1_ON();
	}
	else if (counter_1 == 6)
	{
		SW1_OFF();
	}
	else if (counter_1 == 7)
	{
		SW1_ON();
	}
	else if (counter_1 == 8)
	{
		SW1_OFF();
	}
	else if (counter_1 >= 25)
	{
		SW1_ON();
		counter_1 = 0;
	}
	break;
     case 6:
	if (counter_1 <= 18)
	{
		if ((counter_1 % 2) == 0)
		{
		  SW1_ON();
		}
                else
		{
		  SW1_OFF();
		}
	}
	else if (counter_1 == 35)
	{
		counter_1 = 0;
	}
       break;
  }


  switch (light_2_mode)
  {
    default:	// Always off
	SW2_OFF();
	break;
    case 1:	// Always on
	SW2_ON();
	break;
    case 2:	// Blink
    case 3:
    case 4:
	if (counter_2 == (light_2_mode*5 - 4))
	{
		SW2_OFF();
	}
	else if (counter_2 >= 20)
	{
		SW2_ON();
		counter_2 = 0;
	}
	break;
  }
  counter_1++;
  counter_2++;
}
