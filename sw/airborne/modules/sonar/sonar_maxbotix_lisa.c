/*
 *
 * Copyright (C) 2010  Gautier Hattenberger
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

#include "modules/sonar/sonar_maxbotix.h"
#include "mcu_periph/adc.h"

#define PB1 1

#ifndef INS_SONAR_THRESHOLD
#define INS_SONAR_THRESHOLD 100
#endif

uint16_t sonar_meas;
uint16_t sonar_thresh;
uint16_t v1;
uint16_t v2;
uint16_t v3;
bool_t sonar_data_available;

static struct adc_buf sonar_adc;


void maxbotix_init(void) {
  
  sonar_meas = 0;
  sonar_data_available = FALSE;
  sonar_thresh = INS_SONAR_THRESHOLD;

  adc_buf_channel(PB1, &sonar_adc, 0x20);
    
}

/** Read ADC value to update sonar measurement
 */
void maxbotix_read(void) {
  sonar_meas = sonar_adc.sum / sonar_adc.av_nb_sample;
  //sonar_meas = (sonar_adc.values[0]+sonar_adc.values[1]+sonar_adc.values[2]+sonar_adc.values[3])>>2;
  v1 = 0;
  v2 = 0;
  v3 = 0;
  sonar_data_available = TRUE;
}

