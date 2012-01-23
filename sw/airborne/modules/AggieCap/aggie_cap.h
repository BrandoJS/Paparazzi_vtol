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

/** aggie_cap.h
 *  Aggie Cap Interface for Lisa plus Camera Track Control
 *
 * 
 */

#ifndef AGGIE_CAP_MODULE_H
#define AGGIE_CAP_MODULE_H

#include <inttypes.h>
#include "mcu_periph/uart.h"

#ifdef USE_SERVOS_7AND8
#define ACTUATORS_PWM_NB 8
#else
#define ACTUATORS_PWM_NB 6
#endif

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/tim.h>

#define PCLK 72000000
#define ONE_MHZ_CLK 1000000
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

#ifndef PWM_5AND6_TIMER
#define PWM_5AND6_TIMER TIM4
#define PWM_5AND6_RCC RCC_APB1Periph_TIM4
#define PWM_5AND6_GPIO GPIOB
#define PWM5_OC 3
#define PWM6_OC 4
#define PWM5_Pin GPIO_Pin_8
#define PWM6_Pin GPIO_Pin_9
#endif

#define _TIM_OC_INIT(n) TIM_OC##n##Init
#define TIM_OC_INIT(n) _TIM_OC_INIT(n)

#define _TIM_OC_PRELOADCONFIG(n) TIM_OC##n##PreloadConfig
#define TIM_OC_PRELOADCONFIG(n) _TIM_OC_PRELOADCONFIG(n)

#define _TIM_SETCOMPARE(n) TIM_SetCompare##n
#define TIM_SETCOMPARE(n) _TIM_SETCOMPARE(n)


#ifndef CAMERA_MODE_DEFAULT
#define CAMERA_MODE_DEFAULT 2
#endif

#ifndef TRACK_MODE_DEFAULT
#define TRACK_MODE_DEFAULT 0
#endif

#ifndef CENTER0
#define CENTER0 1500
#endif

#ifndef CENTER1
#define CENTER1 1500
#endif

#ifndef SCALE0
#define SCALE0 -11.1111111
#endif

#ifndef SCALE1
#define SCALE1 -11.1111111
#endif


#define CAM_YAW 1
#define CAM_PIT 0

#define MSG0 0x93
#define MSG1 0xE0
#define GPS_HEADER 0x00
#define IMU_HEADER 0x01
#define COM_HEADER 0x02
/*

#define MSG0 0x41
#define MSG1 0x41

#define GPS_HEADER 0x47
#define IMU_HEADER 0x49
#define COM_HEADER 0x43
*/
#define IMU_HEADER_SIZE 0x06
#define GPS_HEADER_SIZE 0x28
#define COM_HEADER_SIZE 0x01

#define IMU_MESSAGE_SIZE 12
#define GPS_MESSAGE_SIZE 46
#define COM_MESSAGE_SIZE 7

#define TURN_CAM_ON	0x01
#define TURN_CAM_OFF	0x04
#define CAM_TRIGGER	0x02
#define RESTART		0x08

extern void l_actuators_pwm_arch_init(void);
extern void l_actuators_pwm_commit(void);

#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(_x)  actuators_pwm_values[_x]
#define SERVOS_TICS_OF_USEC(_v) (_v)



extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

extern uint8_t track_mode;
extern uint8_t camera_mode;

extern struct Int32Eulers cam_eulers;
extern struct Int32Eulers cam_psi_eulers;
extern struct Int32Eulers cam_theta_eulers;
extern struct Int32Eulers cam_total_eulers;
extern struct Int32RMat   cam_rmat;
extern struct Int32RMat   cam_psi_rmat;
extern struct Int32RMat   cam_theta_rmat;
extern struct Int32RMat   cam_total_rmat;

extern struct ServoCommands m;

extern uint8_t cam_on;

extern int16_t image_phi;
extern int16_t image_the;
extern int16_t image_psi;

extern int32_t image_count;
void init_aggie_cap(void);
void periodic_aggie_cap(void);
void level(float theta, float d_level);


void ugearIMU(void);
void ugearGPS(void);
void ugearCOM(uint8_t command);
void ugear_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );
void send_buf(uint8_t size,uint8_t *_buf);



#define send_byte(x) (uart2_transmit(x))






#endif
