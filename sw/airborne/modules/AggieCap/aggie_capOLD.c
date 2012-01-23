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

#include "aggie_cap.h"
#include "led.h"
#include "booz_gps.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"


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


uint8_t cam_control_mode;
uint8_t camera_mode;
int32_t image_count;
int32_t image_gps_lat;
int32_t image_gps_lon;
int32_t image_gps_alt;
int32_t image_phi;
int32_t image_the;
int32_t image_psi;
int32_t image_xd;
int32_t image_yd;
int32_t image_zd;

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

#ifndef TRACK_MODE_DEFAULT
#define TRACK_MODE_DEFAULT 2
#endif

#ifndef CAMERA_MODE_DEFAULT
#define CAMERA_MODE_DEFAULT 2
#endif

#define CAM_YAW 1
#define CAM_PIT 0


#define AGGIE_CAP_HEADER 0x56

static int32_t foo;

void init_aggie_cap(void) {
  l_actuators_pwm_arch_init();
  camera_mode = CAMERA_MODE_DEFAULT;
  image_count = 0;
  cam_control_mode = 2;
  
  foo = 0.;
}

void periodic_aggie_cap(void)
{ 
  static uint8_t counter_sw2 = 0;
  float theta = ahrs.ltp_to_body_euler.theta * .0139882;
  float psi = ahrs.ltp_to_body_euler.psi * .0139882;

  switch (cam_control_mode) {
  default: //Center
    for (int i = 0; i < ACTUATORS_PWM_NB; i++) {
    		actuators_pwm_values[i] = 1500;
  	}
    break;
  case 1: //Manual
    //actuators_pwm_values[CAM_PIT] = 11.111 * m.p + 500;
    //actuators_pwm_values[CAM_YAW] = 11.111 * m.y + 500;
  	
	break;
  case 2: //Auto Steady
    foo += 0.0025;
  	int32_t bar = 500 + 2000. * (foo);
  	for (int i = 0; i < ACTUATORS_PWM_NB; i++) {
    		actuators_pwm_values[i] = bar;
  	}
  	
  	if (bar > 2500)
		foo = 0;
	break;

  }
  

  switch (camera_mode)
  {
    default:	// Always off
	CM2_OFF();
	break;
    case 1:	// Toggle
	click();
        camera_mode = 0;
        break;
    case 2:  	//Automatic 
	if (counter_sw2 == 1)
	{
		click();
		
	}
	else if (counter_sw2 >= 10)
	{
		counter_sw2 = 0;
	}
	break;
    
	
  }
  counter_sw2++;	
l_actuators_pwm_commit();
}

void click (void) {
  
  image_count++;/*
  image_gps_lat = booz_gps_state.lla_pos.lat;
  image_gps_lon = booz_gps_state.lla_pos.lon;
  image_gps_alt = booz_gps_state.lla_pos.alt;
  image_xd = ins_enu_speed.x;
  image_yd = ins_enu_speed.y;
  image_zd = ins_enu_speed.z;
  image_phi = ahrs.ltp_to_body_euler.phi;
  image_the = ahrs.ltp_to_body_euler.theta;
  image_psi = ahrs.ltp_to_body_euler.psi;

  uart2_transmit(AGGIE_CAP_HEADER);
  send_int(image_count);    
  send_int(image_phi);
  send_int(image_the);
  send_int(image_psi);
  send_int(image_gps_lat);
  send_int(image_gps_lon);
  send_int(image_gps_alt);
  send_int(image_xd);
  send_int(image_yd);
  send_int(image_zd);
  uart2_transmit(0x44);
  uart2_transmit(0x0A); 
  uart2_transmit(0x0D);
 */
  //send_int(image_count);
  send_int(0x48454C50);
  uart2_transmit(0x0A); 
  uart2_transmit(0x0D);
}

void send_int(int32_t data) {
  uint8_t a = 0;
  for (int8_t i = 0; i <=3 ; i++) {
    a = 0xFF&(data>>(i*8));
    uart2_transmit(a);
  }

}

void l_actuators_pwm_arch_init(void) {

  /* TIM3 and TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(PWM_5AND6_RCC, ENABLE);
#ifdef USE_SERVOS_7AND8
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
#endif

  /* GPIO A,B and C clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
             RCC_APB2Periph_AFIO, ENABLE);
  /* GPIO C */
  /* PC6=servo1 PC7=servo2 PC8=servo3 PC9=servo4 */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* need to remate alternate function, pins 37, 38, 39, 40 on LQFP64 */
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  /* PWM 5/6 GPIO */
  /* PB8=servo5 PB9=servo6 */
  GPIO_InitStructure.GPIO_Pin   = PWM5_Pin | PWM6_Pin;
  GPIO_Init(PWM_5AND6_GPIO, &GPIO_InitStructure);

#ifdef USE_SERVOS_7AND8
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

  /* Time base configuration */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = (PCLK / ONE_MHZ_CLK) - 1; // 1uS
  TIM_TimeBaseStructure.TIM_Period = (ONE_MHZ_CLK / SERVO_HZ) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(PWM_5AND6_TIMER, &TIM_TimeBaseStructure);
#ifdef USE_SERVOS_7AND8
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
#endif

  /* PWM1 Mode configuration: All Channels */
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // default low (no pulse)
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: TIM3 Channel1 */
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM2 Mode configuration: TIM3 Channel2 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM3 Mode configuration: TIM3 Channel3 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM4 Mode configuration: TIM3 Channel4 */
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM5 Mode configuration: TIM4 Channel3 */
  TIM_OC_INIT(PWM5_OC) (PWM_5AND6_TIMER, &TIM_OCInitStructure);
  TIM_OC_PRELOADCONFIG(PWM5_OC)(PWM_5AND6_TIMER, TIM_OCPreload_Enable);

  /* PWM6 Mode configuration: TIM4 Channel4 */
  TIM_OC_INIT(PWM6_OC)(PWM_5AND6_TIMER, &TIM_OCInitStructure);
  TIM_OC_PRELOADCONFIG(PWM6_OC)(PWM_5AND6_TIMER, TIM_OCPreload_Enable);

#ifdef USE_SERVOS_7AND8
  /* PWM7 Mode configuration: TIM4 Channel3 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM8 Mode configuration: TIM4 Channel4 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIM4 enable */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
#endif

  /* PWM1-4 enable */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  /* PWM5/6 enable */
  TIM_ARRPreloadConfig(PWM_5AND6_TIMER, ENABLE);
  TIM_CtrlPWMOutputs(PWM_5AND6_TIMER, ENABLE);
  TIM_Cmd(PWM_5AND6_TIMER, ENABLE);

}

/* set pulse widths from actuator values, assumed to be in us */
void l_actuators_pwm_commit(void) {
  TIM_SetCompare1(TIM3, actuators_pwm_values[0]);
  TIM_SetCompare2(TIM3, actuators_pwm_values[1]);
  TIM_SetCompare3(TIM3, actuators_pwm_values[2]);
  TIM_SetCompare4(TIM3, actuators_pwm_values[3]);

  TIM_SETCOMPARE(PWM5_OC)(PWM_5AND6_TIMER, actuators_pwm_values[4]);
  TIM_SETCOMPARE(PWM6_OC)(PWM_5AND6_TIMER, actuators_pwm_values[5]);

#ifdef USE_SERVOS_7AND8
  TIM_SetCompare1(TIM4, actuators_pwm_values[6]);
  TIM_SetCompare2(TIM4, actuators_pwm_values[7]);
#endif
}
