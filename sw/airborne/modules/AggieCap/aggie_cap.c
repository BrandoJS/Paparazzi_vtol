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

#include "aggie_cap.h"

#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "booz_gps.h"



int32_t actuators_pwm_values[ACTUATORS_PWM_NB];
uint8_t track_mode;
uint8_t camera_mode;
;
float psi_c;
float theta_c;

uint8_t IMU_buf[IMU_MESSAGE_SIZE];
uint8_t GPS_buf[GPS_MESSAGE_SIZE];
uint8_t COM_buf[COM_MESSAGE_SIZE];

int16_t image_phi;
int16_t image_the;
int16_t image_psi;

int32_t image_lon;
int32_t image_lat;
int32_t image_alt;
int32_t image_vsp;
int32_t image_hea;
uint32_t image_pac;
uint32_t image_sac;
uint32_t image_gsp;
uint32_t image_tow;
uint16_t image_pdp;
uint8_t image_nSV;
uint8_t image_bbb;
int32_t image_count;
uint8_t cam_on;

struct Int32Eulers cam_eulers;
struct Int32Eulers cam_psi_eulers;
struct Int32Eulers cam_theta_eulers;
struct Int32Eulers cam_total_eulers;
struct Int32RMat   cam_rmat;
struct Int32RMat   cam_psi_rmat;
struct Int32RMat   cam_theta_rmat;
struct Int32RMat   cam_total_rmat;


static float foo;

void init_aggie_cap(void) {
	l_actuators_pwm_arch_init();

        camera_mode = 4;//CAMERA_MODE_DEFAULT;
	track_mode = 0;//TRACK_MODE_DEFAULT;
        foo = 0.;
	image_count = 0;
	cam_on = 0;

	/* Message Buffer Headers never change */
	IMU_buf[0] = MSG0;
	GPS_buf[0] = MSG0;
	COM_buf[0] = MSG0; 
	IMU_buf[1] = MSG1;
	GPS_buf[1] = MSG1;	
	COM_buf[1] = MSG1;
	IMU_buf[2] = IMU_HEADER;
	GPS_buf[2] = GPS_HEADER;
	COM_buf[2] = COM_HEADER;
	IMU_buf[3] = IMU_HEADER_SIZE;	
	GPS_buf[3] = GPS_HEADER_SIZE;	
	COM_buf[3] = COM_HEADER_SIZE;	

}

void periodic_aggie_cap(void)
{
  static uint16_t counter_sw2 = 0;
  
  switch (track_mode)
  {
    default:	// Always off/Waiting state
	for (int i = 0; i < ACTUATORS_PWM_NB; i++) {
    		actuators_pwm_values[i] = 1500;
  	}
  	break;
    case 1:	// Manual Control
        
	/*
	actuators_pwm_values[0] = 11.111 * m.p + 500;
	actuators_pwm_values[1] = 11.111 * m.y + 500;
  	*/
	break;
    case 2:  //Auto
	
  	foo += 0.0025;
  	int32_t bar = 500 + 2000. * (foo);
  	for (int i = 0; i < ACTUATORS_PWM_NB; i++) {
    		actuators_pwm_values[i] = bar;
  	}
  	
  	if (bar > 2500)
		foo = 0;
	break;
    case 3:  //Auto
	foo = 0;
  	//point = 0;
        //psi_c = psi-point;
        psi_c = ahrs.ltp_to_body_euler.psi * .0139882;
        actuators_pwm_values[CAM_YAW] = 11.111 * (psi_c) + 1500;
        EULERS_ASSIGN(cam_eulers, 0, 0, ahrs.ltp_to_body_euler.psi);
        INT32_RMAT_OF_EULERS(cam_rmat,cam_eulers);
        INT32_RMAT_COMP_INV(cam_psi_rmat,cam_rmat,ahrs.ltp_to_imu_rmat);
        INT32_EULERS_OF_RMAT(cam_psi_eulers,cam_psi_rmat);
        theta_c = cam_psi_eulers.theta * 0.0139882;
        actuators_pwm_values[CAM_PIT] = 11.111*theta_c + 1500;
        EULERS_ASSIGN(cam_theta_eulers, 0, cam_psi_eulers.theta, 0);
        INT32_RMAT_OF_EULERS(cam_theta_rmat,cam_theta_eulers);
        INT32_RMAT_COMP_INV(cam_total_rmat,cam_theta_rmat,cam_psi_rmat);
        INT32_EULERS_OF_RMAT(cam_total_eulers,cam_total_rmat);
	break;
       
  }

  /* Always stream IMU data*/
  if ((counter_sw2 % 3)==0) { //every 30 ms
	ugearIMU();
  }
  if ((counter_sw2 % 12)==1) { //every 150 ms 
	ugearGPS();
  }
  
  
  switch (camera_mode)
  {
    default:	// Idle
	
	break;
    case 1:	// Turn Cam On
	ugearCOM(TURN_CAM_ON);
        camera_mode = 0;
	cam_on = 1;
        break;
    case 2:	//Turn Cam Off
	ugearCOM(TURN_CAM_OFF);
	cam_on = 0;
        camera_mode = 0;
	break;
    case 3:	//Manual Trigger
        if (cam_on) {
		ugearCOM(CAM_TRIGGER);	
                image_count++;
        }
        camera_mode = 0;
	break; 
    case 4: //Auto
	if (cam_on) {
           if (counter_sw2 > 400) {
                counter_sw2 = 0;
                ugearCOM(CAM_TRIGGER);
		image_count++;
           }
 	}  else { camera_mode = 0;}
        break;
     case 5: //Stop Auto
        camera_mode = 0;
        break;
     case 6: //Restart
	ugearCOM(RESTART);
        cam_on = 0;
        camera_mode = 0;
        break;
	
  
  
  } //End time modulation
  counter_sw2++;
  
  l_actuators_pwm_commit();
 
}


void ugearCOM(uint8_t command) {
  uint8_t cksum0, cksum1;
  COM_buf[4] = command;
  ugear_cksum(COM_HEADER, COM_HEADER_SIZE, (uint8_t *)COM_buf, &cksum0, &cksum1 );
  COM_buf[5] = cksum0;
  COM_buf[6] = cksum1;
  send_buf(COM_MESSAGE_SIZE,COM_buf);
}

void ugearIMU(void) {
  uint8_t cksum0, cksum1;
  if (track_mode == 0){
  image_phi = (int16_t)( (float)ahrs.ltp_to_body_euler.phi*2.422866068);
  image_the = (int16_t)( (float)ahrs.ltp_to_body_euler.theta*2.422866068);
  image_psi = (int16_t)( (float)ahrs.ltp_to_body_euler.psi*2.422866068);
  } else { 
  image_phi = 0;
  image_the = 0;
  image_psi = 0;
  }
  IMU_buf[4] = 0xFF&image_phi;
  IMU_buf[5] = 0xFF&(image_phi>>8);
  IMU_buf[6] = 0xFF&image_the;
  IMU_buf[7] = 0xFF&(image_the>>8);
  IMU_buf[8] = 0xFF&image_psi;
  IMU_buf[9] = 0xFF&(image_psi>>8);

  ugear_cksum(IMU_HEADER, IMU_HEADER_SIZE, (uint8_t *)IMU_buf, &cksum0, &cksum1 );
  IMU_buf[10] = cksum0;
  IMU_buf[11] = cksum1;
  send_buf(IMU_MESSAGE_SIZE,IMU_buf);

}

void ugearGPS(void) {
  uint8_t cksum0, cksum1;
  image_lon = booz_gps_state.lla_pos.lon;
  image_lat = booz_gps_state.lla_pos.lat;
  image_alt = booz_gps_state.lla_pos.alt; 
  image_vsp = booz_gps_state.vsd;
  image_hea = booz_gps_state.heading;
  image_pac = booz_gps_state.pacc;
  image_sac = booz_gps_state.sacc;
  image_gsp = booz_gps_state.Gspeed;
  image_tow = booz_gps_state.tow;
  image_pdp = booz_gps_state.pdop;
  image_nSV = booz_gps_state.tow;
  image_bbb = 0x44;

  for (uint8_t i = 0;i<4;i++){
	GPS_buf[4+i] = 0xFF&(image_lon>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[8+i] = 0xFF&(image_lat>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[12+i] = 0xFF&(image_alt>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[16+i] = 0xFF&(image_vsp>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[20+i] = 0xFF&(image_hea>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[24+i] = 0xFF&(image_pac>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[28+i] = 0xFF&(image_sac>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[32+i] = 0xFF&(image_gsp>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[36+i] = 0xFF&(image_tow>>(i*8));
  }
  for (uint8_t i = 0;i<2;i++){
	GPS_buf[40+i] = 0xFF&(image_pdp>>(i*8));
  }
  GPS_buf[42] = image_nSV;
  GPS_buf[43] = image_bbb;

  ugear_cksum(GPS_HEADER, GPS_HEADER_SIZE, (uint8_t *)GPS_buf, &cksum0, &cksum1 );
  GPS_buf[44] = cksum0;
  GPS_buf[45] = cksum1;
  send_buf(GPS_MESSAGE_SIZE,GPS_buf);
  

}

void send_buf(uint8_t size, uint8_t *_buf){
  for (uint8_t i = 0;i<size;i++){
  	uart2_transmit(_buf[i]);
  }
}

void ugear_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 ) {
  uint8_t c0 = 0;
  uint8_t c1 = 0;
  uint8_t i = 0;
  uint8_t size = hdr2;

  c0 += hdr1;
  c1 += c0;
  
  c0 += hdr2;
  c1 += c0;

  for ( i = 4; i < (size+4); i++ ) {
  	c0 += (uint8_t)buf[i];
    	c1 += c0;
  }

  
  *cksum0 = c0;
  *cksum1 = c1;
  /**cksum0 = 0x0A;
  *cksum1 = 0x0D;*/

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

