/*
 * mydelay.c
 *
 *  Created on: Apr 20, 2021
 *      Author: Alejandro
 */
#include "myDelay.h"
#include "stm32f0xx_hal.h"

/* ---- Non Blocking Delay ---- */

void delayConfig( delay_t * delay, uint16_t duration ){
   delay->duration = duration;
   delay->running = 0;
}

uint8_t delayRead( delay_t * delay ){

   uint8_t timeArrived = 0;

   if( !delay->running ){
      delay->startTime = HAL_GetTick();
      delay->running = 1;
   }
   else{
      if ( (HAL_GetTick() - delay->startTime) >= delay->duration ){
         timeArrived = 1;
         delay->running = 0;
      }
   }

   return timeArrived;
}

void delayWrite( delay_t * delay, uint16_t  duration ){
   delay->duration = duration;
}
