/*
 * mydelay.h
 *
 *  Created on: Apr 20, 2021
 *      Author: Alejandro
 */

#ifndef INC_MYDELAY_H_
#define INC_MYDELAY_H_
#include "stdint.h"


//!  Estructura par el manejo del Delay no bloqueante
typedef struct{
   uint32_t startTime; //!< almacena el valor de inicio, leyendo el valor actual del SysTick
   uint16_t duration;  //!< almacena la duración del delay
   uint8_t running;    //!< Indica si el tiempo se ha cumplido(0), o si aún esta corriendo(1)
} delay_t;

/**
 * @brief Función que configura el tiempo y la duración del delay.
 * 
 * @param delay_t   Estructura para almacenar la duración del delay, el tiempo de arranque y el estado 
 * @param duration  tiempo de duración del delay
 */
void delayConfig( delay_t * delay_t, uint16_t duration );

/**
 * @brief Función que lee el estado del Delay. 
 *        La primera vez que es invocada, almacena el tiempo de  SysTick y pone a running en 1.
 *        Para el resto de las llamadas, compara el tiempo actual del SysTick con el que almacenó 
 *        en la primera llamda en startTime y verifica si es mayor a la duración del delay.
 *
 * @param delay_t Puntero a una estructura del tipo delay_t
 * @return uint8_t Devuelve 0 si el delay llegó al tiempo fijado y 1 si aún está contando.
 */
uint8_t delayRead( delay_t * delay_t );

/**
 * @brief Función que se utiliza para cambiar el tiempo de un delay que ya esté configurado.
 * 
 * @param delay_t Estructura para almacenar la duración del delay, el tiempo de arranque y el estado 
 * @param duration tiempo de duración del delay
 */
void delayWrite( delay_t * delay_t , uint16_t duration);


#endif /* INC_MYDELAY_H_ */
