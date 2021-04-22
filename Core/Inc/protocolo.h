/*
 * protocolo.h
 *
 *  Created on: Apr 20, 2021
 *      Author:  Alejandro
 */

#ifndef INC_PROTOCOLO_H_
#define INC_PROTOCOLO_H_

#include "stdint.h"
#include <myDelay.h>
#include "stm32f0xx_hal.h"

/**
 * @brief Función publica que lee los datos que llegaron a la UART
 * 
 * @return uint8_t Devuelde el ID del comando recibido.
 */
uint8_t leerDatos(void);

/**
 * @brief Función publica que envía los datos de los sensores a la PC
 * 
 * @param adcData Puntero a un arreglo que contiene los valores del ADC;
 * @return uint8_t DEvuelve Ok si se pudieron enviar los datos
 */
uint8_t enviarDatos(uint16_t *adcData);


uint8_t leerAck(void);


#endif /* INC_PROTOCOLO_H_ */
