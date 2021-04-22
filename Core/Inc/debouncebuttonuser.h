/*
 * debouncebuttonuser.h
 *
 *  Created on: Apr 20, 2021
 *      Author: Alejandro
 */

#ifndef INC_DEBOUNCEBUTTONUSER_H_
#define INC_DEBOUNCEBUTTONUSER_H_

#include "stdint.h"
#include "stm32f0xx_hal.h"

/**
 * @brief Función que devuelve el estado del boton, usando MEF como antirrebote
 * 
 * @param GPIOx     Puerto del micro donde está el boton.
 * @param GPIO_Pin  Número de pin del puerto donde esta el boton
 * @return uint8_t  Devuelve 0 si el boton esta presionado y 1 si no lo esta
 */
uint8_t estadoBoton( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


#endif /* INC_DEBOUNCEBUTTONUSER_H_ */
