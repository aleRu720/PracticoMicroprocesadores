/*
 * debouncebuttonuser.c
 *
 *  Created on: Apr 20, 2021
 *      Author:  Alejandro
 */

#include "debounceButtonUser.h"

//! Variable privada que contiene el estado actual del boton
static uint8_t currentButtonState;

//static uint8_t ButtonState;

//! Enumeracion de los diferentes estados de la MEF
typedef enum {
    BUTTON_DOWN,
    BUTTON_UP, 
    BUTTON_RISING, 
    BUTTON_FALLING
} estadoMef;

static estadoMef currentState;

/**
 * @brief Función privada que recorre a MEF y actualiza el estado de la misma
 * 
 */
static void actualizaMef(void);


static void actualizaMef(void)
{
	switch(currentState )
	{
		case BUTTON_DOWN:
			if (!currentButtonState)
				currentState = BUTTON_FALLING;
		break;
        
        case BUTTON_UP :
			if (currentButtonState)
				currentState = BUTTON_RISING;
		break;
		
		case BUTTON_RISING:
			if (currentButtonState)
			{
				currentState = BUTTON_DOWN;
			}
			else 
				currentState = BUTTON_UP;
		break;
		case BUTTON_FALLING:
			if (!currentButtonState)
			{
				currentState = BUTTON_UP;
			}
			else 
				currentState = BUTTON_DOWN;
		break;
        default:
            currentState = BUTTON_UP;
	}
	
}

uint8_t estadoBoton( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
    currentButtonState=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
	actualizaMef();
    return currentState;
}
