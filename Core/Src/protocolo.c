/*
 * protocolo.c
 *
 *  Created on: Apr 20, 2021
 *      Author: Alejandro
 */
#include "protocolo.h"

#define OK 1
#define NOK 0

static uint8_t indexrx=0, headerrx=0, timeoutrx=0, nbytesrx=0, cksrx=0, ackHeader, ackTxCplt;

//! estructura para el puerto serie, tiene los buffers circulares de entrada y salida,
//! los indices de lectura y escritura de dichos buffers.
typedef struct{
	uint8_t  outBuff[256];	//!< Buffer circular de salida de datos (Tx)
	uint8_t  inBuff[256];	//!< Buffer circular de entrada de datos (Rx)
	uint8_t  txRead;		//!< Indice de lectura del buffer de salida de datos
	uint8_t  txWrite;		//!< Indice de escritura del buffer de salida de datos
	uint8_t  rxRead;		//!< Indice de lectura del buffer de entrada de datos
	uint8_t  rxWrite;		//!< Indice de escritura del buffer de entrada de datos
	uint8_t  rxdata[50];	//!< Vector para el manejo de datos temporales
    uint16_t adcChanel[3];	//!< Vector para almacenar los datos de los canales del ADC.
}_sSerial;

//! Variable del tipo sSerial de la UART 4
static _sSerial usart1Com;

//! Unión para descomponer los datos de mas de 8 bits en datos de 8 bits para su envío por el puerto serie,
//! También sirve para componer los datos recibidos por el puerto serie en 8 bits, en datos de mayor tamaño.
typedef union {
	    float f32;
	    uint32_t ui32;
		int32_t i32;
	    int16_t i16[2];
	    uint16_t ui16[2];
	    uint8_t ui8[4];
} _uWord;

//! Variable del tipo uword
static _uWord myWord;

//! Vector para recibir los datos que llegan a la UART 4. 
static uint8_t dataRecive[2];

//! Manejador de la UART 4 definido en main.
extern UART_HandleTypeDef huart4;

/**
 * @brief Función privada para el envío de datos por la UART hacia la PC 
 * 
 * @param data Recibe una estructura del tipo sSerial
 */
static void enviar_byte(_sSerial *data);

/**
 * @brief Función que decodifica los datos recibidos por medio de un protocolo.
 * 
 * @param data Recibe una estructura del tipo sSerial
 */
static void decodeHeader(_sSerial *data);

/**
 * @brief Función que decodifica y devuelve, si fuera el caso, el comando recibido desde la PC,
 *		  también se utiliza cuando se quieren enviar datos para armar el envío de acuerdo al 
 *        protocolo de comunicación.
 * 
 * @param data Recibe una estructura del tipo sSerial
 */
static void DecodeCommand(_sSerial *data);


/// *************************************************************************************************************************
/// ***************************************************   INICIA RUTINAS DE RECPCIÓN Y TRANSMISION UART  ********************
/// *************************************************************************************************************************

/**
 * @brief Función llamada por interrupción cuando se recibe un dato en la UART
 * 
 * @param huart Manejador de la UART 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART4) {
		HAL_UART_Receive_IT(&huart4, dataRecive, 1);
		usart1Com.inBuff[usart1Com.rxWrite++]=dataRecive[0];
	}
}

/**
 * @brief Función llamada por interrupción cuando se completa el envío de datos por la UART
 *
 * @param huart Manejador de la UART
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART4) {
		ackTxCplt=OK;
	}
}


static void enviar_byte(_sSerial *data){
	uint8_t nbytes;
	uint8_t tempBuff[256];
	if(data->txRead > data->txWrite )
		nbytes=((256-data->txRead) + data->txWrite);
	else
		nbytes=(data->txWrite - data->txRead);
	if(nbytes!=0){

		for(uint8_t a=0;a<nbytes;a++)
					tempBuff[a]=data->outBuff[data->txRead+a];

		HAL_UART_Transmit_IT(&huart4, tempBuff, nbytes);
	}
	data->txRead=data->txWrite;
}


static void decodeHeader(_sSerial *data){
	unsigned char msb_nbytes=0;
	while(data->rxRead!=data->rxWrite ){
		switch(headerrx){
			case 0:
			if(data->inBuff[(data->rxRead)++] == 'U'){
				headerrx = 1;
				timeoutrx = 5;
			}
			break;
			case 1:
			if(data->inBuff[data->rxRead] == 'N')
			{
				(data->rxRead)++;
				headerrx = 2;
			}
			else{
				if(data->inBuff[data->rxRead]!='U')
				headerrx = 0;

			}
			break;
			case 2:
			if(data->inBuff[data->rxRead] == 'E')
			{
				(data->rxRead)++;
				headerrx = 3;
			}
			else{
				if(data->inBuff[data->rxRead]!='U')
				headerrx = 0;
				else
				headerrx = 1;
			}
			break;
			case 3:
			if(data->inBuff[data->rxRead] == 'R'){
				(data->rxRead)++;
				headerrx = 4;
			}
			else{
				if(data->inBuff[data->rxRead]!='U')
				headerrx = 0;
				else
				headerrx = 1;
			}
			break;
			case 4:
			nbytesrx = data->inBuff[(data->rxRead)++];
			headerrx = 5;
			break;
			case 5:
			msb_nbytes = data->inBuff[(data->rxRead)++];
			headerrx = 6;
			break;
			case 6:
			if(data->inBuff[(data->rxRead)++] == ':'){
				headerrx = 7;
				cksrx = 'U' ^ 'N' ^ 'E' ^ 'R' ^ nbytesrx ^ msb_nbytes ^ ':';
				data->rxdata[0] = nbytesrx;
				indexrx = 1;
			}
			else{
				if(data->inBuff[(data->rxRead)]!='U')
				headerrx = 0;
				else
				headerrx =1;
			}
			break;
			case 7:
			if(nbytesrx > 1){
				cksrx ^= data->inBuff[(data->rxRead)];
				data->rxdata[indexrx++]= data->inBuff[(data->rxRead)++];
			}
			nbytesrx--;
			if(nbytesrx == 0){
				headerrx = 0;
				if(data->inBuff[(data->rxRead)] == cksrx){
				    DecodeCommand(data);
                }
			}
			break;
			default:
				headerrx = 0;
		}
	}
}


static void DecodeCommand(_sSerial *data)
{
    uint8_t inicioTx=0, cheksum=0;
    unsigned char buffer[256];
    //! 0x0D ACK
    //! 0X0A Envío de datos del ADC

    buffer[inicioTx++]='U';
    buffer[inicioTx++]='N';
    buffer[inicioTx++]='E';
    buffer[inicioTx++]='R';
    buffer[inicioTx++]=0x00;
    buffer[inicioTx++]=0x00;
    buffer[inicioTx++]=':';

    switch(data->rxdata[1]) {
        case 0xA0: //!< Valor del ADC several channels
            buffer[inicioTx++]=0xA0;
           	myWord.ui16[0]= data->adcChanel[0];
			buffer[inicioTx++]=myWord.ui8[0];
			buffer[inicioTx++]=myWord.ui8[1];
            myWord.ui16[0]= data->adcChanel[0];
			buffer[inicioTx++]=myWord.ui8[0];
			buffer[inicioTx++]=myWord.ui8[1];
            buffer[4]=0x06;
            //! Checksum y carga de datos en el buffer de trasnmisión de la estructura.
            for(uint8_t a=0; a<inicioTx; a++) {
		        cheksum^=buffer[a];
		        data->outBuff[data->txWrite++] =buffer[a];
	        }
	        data->outBuff[data->txWrite++]=cheksum;
            break;
        case 0xA1: //!<  No ACK de la PC, no pudo decodificar el mensaje
        	ackHeader=NOK;
            break;
        case 0x0D: //!< ACK de la PC mensaje recibido y decodificado correctamente
        	ackHeader=OK;
        	break;
        default:

            ;
    }

}

/// **********************************************************************************************************************************
/// ******************************************************* FINALIZA RUTINAS DE RECEPCION Y TRANSMISION USART ************************
/// **********************************************************************************************************************************
uint8_t leerDatos(void){

    if(usart1Com.rxRead != usart1Com.rxWrite){
        ackHeader=NOK;
        decodeHeader(&usart1Com);
    }
    return ackHeader;
}

uint8_t enviarDatos(uint16_t *adcData){
	uint8_t dataReturned=NOK;
    for (uint8_t a=0 ; a<2; a++){
        usart1Com.adcChanel[a]=adcData[a];
    }
    usart1Com.rxdata[1]=0xA0;
    DecodeCommand(&usart1Com);
    if (usart1Com.txRead != usart1Com.txWrite){
    	ackTxCplt=NOK;
        enviar_byte(&usart1Com);
        dataReturned=OK;
    }
    return dataReturned;
}

uint8_t leerAck(void){
	return ackTxCplt;
}
