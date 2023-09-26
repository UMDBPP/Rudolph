/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX126x driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"




static uint8_t IrqNestLevel = 0;

void DelayMs( uint32_t ms )
{
    HAL_Delay( ms );
}

 void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

 void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(	RADIO_DIO0_Pin == GPIO_Pin)
	{
		RadioOnDioIrq( );
	}
}
static uint16_t SpiInOut( SPI_HandleTypeDef *obj, uint16_t outData )
{
    uint8_t rxData = 0;

    if(  obj == NULL )
    {
        assert_param( FAIL );
    }

    __HAL_SPI_ENABLE( obj );

    BoardDisableIrq( );

    HAL_SPI_TransmitReceive( obj, ( uint8_t * ) &outData, ( uint8_t* ) &rxData, 1, 0xFFFF);

    BoardEnableIrq( );

    return( rxData );
}
/*!
 * Antenna switch GPIO pins objects
 */

void SX126xIoInit( void )
{
 MX_GPIO_Init();
}


void SX126xReset( void )
{
    HAL_Delay( 10 );
	  HAL_GPIO_WritePin(GPIOA, RADIO_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay( 20 );
    HAL_GPIO_WritePin(GPIOA, RADIO_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay( 10 );
}

void SX126xWaitOnBusy( void )
{
    while( HAL_GPIO_ReadPin(GPIOB,RADIO_BUSY_Pin) == 1 );
}

void SX126xWakeup( void )
{
    BoardDisableIrq( );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, RADIO_GET_STATUS );
    SpiInOut( SPI_PTR, 0x00 );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    BoardEnableIrq( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SPI_PTR, buffer[i] );
    }

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, ( uint8_t )command );
    SpiInOut( SPI_PTR, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SPI_PTR, 0 );
    }

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);
    
    SpiInOut( SPI_PTR, RADIO_WRITE_REGISTER );
    SpiInOut( SPI_PTR, ( address & 0xFF00 ) >> 8 );
    SpiInOut( SPI_PTR, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SPI_PTR, buffer[i] );
    }

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, RADIO_READ_REGISTER );
    SpiInOut( SPI_PTR, ( address & 0xFF00 ) >> 8 );
    SpiInOut( SPI_PTR, address & 0x00FF );
    SpiInOut( SPI_PTR, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SPI_PTR, 0 );
    }
     HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

     HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, RADIO_WRITE_BUFFER );
    SpiInOut( SPI_PTR, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SPI_PTR, buffer[i] );
    }
     HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

     HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_RESET);

    SpiInOut( SPI_PTR, RADIO_READ_BUFFER );
    SpiInOut( SPI_PTR, offset );
    SpiInOut( SPI_PTR, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SPI_PTR, 0 );
    }
     HAL_GPIO_WritePin(GPIOA, RADIO_NSS_Pin, GPIO_PIN_SET);

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}


bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
