/*
* SPI_DRIVER.h
*
*
*	Creatrd on: 1/2/2019
*			Author: Eric Zavala
*/

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f446xx.h"


/************************************************************************************************************/
/*																																																					*/
/*															1. Macros for SPI Initialization 																			*/
/*																																																					*/
/************************************************************************************************************/


/*************************** Definitions for SPI_CR1 register***/
#define SPI_REG_CR1_BIDIMODE										((unit32_t) 1 << 15)
#define SPI_ENABLE_2_LINE_UNI_DIR								0
#define SPI_ENABLE_1_LINE_BIDI									1

#define SPI_REG_CR1_DFF													((uint32_t) 1 << 11)
#define SPI_8BIT_DF_ENABLE											0
#define SPI_16_BIT_DF_ENABLE 										1


#define SPI_REG_CR1_SSM													((uint32_t) 1 << 9)
#define SPI_REG_CR1_SSI													((uint32_t) 1 << 8)


#define SPI_CR1_LSBFRST 												((uint32_t) 1 << 7)
#define SPI_TX_MSB_FIRST												0
#define SPI_TX_LSB_FIRST												1

#define SPI_REG_CR1_SPE													((uint32_t) 1 << 6)

#define SPI_REG_CR1_BR_PCLK_DIV_2								((uint32_t) 0 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_4								((uint32_t) 1 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_8								((uint32_t) 2 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_16							((uint32_t) 3 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_32							((uint32_t) 4 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_64							((uint32_t) 5 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_128							((uint32_t) 6 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_256							((uint32_t) 7 << 3)

#define SPI_REG_CR1_MSTR												((uint32_t) 1 << 2)
#define SPI_SLAVE_MODE_SEL											0
#define SPI_MASTER_MODE_SEL											1

#define SPI_REG_CR1_CPOL												((uint32_t) 1 << 1)
#define SPI_CPOL_LOW														0
#define SPI_CPOL_HIGH														1

#define SPI_REG_CR1_CPHA												((uint32_t) 1 << 0)
#define	SPI_FIRST_CLOCK_TRANS										0
#define SPI_SECOND_CLOCK_TRANS									1

/*************************** Definitions for SPI_CR2 register***/
#define SPI_REG_CR2_TXEIE_ENABLE 								((uint32_t) 1 << 7)
#define SPI_REG_CR2_RXNEIE_ENABLE								((uint32_t) 1 << 6)
#define SPI_REG_CR2_ERRIE_ENABLE								((uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT								((uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE												0
#define SPI_TI_MODE															1

#define SPI_REG_CR2_SSOE												((uint32_t) 1 << 2)


/*************************** Definitions for SPI_SR register***/
#define SPI_REG_SR_FRE_FLAG											((uint32_t) 1 << 8)
#define SPI_REG_SR_BUSY_FLAG										((uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG											((uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG										((uint32_t) 1 << 0)

/* Redefine base address for SPI*/
#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3

#define SPI_IS_BUSY 1
#define SPI_IS_NOT_BUSY 0

/* Macros to enable clock for different spi devices */
#define RCC_SPI1_CLK_ENABLE()						(RCC->APB2ENR |= (1 << 12))
#define RCC_SPI2_CLK_ENABLE()						(RCC->APB1ENR |= (1 << 14))
#define RCC_SPI3_CLK_ENABLE()						(RCC->APB1ENR |= (1 << 15))



#define RESET 0
#define SET 1

/************************************************************************************************************/
/*																																																					*/
/*															2. Data Dtructure for SPI Initialization 															*/
/*																																																					*/
/************************************************************************************************************/


/** 
  * @brief SPI state structure definition 
  */
typedef enum{
	SPI_STATE_RESET				= 0x00,  // SPI not yet initialized or disable 
	SPI_STATE_READY				= 0x01,  // SPI initialized and ready for use
	SPI_STATE_BUSY				= 0x02,  // SPI process is ongoing
	SPI_STATE_ERROR				= 0x03,	 // SPI error state
	SPI_STATE_BUSY_TX 		= 0x12,	 // Data transmission process is ongoing
	SPI_STATE_BUSY_RX			= 0x22,  // Data reception process is ongoing
	SPI_STATE_BUSY_TX_RX	= 0x32  // Data transmission and reception is ongoing
}spi_state_t;

/** 
  * @brief SPI configuration structure definition 
  */
typedef struct{
	uint32_t Mode;						  // specifies spi operating mode
	uint32_t direction; 				// specifies the spi directional mode state
	uint32_t dataSize;					// specifies the spi data size 
	uint32_t CLKPolarity;				// specifies the serial clock steady state
	uint32_t CLKPhase;					// specifies the clock active edge for the bit capture
	uint32_t NSS;								// Specifies whether the nss signal is managed by 
															// hardware (nss pin) or by software using the ssi bit
	uint32_t BaudRatePrescaler; // specifies the baud rate prescaler value which will be used to configure
															// the transmit and recieve sck clock
	uint32_t FirstBit;					// specifies whether data transfers start from msb or lsb bit
}spi_init_t;

typedef struct{
	SPI_TypeDef *Instance;	// spi registers base address
	spi_init_t Init;				// spi communication parameters
	uint8_t *pTxBuffPtr;		// pointer to spi tx trnafer buffer
	uint16_t TxXferSize;		// spi tx transfer size 
	uint16_t TxXferCount;		// spi tx tranfer counter
	uint8_t *pRxBuffPtr;		// pointer to spi rx transfer buffer
	uint16_t RxXferSize;		// spi rx transfer size
	uint16_t RxXferCount;		// spi rx tranfer counter
	spi_state_t State;			// spi communication state
}spi_handle_t;

/************************************************************************************************************/
/*																																																					*/
/*															3. Driver Exposed APIs																											*/
/*																																																					*/
/************************************************************************************************************/

/**
	* @brief	api used to do initialize the given spi device
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_init(spi_handle_t *spi_handle);

/**
	* @brief	api used to do master data transmission
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the tx buffer
	*	@param	length: length of the tx data
	*	@retval	None 
**/
void spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length);

/**
	* @brief	api used to do slave data transmission
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the tx buffer
	*	@param	length: length of the tx data
	*	@retval	None 
**/
void spi_slave_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length);

/**
	* @brief	api used to do master data reception
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length);

/**
	* @brief	api used to do slave data reception
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_slave_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length);

/**
	* @brief	this function handles spi interrupt request
	* @param	*hspi: pointer to a spi_handle_t structure that cointains
	*									the config info for the spi module
	*	@retval	None 
**/
void spi_irq_handler(spi_handle_t *hspi);

#endif 
