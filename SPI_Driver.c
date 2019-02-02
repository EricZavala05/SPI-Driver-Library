/*
* SPI_DRIVER.c
*
*
*	Creatrd on: 1/2/2019
*			Author: Eric Zavala
*/

#include "SPI_Driver.h"

/************************************************************************************************************/
/*																																																					*/
/*															Static Helper Functions																											*/
/*																																																					*/
/************************************************************************************************************/
/**
	* @brief	Enables the SPI device
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_enable(SPI_TypeDef *SPIx)
{
	if(!(SPIx->CR1 & SPI_REG_CR1_SPE))
	SPIx->CR1 |= SPI_REG_CR1_SPE;
}

/**
	* @brief	Disables the SPI device
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 &= ~SPI_REG_CR1_SPE;
	
}

/**
	* @brief	Configures the spi clk phase and polarity
	* @param	*SPIx	: Base Address of the SPI
	*	@param	phase	: configures phase
	*	@param	polarity	: configures polarity 
	*	@retval	None 
**/
static void spi_config_phase_polarity(SPI_TypeDef *SPIx, uint32_t phase, uint32_t polarity)
{
	if(phase)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPHA;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
	}
	
	if(polarity)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPOL;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
	}
	
}

/**
	* @brief	Configures master or slave mode 
	* @param	*SPIx	: Base Address of the SPI
	*	@param	mode	: if 1, then master, otherwise slave
	*	@retval	None 
**/
static void spi_config_device_mode(SPI_TypeDef *SPIx, uint32_t mode)
{
	if(mode)
	{
		SPIx->CR1 |= SPI_REG_CR1_MSTR;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
	}
	
}

/**
	* @brief	Configures the spi datasize
	* @param	*SPIx	: Base Address of the SPI
	*	@param	datasize	: configures data size
	*	@param	firsbit	: if 1, then lsb will be sent first, otherwise msb will be sent first
	*	@retval	None 
**/
static void spi_config_datasize(SPI_TypeDef *SPIx, uint32_t datasize, uint32_t firstbit)
{
	if(datasize)
	{
		SPIx->CR1 |= SPI_REG_CR1_DFF;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_DFF;
	}
	
	if(firstbit)
	{
		SPIx->CR1 |= SPI_CR1_LSBFRST;
	}
	else
	{
		SPIx->CR1 &= ~SPI_CR1_LSBFRST;
	}
	
}

/**
	* @brief	Configures the nss pin of the master
	* @param	*SPIx	: Base Address of the SPI
	*	@param	datasize	: configures data size
	*	@retval	None 
**/
static void spi_config_nss_master(SPI_TypeDef *SPIx, uint32_t ssm)
{
	if(ssm)
	{
		SPIx->CR1 |= SPI_REG_CR1_SSM;
		SPIx->CR1 |= SPI_REG_CR1_SSI;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
	
}

/**
	* @brief	Configures the nss pin of the slave
	* @param	*SPIx	: Base Address of the SPI
	*	@param	datasize	: configures data size
	*	@retval	None 
**/
static void spi_config_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm)
{
	if(ssm)
	{
		SPIx->CR1 |= SPI_REG_CR1_SSM;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
	
}

/**
	* @brief	Enable the Tx buffer empty interrupt (TXE)
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

/**
	* @brief	Disable the Tx buffer empty interrupt (TXE)
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

/**
	* @brief	Enable the Rxn buffer empty interrupt (TXE)
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
	* @brief	Disable the Rxn buffer empty interrupt (TXE)
	* @param	*SPIx	: Base Address of the SPI
	*	@retval	None 
**/
static void spi_disable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
	* @brief	handles TXE interrupt
	* @param	*hspi: pointer to a spi_handle_t structure that cointains
	*									the config info for the spi module
	*	@retval	None 
**/
static void spi_handle_tx_interrupt(spi_handle_t *hspi)
{
	// transmit data in 8 bit mode
	if(hspi->Init.dataSize == SPI_8BIT_DF_ENABLE)
	{
		hspi->Instance->DR = (*hspi->pTxBuffPtr++);
		hspi->TxXferCount--; // we sent 1 byte
	}
	// transmit data in 16 bit mode
	else
	{
		hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr++);
		hspi->pTxBuffPtr+=2; 
		hspi->TxXferCount-=2; // we sent 2 bytes
	}
	if(hspi->TxXferCount == 0)
	{
		// we reached the end of transmission, so close the txe interrupt
		
		// disable txe interrupt 
		spi_disable_txe_interrupt(hspi->Instance);
		
		// if master and if driver state is not SPI_STATE_BUSY_RX then 
		// make state
		if(hspi->Init.Mode && (hspi->State != SPI_STATE_BUSY_RX))
			hspi->State = SPI_STATE_READY;
	}
	
}

/**
	* @brief	handles RXNE interrupt
	* @param	*hspi: pointer to a spi_handle_t structure that cointains
	*									the config info for the spi module
	*	@retval	None 
**/
static void spi_handle_rx_interrupt(spi_handle_t *hspi)
{
	// recieve data in 8 bit mode
	if(hspi->Init.dataSize == SPI_8BIT_DF_ENABLE)
	{
		// NULL check 
		if(hspi->pRxBuffPtr++)
			(*hspi->pRxBuffPtr++) = hspi->Instance->DR;
		hspi->RxXferCount--;
	}
	// receive data in 16 bit mode
	else
	{
		*((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
		hspi->pRxBuffPtr+=2; 
		hspi->RxXferCount-=2; // we sent 2 bytes
	}
	if(hspi->RxXferCount == 0)
	{
		// we are done with the Rxing of data, lets close the rxne interrupt 
		spi_disable_rxne_interrupt(hspi->Instance);
		hspi->State = SPI_STATE_READY;
	}
	
}

/************************************************************************************************************/
/*																																																					*/
/*															Driver exposed APIs																													*/
/*																																																					*/
/************************************************************************************************************/
/**
	* @brief	api used to do initialize the given spi device
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_init(spi_handle_t *spi_handle)
{
	
	
}

/**
	* @brief	api used to do master data transmission
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the tx buffer
	*	@param	length: length of the tx data
	*	@retval	None 
**/
void spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length)
{
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferCount = length;
	spi_handle->TxXferSize = length;
	
	spi_handle->State = SPI_STATE_BUSY_TX;
	
	spi_enable(spi_handle->Instance);
	
	spi_enable_txe_interrupt(spi_handle->Instance);
}

/**
	* @brief	api used to do slave data transmission
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the tx buffer
	*	@param	length: length of the tx data
	*	@retval	None 
**/
void spi_slave_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length)
{
	// populate the pointers and length information to TX the data
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferCount = length;
	spi_handle->TxXferSize = length;
	
	// pointers to handle dummy rx,you can reuse the same pointer
	spi_handle->pRxBuffPtr = buffer;
	spi_handle->RxXferCount = length;
	spi_handle->RxXferSize = length;
	
	// Driver is busy in TX
	spi_handle->State = SPI_STATE_BUSY_TX;
	
	spi_enable(spi_handle->Instance);
	
	// Enable both TXE and RXNE interrupt
	spi_enable_rxne_interrupt(spi_handle->Instance);
	spi_enable_txe_interrupt(spi_handle->Instance);	
}

/**
	* @brief	api used to do master data reception
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length)
{
	uint32_t i = 0;
	uint32_t val;
	// dummy tx
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferCount = length;
	spi_handle->TxXferSize = length;
	
	// data recieved to rx buffer
	spi_handle->pRxBuffPtr = buffer;
	spi_handle->RxXferCount = length;
	spi_handle->RxXferSize = length;
	
	// Driver is busy in RX
	spi_handle->State = SPI_STATE_BUSY_RX;
	
	spi_enable(spi_handle->Instance);
	
	// read data register once before enabling 
	// the RXNE interrupt to make sure DR is empty
	val = spi_handle->Instance->DR;
	
	
	// Enable both TXE and RXNE interrupt
	spi_enable_rxne_interrupt(spi_handle->Instance);
	spi_enable_txe_interrupt(spi_handle->Instance);
	
}

/**
	* @brief	api used to do slave data reception
	* @param	*SPIx	: Base Address of the SPI
	*	@param	*buffer: pointer to the rx buffer
	*	@param	length: length of the rx data
	*	@retval	None 
**/
void spi_slave_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t length)
{
	// populate the rcv buffer pointer address along with size in the handle
	spi_handle->pRxBuffPtr = buffer;
	spi_handle->RxXferCount = length;
	spi_handle->RxXferSize = length;
	
	// Driver is busy in RX 
	spi_handle->State = SPI_STATE_BUSY_RX;
	
	// enable the peripheral if it is not enabled
	spi_enable(spi_handle->Instance);
	
	// slave need to rcv date, sp enable the rxne interrupt
	// byte reception will be taken care in the rxne interrupt handling code
	spi_enable_txe_interrupt(spi_handle->Instance);
	
}

/**
	* @brief	this function handles spi interrupt request
	* @param	*hspi: pointer to a spi_handle_t structure that cointains
	*									the config info for the spi module
	*	@retval	None 
**/
void spi_irq_handler(spi_handle_t *hspi)
{
	uint32_t tmp1 = 0, tmp2 = 0;
	
	// check to see RXNE is set in the status register
	tmp1 = (hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);
	// check to see RXNEIE bit is enabled in the control register
	tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_RXNEIE_ENABLE);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		// RXNE flag is set, handle the RX of data bytes
		spi_handle_rx_interrupt(hspi);
		return;
	}		
	
	// check to see TXE is set in the status register
	tmp1 = (hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);
	// check to see RXNEIE bit is enabled in the control register
	tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_TXEIE_ENABLE);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		// RXNE flag is set, handle the RX of data bytes
		spi_handle_tx_interrupt(hspi);
		return;
	}	
	
}
