/** 

	@file stm32h7xx_it.h

*/

/* Includes ------------------------------------------------------------------*/
#include "stdhdr.h"
#include "stm32h7xx_it.h"
#include "DEVICE_SYSTEM.h"
#include "FACILITY_SYSTEM.h"



/**
  * @brief  This function handles the NMI exception.
  */
void NMI_Handler( void )
{
}

/**
  * @brief  This function handles the Hard Fault exception.
  */
void HardFault_Handler( void )
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles the Memory Manage exception.
  */
void MemManage_Handler( void )
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles the Bus Fault exception.
  */
void BusFault_Handler( void )
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles the Usage Fault exception.
  */
void UsageFault_Handler( void )
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles the Debug Monitor exception.
  */
void DebugMon_Handler( void )
{
}


/**
  * @brief   handles USART2 interrupt requests
  * @retval  none
  */
void USART2_IRQHandler( void )
{
	SERIAL_IRQHandler(hxGetSerial2());
}


/**
  * @brief   handles USART3 interrupt requests
  * @retval  none
  */
void USART3_IRQHandler( void )
{
	if(LL_USART_IsActiveFlag_RXNE(hxGetSerial3()->USARTx) && LL_USART_IsEnabledIT_RXNE(hxGetSerial3()->USARTx))
	{
		//CMDPROC_Consume(LL_USART_ReceiveData8(hxGetSerial3()->USARTx));
	}
}


/**
  * @brief   handles UART4 interrupt requests
  * @retval  none
  */
void UART4_IRQHandler( void )
{
	SERIAL_IRQHandler(hxGetSerial4());
}


void EXTI15_10_IRQHandler( void )
{
	
	if(LL_EXTI_IsActiveFlag_0_31(hxGetUserBtn()->exti.line) != RESET)
	{
		LL_EXTI_ClearFlag_0_31(hxGetUserBtn()->exti.line);
		TRILED_Yellow(-1);		
	}
}



/**
	@brief  This function handles DMA2 stream0 interrupt request.
	@note	DMA2 is attached to USART2 in this application
*/
void DMA2_Stream0_IRQHandler( void )
{
	if(LL_DMA_IsActiveFlag_TC0(DMA2))
	{
		LL_DMA_ClearFlag_TC0(DMA2);
		//TransferComplete();
	}
	else if(LL_DMA_IsActiveFlag_TE0(DMA2))
	{
		LL_DMA_ClearFlag_TE0(DMA2);
		//TransferError();
	}
}


