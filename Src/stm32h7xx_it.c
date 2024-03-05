/** 

	@file stm32h7xx_it.h

*/

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"


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
}


/**
  * @brief   handles USART3 interrupt requests
  * @retval  none
  */
void USART3_IRQHandler( void )
{
}


/**
  * @brief   handles UART4 interrupt requests
  * @retval  none
  */
void UART4_IRQHandler( void )
{
}


void EXTI15_10_IRQHandler( void )
{
}



/**
	@brief  This function handles DMA2 stream0 interrupt request.
	@note	DMA2 is attached to USART2 in this application
*/
void DMA2_Stream0_IRQHandler( void )
{
}


