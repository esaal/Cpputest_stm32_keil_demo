
/**
  ******************************************************************************
  * @file           : mainH7.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "mainH7.h"
#include "stm32h7xx_hal.h"
#include "stdhdr.h"

#include "CommandLineTestRunner.h"


void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void mainH7(void)
{
  HAL_Init();

  /* Configure the system clock */
  //SystemClock_Config();
    
  //turn on verbose mode
  const char * av_override[] = { "exe", "-v" };
		
  CommandLineTestRunner::RunAllTests(2, av_override);
		
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /* Configure FLASH Latency*/
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_6);
    
  /* Wait till FLASH Latency ready */
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_6)
  {
  }
  
  /* Configure the main internal Regulator output voltage */
  //LL_PWR_ConfigSupply(LL_PWR_DIRECT_SMPS_SUPPLY);               // PWR->CR3 SMPS Enable
  MODIFY_REG(PWR->CR3, (PWR_CR3_SMPSLEVEL | PWR_CR3_SMPSEXTHP | PWR_CR3_SMPSEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS), LL_PWR_DIRECT_SMPS_SUPPLY);
  //LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);    // Regulator Voltage Scaling (PWR_D3CR_VOS_0 | PWR_D3CR_VOS_1)
  MODIFY_REG(PWR->SRDCR, PWR_SRDCR_VOS, LL_PWR_REGU_VOLTAGE_SCALE0);
  /* Wait till the Regulator is ready */
  //while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
    __IO uint32_t tmpreg = 0x00;  
    do
    {
      tmpreg = ((READ_BIT(PWR->SRDCR, PWR_SRDCR_VOS) == (PWR_SRDCR_VOS)) ? 1UL : 0UL);
    }while (tmpreg == 0);
  }

  /* Enable HSI oscillator */
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);       /* RCC_HSI_DIV1 turn ON the HSI oscillator and divide it by 1 (default after reset) */

  /* Wait till HSI is ready */
  //while(LL_RCC_HSI_IsReady() != 1)
  {
    /* Get Start Tick*/
    uint32_t tickstart;
    tickstart = HAL_GetTick();
      
    while((READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY)) == 0U)
    {
      if((uint32_t) (HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
      {
        break;
      }
    }
  }
  
  LL_RCC_HSI_SetCalibTrimming(64);              /* Adjusts the Internal High Speed oscillator (HSI) calibration value */  

  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);   /* HSI source clock selected ((uint32_t)0x00000000) */
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(35);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(4);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  //while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  //while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(280000000);
  //LL_SetSystemCoreClock(280000000);

  /* Set systick to 1ms */
  /* SysTick_IRQn interrupt priority */
  SysTick_Config(280000000 / 1000);

  SystemCoreClock = 280000000;

  #if 0
   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  #endif
}
