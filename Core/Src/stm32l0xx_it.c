/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint32_t tick;
extern volatile uint32_t i2c_wd;
extern volatile uint8_t cn_flag;
extern volatile uint16_t cn_mul;
extern volatile uint16_t cn_power;
extern volatile uint16_t cn_voltage;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  static uint16_t cnt = 0;
  ++tick;
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  if(++cnt >= 2000)
  {
    cnt = 0;
    cn_power = LL_TIM_GetCounter(TIM21); LL_TIM_SetCounter(TIM21, 0);
    cn_voltage = LL_TIM_GetCounter(TIM2); LL_TIM_SetCounter(TIM2, 0);
    cn_flag = 1;
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
  //LL_GPIO_TogglePin(OUT_GPIO_Port, OUT_Pin);

  if(LL_DMA_IsActiveFlag_TC2(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC2(DMA1);
    LPUART_TxCpltCallback();
  }
  if(LL_DMA_IsActiveFlag_HT2(DMA1) == 1)
  {
    LL_DMA_ClearFlag_HT2(DMA1);
  }
  if(LL_DMA_IsActiveFlag_TE2(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TE2(DMA1);
  }
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  static uint8_t n;
  /* USER CODE BEGIN I2C1_IRQn 0 */
  if(LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
    LL_GPIO_SetOutputPin(OUT_GPIO_Port, OUT_Pin);
    LL_I2C_ClearFlag_ADDR(I2C1);
    i2c_wd = tick + 10;
    // Slave addressed
    if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
    {
      // Slave transmit
      LL_I2C_ClearFlag_TXE(I2C1);
      // LL_I2C_TransmitData8(I2C1, I2C1_Slave_TX());
    }
    else
    {
      // Slave receive
      n = 0;
    }
  }
  if(LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    // STOP condition detected
    LL_I2C_ClearFlag_STOP(I2C1);
    i2c_wd = 0;
    LL_GPIO_ResetOutputPin(OUT_GPIO_Port, OUT_Pin);
  }
  if(LL_I2C_IsActiveFlag_RXNE(I2C1))
  {
    // Data received
    I2C1_Slave_RX(n++, LL_I2C_ReceiveData8(I2C1));
  }
  if(LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    // Write new data to transmit
    LL_I2C_TransmitData8(I2C1, I2C1_Slave_TX());
  }

  /* USER CODE END I2C1_IRQn 0 */

  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
