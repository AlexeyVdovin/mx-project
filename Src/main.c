/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdio_impl.h"
#include "lwrb.h"
#include "proto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint32_t adcBuffer[2];
volatile uint32_t dma_lum = 0;
volatile uint16_t dma_n = 0;

lwrb_t usart_tx_rb;
uint8_t usart_tx_rb_data[128];
volatile size_t usart_tx_dma_current_len;

lwrb_t usart_rx_rb;
uint8_t usart_rx_rb_data[128];

uint8_t usart_rx_dma_buffer[32];

static uint32_t regs[0x40];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
size_t usart_send_buff(const char* str, size_t len);
int usart_rx_check(void);
uint8_t usart_start_tx_dma_transfer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t* adc = (uint16_t*)adcBuffer;

  ++dma_n;
  dma_lum += adc[0];

//  __HAL_DMA_DISABLE(&hdma_adc);
//  __HAL_DMA_SET_COUNTER(&hdma_adc, 3);

}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  // Remove me!
  ++dma_n;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len);/* Skip buffer, it has been successfully sent out */
  usart_tx_dma_current_len = 0;           /* Reset data length */
  usart_start_tx_dma_transfer();          /* Start new transfer */

  // Remove me!
  // ++dma_n;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  // Check if it is an "Idle Interrupt"
  {									
//    size_t   len;
//    uint8_t* ptr;

    __HAL_UART_CLEAR_IT(huart, USART_ICR_IDLECF);
    usart_rx_check();
#if 0
    // RX completed, we can start TX
    if(usart_tx_dma_current_len == 0)
    {
      do
      {
        len = lwrb_get_linear_block_read_length(&usart_rx_rb);
        ptr = lwrb_get_linear_block_read_address(&usart_rx_rb);
        lwrb_write(&usart_tx_rb, ptr, len);        /* Write data to TX buffer for loopback */
        lwrb_skip(&usart_rx_rb, len);              /* Skip buffer, it has been successfully sent out */
      } while(len);
      usart_start_tx_dma_transfer();             /* Then try to start transfer */
    }
#endif    
  }
  else
  {
    usart_rx_check();

  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  usart_rx_check();
  // Remove me!
  // ++dma_n;
}

/*
// On RX timeout function called with number of received bytes.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  usart_rx_check();
  // Remove me!
  ++dma_n;
  // huart->RxState;
//  usart_send_buff((const char*)usart_rx_dma_buffer, Size);
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
}
*/

uint8_t usart_start_tx_dma_transfer(void) 
{
  uint8_t started = 0;
  uint8_t* ptr;

  /*
    * First check if transfer is currently in-active,
    * by examining the value of usart_tx_dma_current_len variable.
    *
    * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
    *
    * It is not necessary to disable the interrupts before checking the variable:
    *
    * When usart_tx_dma_current_len == 0
    *    - This function is called by either application or TX DMA interrupt
    *    - When called from interrupt, it was just reset before the call,
    *         indicating transfer just completed and ready for more
    *    - When called from an application, transfer was previously already in-active
    *         and immediate call from interrupt cannot happen at this moment
    *
    * When usart_tx_dma_current_len != 0
    *    - This function is called only by an application.
    *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
    *
    * Disabling interrupts before checking for next transfer is advised
    * only if multiple operating system threads can access to this function w/o
    * exclusive access protection (mutex) configured,
    * or if application calls this function from multiple interrupts.
    *
    * This example assumes worst use case scenario,
    * hence interrupts are disabled prior every check
    */
  if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) 
  {
    ptr = lwrb_get_linear_block_read_address(&usart_tx_rb);
    HAL_UART_Transmit_DMA(&huart2, ptr, usart_tx_dma_current_len);
    started = 1;
  }

  return started;
}

size_t usart_send_string(const char* str) 
{
    size_t rc = lwrb_write(&usart_tx_rb, str, strlen(str)); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();              /* Then try to start transfer */
    return rc;
}

size_t usart_send_buff(const char* str, size_t len) 
{
    size_t rc = lwrb_write(&usart_tx_rb, str, len); /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();                   /* Then try to start transfer */
    return rc;
}

void usart_process_data(const uint8_t* data, size_t len) 
{
  // Process incoming data on the fly or copy to somewhere for future processing...
  size_t rc = lwrb_write(&usart_rx_rb, data, len);
  if(rc != len) printf("usart_process_data: RX overflow!");
}

int usart_rx_check(void) 
{
  static size_t old_pos = 0;
  size_t pos, len = 0;

  /* Calculate current position in buffer and check for new data available */
  pos = ARRAY_LEN(usart_rx_dma_buffer) - huart2.hdmarx->Instance->CNDTR;          // LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
  if (pos != old_pos)                       /* Check change in received data */
  {
    if (pos > old_pos)                     /* Current position is over previous one */
    {
      /*
        * Processing is done in "linear" mode.
        *
        * Application processing is fast with single data block,
        * length is simply calculated by subtracting pointers
        *
        * [   0   ]
        * [   1   ] <- old_pos |------------------------------------|
        * [   2   ]            |                                    |
        * [   3   ]            | Single block (len = pos - old_pos) |
        * [   4   ]            |                                    |
        * [   5   ]            |------------------------------------|
        * [   6   ] <- pos
        * [   7   ]
        * [ N - 1 ]
        */
      len = pos - old_pos;
      usart_process_data(&usart_rx_dma_buffer[old_pos], len);
    }
    else
    {
      /*
        * Processing is done in "overflow" mode..
        *
        * Application must process data twice,
        * since there are 2 linear memory blocks to handle
        *
        * [   0   ]            |---------------------------------|
        * [   1   ]            | Second block (len = pos)        |
        * [   2   ]            |---------------------------------|
        * [   3   ] <- pos
        * [   4   ] <- old_pos |---------------------------------|
        * [   5   ]            |                                 |
        * [   6   ]            | First block (len = N - old_pos) |
        * [   7   ]            |                                 |
        * [ N - 1 ]            |---------------------------------|
        */
      len = ARRAY_LEN(usart_rx_dma_buffer) - old_pos;
      usart_process_data(&usart_rx_dma_buffer[old_pos], len);
      if (pos > 0) 
      {
        len += pos;
        usart_process_data(&usart_rx_dma_buffer[0], pos);
      }
    }
    old_pos = pos;                          /* Save current position as old for next transfers */
  }
  return len;
}

void process_pkt(packet_t* pkt, int id)
{
	uint8_t tx[sizeof(packet_t)+MAX_DATA_LEN+2] = {
			SOP_485_CODE0, SOP_485_CODE1, SOP_485_CODE2, SOP_485_CODE3,
			pkt->src, (uint8_t)(id&0xFF)/*from*/, 1, 0, 0 };
	packet_t* p = NULL;
	uint8_t i, len = 0;
	uint16_t addr = 0;
	
	// printf("process_pkt()\n");
	
	switch(pkt->cmd)
	{
		case CMD_485_PING:
			p = (packet_t*)tx;
			p->len = 0;
			p->cmd = 0x80 | CMD_485_PING;
			break;
		case CMD_485_REG_READ8:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr < sizeof(regs))
			{
				p = (packet_t*)tx;
				p->len = 1;
				p->cmd = 0x80 | CMD_485_REG_READ8;
				p->data[0] = regs[addr];
			}
			break;
		}
		case CMD_485_REG_READ16:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr+1 < sizeof(regs))
			{
				p = (packet_t*)tx;
				p->len = 2;
				p->cmd = 0x80 | CMD_485_REG_READ16;
				p->data[0] = regs[addr];
				p->data[1] = regs[addr+1];
			}
			break;
		}
		case CMD_485_REG_READ32:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr+3 < sizeof(regs))
			{
				p = (packet_t*)tx;
				p->len = 4;
				p->cmd = 0x80 | CMD_485_REG_READ16;
				p->data[0] = regs[addr];
				p->data[1] = regs[addr+1];
				p->data[2] = regs[addr+2];
				p->data[3] = regs[addr+3];
			}
			break;
		}
		case CMD_485_DATA_READ:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			len = pkt->data[2];
			if(len && addr+len-1 < sizeof(regs))
			{
				p = (packet_t*)tx;
				p->len = len;
				p->cmd = 0x80 | CMD_485_DATA_READ;
				for(i = 0; i < len; ++i) p->data[i] = regs[addr+i];
			}
			break;
		}
		case CMD_485_REG_WRITE8:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr < sizeof(regs))
			{
				regs[addr] = pkt->data[2];
				p = (packet_t*)tx;
				p->len = 0;
				p->cmd = 0x80 | CMD_485_REG_WRITE8;
			}
			break;
		}
		case CMD_485_REG_WRITE16:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr+1 < sizeof(regs))
			{
				regs[addr] = pkt->data[2];
				regs[addr+1] = pkt->data[3];
				p = (packet_t*)tx;
				p->len = 0;
				p->cmd = 0x80 | CMD_485_REG_WRITE16;
			}
			break;
		}
		case CMD_485_REG_WRITE32:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			if(addr+3 < sizeof(regs))
			{
				regs[addr] = pkt->data[2];
				regs[addr+1] = pkt->data[3];
				regs[addr+2] = pkt->data[4];
				regs[addr+3] = pkt->data[5];
				p = (packet_t*)tx;
				p->len = 0;
				p->cmd = 0x80 | CMD_485_REG_WRITE32;
			}
			break;
		}
		case CMD_485_DATA_WRITE:
		{
			addr = pkt->data[0] | (pkt->data[1]<<8);
			len = pkt->data[2];
			if(len && addr+len-1 < sizeof(regs))
			{
				for(i=0; i < len; ++i) regs[addr+i] = pkt->data[3+i];
				p = (packet_t*)tx;
				p->len = 0;
				p->cmd = 0x80 | CMD_485_DATA_WRITE;
			}
			break;
		}
			
		default:
			break;
	}
	if(p) proto_tx(&usart_tx_rb, p);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t t, t1;
  // uint16_t *adc = (uint16_t*)adcBuffer;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);

  /* Initialize ringbuff */
  lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));
  lwrb_init(&usart_rx_rb, usart_rx_rb_data, sizeof(usart_rx_rb_data));

  proto_init(0, &usart_rx_rb);

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuffer, 3);

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//  __HAL_UART_CLEAR_IT(&huart2, USART_ICR_IDLECF);
  HAL_UART_Receive_DMA(&huart2, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
//  __HAL_UART_CLEAR_IT(&huart2, USART_ICR_IDLECF);

  // __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));

  printf("Hello World!\n");

  // usart_send_string("String from STM32 DMA\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  t1 = HAL_GetTick() + 1000;
  while (1)
  {
    packet_t* pkt;

    pkt = proto_poll(0);
    if(pkt)
    {
      process_pkt(pkt, 2);
      ++dma_n;
    }
    usart_start_tx_dma_transfer();

    t = HAL_GetTick();
    if(t1 < t)
    {
      t1 = t+100;

      // regs.adc_lum = dma_lum /dma_n;
      // dma_lum = 0;
      // dma_n = 0;



    }
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  // TODO: Set RX timeout ???

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

