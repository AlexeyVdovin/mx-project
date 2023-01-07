/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include "stdio_impl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum
{
  REG_ST_STATUS = 0x0007,
  REG_ST_1W_PWR_OK = 0x0010
};

enum
{
  REG_CTL_LOW_PWR   = 0x0001,
  REG_CTL_DR1       = 0x0002,
  REG_CTL_DR2       = 0x0004,
  REG_CTL_ENABLE_PM = 0x8000
};

typedef struct
{
  int64_t tick;       // 0x00 RO
  int16_t status;     // 0x08 RO
  int16_t control;    // 0x0a RW
  int16_t res_1;      // 0x0c RO
  int16_t res_2;      // 0x0e RO
  int16_t adc_12v;    // 0x10 RO
  int16_t adc_batt;   // 0x12 RO
  int16_t adc_z5v;    // 0x14 RO
  int16_t adc_z3v3;   // 0x16 RO
  int16_t adc_ref;    // 0x18 RO
  int16_t adc_temp;   // 0x1a RO
} REGS;

typedef struct
{
	int16_t	k; // k in 1/8192
	int16_t c; // value = k * RAW + c
} kv_t;

typedef union 
{
  uint32_t dummy;
  struct
  {
    uint16_t i2c_id;
    uint16_t node_id;
    kv_t kv_12v;
    kv_t kv_batt;
    kv_t kv_z5v;
    kv_t kv_z3v3;
    kv_t kv_ref;
    kv_t kv_temp;
  };
} CONF;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SLAVE_ADDR  0x30

#define max(a,b) ((a)>(b)?(a):(b))
#define BIT_SET(var, bit, value) (((var) & (~(bit))) | (((value) ? (bit) : 0)))


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t adcBuffer[3];
volatile uint32_t adc_sum[6] = {0};
volatile uint32_t dma_n = 0;

volatile REGS regs;
volatile CONF conf;

volatile uint64_t wdt_i2c;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Private define Sequential Transfer Options default/reset value */
#define I2C_NO_OPTION_FRAME     (0xFFFF0000U)

#define LED_ON  GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET
#define LED_TOGGLE (GPIO_PIN_SET+1)

#define PWR_OFF GPIO_PIN_RESET
#define PWR_ON  GPIO_PIN_SET

void LED_Red(GPIO_PinState PinState)
{
  if(PinState > GPIO_PIN_SET) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, PinState);
}

void LED_Green(GPIO_PinState PinState)
{
  if(PinState > GPIO_PIN_SET) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, PinState);
}

void PWR_Enable(GPIO_PinState PinState)
{
  if(regs.control & REG_CTL_ENABLE_PM) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, PinState);
}

void DR1_Start(int dir)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void DR1_Stop()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void DR2_Start(int dir)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void DR2_Stop()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

GPIO_PinState PWR_1w_status()
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  ++regs.tick;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t* adc = (uint16_t*)adcBuffer;

  ++dma_n;
  adc_sum[0] += adc[0];
  adc_sum[1] += adc[1];
  adc_sum[2] += adc[2];
  adc_sum[3] += adc[3];
}

uint8_t reg_addr = 0;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  HAL_StatusTypeDef rc;
  uint8_t* r = (uint8_t*)&regs;
  uint8_t  a = reg_addr;
  wdt_i2c = regs.tick + 30000;

  if(TransferDirection != I2C_DIRECTION_TRANSMIT) // Slave Tx
  {
    rc = HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, r+a, sizeof(regs)-a, I2C_NEXT_FRAME);
  }
  else   // Slave Rx
  {
    // First byte is a REG address
    rc = HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, &reg_addr, sizeof(reg_addr), I2C_NEXT_FRAME);
  }
  if(rc != HAL_OK)
  {
    Error_Handler();
  }
}

// i2c STOP handler
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  uint8_t  sz = sizeof(regs)-reg_addr - hi2c->XferSize-1;

  reg_addr += sz;

  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}
*/

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  uint8_t* r = (uint8_t*)&regs;
  uint8_t  a = reg_addr;

  if(HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, r+a, sizeof(regs)-a, I2C_NEXT_FRAME) != HAL_OK)
  {
    Error_Handler();
  }
}

enum
{
  ST_IDLE = 0,
  ST_START,
  ST_SHDN,
  ST_LOW_POWER
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint64_t t1, t2, tdr1 = 0, tdr2 = 0;
  uint16_t ctl = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  regs.tick = 0;

  conf.i2c_id = I2C_SLAVE_ADDR;
  conf.node_id = 0x3FFF;
  conf.kv_12v.k = 16582;
  conf.kv_12v.c = 0;
  conf.kv_batt.k = 16582;
  conf.kv_batt.c = 0;
  conf.kv_z5v.k = 13130;
  conf.kv_z5v.c = 0;
  conf.kv_z3v3.k = 14644;
  conf.kv_z3v3.c = 0;
  conf.kv_ref.k = 8192;
  conf.kv_ref.c = 0;
  conf.kv_temp.k = 8192; // TODO: Set correct default Temperature conversion constants
  conf.kv_temp.c = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  memset((void*)&regs, 0, sizeof(regs)); 

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuffer, 6);

  LED_Green(LED_ON);

  printf("Hello !\n");  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  t1 = regs.tick + 100;
  t2 = regs.tick + 200;
  wdt_i2c = regs.tick + 30000; // 5min

  tdr1 = regs.tick + 2000;
  DR1_Start(0);
  tdr2 = regs.tick + 2000;
  DR2_Start(0);
  ctl = regs.control;

  while (1)
  {
    if(t1 < regs.tick)
    {
      uint16_t c;
      t1 = regs.tick+10;

      regs.status = BIT_SET(regs.status, REG_ST_1W_PWR_OK, PWR_1w_status());
      regs.adc_12v = conf.kv_12v.k*(adc_sum[0]/dma_n)/4096 + conf.kv_12v.c;
      regs.adc_batt = conf.kv_batt.k*(adc_sum[1]/dma_n)/4096 + conf.kv_batt.c;
      regs.adc_z5v = conf.kv_z5v.k*(adc_sum[2]/dma_n)/8192 + conf.kv_z5v.c;
      regs.adc_z3v3 = conf.kv_z3v3.k*(adc_sum[3]/dma_n)/8192 + conf.kv_z3v3.c;

      dma_n = 0;
      adc_sum[0] = 0;
      adc_sum[1] = 0;
      adc_sum[2] = 0;
      adc_sum[3] = 0;

      c = regs.control ^ ctl;
      if(c && (c & REG_CTL_DR1))
      {
        tdr1 = regs.tick + 2000;
        DR1_Start(regs.control & REG_CTL_DR1);
      }
      if(c && (c & REG_CTL_DR2))
      {
        tdr2 = regs.tick + 2000;
        DR2_Start(regs.control & REG_CTL_DR2);
      }

      ctl = regs.control;
      if(tdr1 && tdr1 < regs.tick) { tdr1 = 0; DR1_Stop(); }
      if(tdr2 && tdr2 < regs.tick) { tdr2 = 0; DR2_Stop(); }
    }
    if(t2 < regs.tick)
    {
      int status = regs.status & REG_ST_STATUS;
      t2 = regs.tick+50;

      if(regs.adc_z3v3 < 2900 && status == ST_IDLE)
      { // Self power OFF, waiting for power cycle
        wdt_i2c = regs.tick + 1000; // 10sec
        if(regs.control & REG_CTL_LOW_PWR) status = ST_LOW_POWER;
        else status = ST_SHDN;
        LED_Green(LED_OFF);
        PWR_Enable(PWR_OFF);
      }
      if(regs.adc_batt < 10500 && regs.adc_12v < 11500 && status == ST_IDLE)
      { // Emergency low power, forcing power OFF
        wdt_i2c = regs.tick + 3000; // 30sec
        status = ST_LOW_POWER;
        LED_Green(LED_OFF);
        PWR_Enable(PWR_OFF);
      }
      else if((regs.adc_batt > 12000 || regs.adc_12v > 12500) && status == ST_LOW_POWER)
      { // Delay before Power ON to stabilize power
        wdt_i2c = regs.tick + 1000; // 10sec
        status = ST_SHDN;
      }

      // Print power rails voltage in mV
      // printf("%d\t%d\t%d\t%d\n", regs.adc_12v, regs.adc_batt, regs.adc_z5v, regs.adc_z3v3);

      LED_Red(LED_TOGGLE);
      regs.status = (regs.status & (~REG_ST_STATUS)) | (status & REG_ST_STATUS);
    }
    if(wdt_i2c != 0 && wdt_i2c < regs.tick)
    {
      int status = regs.status & 0x0007;
      wdt_i2c = regs.tick + 30000; // 5min
      if(status == ST_START)
      { // OPi is starting, give it more time
        status = ST_IDLE;
      }
      else if(status == ST_IDLE)
      { // No activity from OPi, force power cycle
        wdt_i2c = regs.tick + 1000; // 10sec
        printf("WDT reboot!\n");
        status = ST_SHDN;
        LED_Green(LED_OFF);
        PWR_Enable(PWR_OFF);
      }
      else if(status == ST_SHDN)
      { // Time to power ON
        printf("Power On!\n");
        status = ST_START;
        regs.control &= ~(REG_CTL_LOW_PWR|REG_CTL_ENABLE_PM);
        LED_Green(LED_ON);
        PWR_Enable(PWR_ON);
      }
      regs.status = (regs.status & (~REG_ST_STATUS)) | (status & REG_ST_STATUS);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __WFI(); 
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 96;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  // hi2c1.XferISR = I2C_Slave_ISR_IT;

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 60000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB3 PB4
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

