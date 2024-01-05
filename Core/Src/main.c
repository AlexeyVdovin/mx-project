/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwrb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t tick;
  uint16_t status;
  uint16_t control;
  uint64_t energy;    // in watts/minute
  uint16_t power;     // in 1/10 watt
  uint16_t voltage;   // in 1/10 volt
} REGS_T;

typedef struct
{
  uint8_t   addr;
  float     energy_k;
  float     power_k;
  float     voltage_k;
} CFG_T;

enum
{
  EE_ADDR_BASE  = DATA_EEPROM_BASE,
  EE_I2C_ADDR   = DATA_EEPROM_BASE + 0x04, //  8 bit
  EE_ENERGY_K   = DATA_EEPROM_BASE + 0x08, // 32 bit
  EE_POWER_K    = DATA_EEPROM_BASE + 0x0C, // 16 bit
  EE_VOLTAGE_K  = DATA_EEPROM_BASE + 0x0E  // 16 bit
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_I2C_ADDR  0x6E
#define DEFAULT_ENERGY_K  1
#define DEFAULT_POWER_K   1
#define DEFAULT_VOLTAGE_K 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint64_t tick = 0;

lwrb_t usart_tx_rb;
uint8_t usart_tx_rb_data[128];
volatile size_t usart_tx_dma_current_len;
volatile uint16_t dma_n = 0;

volatile REGS_T reg;
CFG_T           cfg;

volatile uint8_t cn_flag = 0;
volatile uint16_t cn_power = 0;
volatile uint16_t cn_voltage = 0;

uint64_t total_energy;      // in watts/minute
uint16_t current_power;     // in 1/10 watt
uint16_t current_voltage;   // in 1/10 volt

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */
size_t usart_send_string(const char* str);
size_t usart_send_buff(const char* str, size_t len);
uint8_t usart_start_tx_dma_transfer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 0
void unlock_EE()
{
  HAL_FLASHEx_DATAEEPROM_Unlock()
  
  /* (1) Wait till no operation is on going */
  /* (2) Check if the PELOCK is unlocked */
  /* (3) Perform unlock sequence */
  while((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
  {
    /* For robust implementation, add here time-out management */
  }
  if((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */
  {
    FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
    FLASH->PEKEYR = FLASH_PEKEY2;
  }
}

void lock_EE()
{
  /* (1) Wait till no operation is on going */
  /* (2) Locks the NVM by setting PELOCK in PECR */
  while((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
  {
    /* For robust implementation, add here time-out management */
  }
  FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
}
#endif

uint32_t read_EE(uint32_t addr)
{
  // assert_param(IS_FLASH_DATA_ADDRESS(addr));
  return *(uint32_t*)addr;
}

void HAL_UART_TxHalfCpltCallback(void *huart)
{
  // Remove me!
  ++dma_n;
}

void HAL_UART_TxCpltCallback(void *huart)
{
  lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len);/* Skip buffer, it has been successfully sent out */
  usart_tx_dma_current_len = 0;           /* Reset data length */
  usart_start_tx_dma_transfer();          /* Start new transfer */

  // Remove me!
  // ++dma_n;
}

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
    // FIX me !!! HAL_UART_Transmit_DMA(&hlpuart1, ptr, usart_tx_dma_current_len);
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

void delay(int dly)
{
  volatile int c = dly;
  while( c--);
}

void DMA_TransmitData(uint8_t* data, uint8_t size)
{
  LL_LPUART_DisableDMAReq_TX(LPUART1);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
#if 0  
  while (DMA1_Channel2->CCR & 0x01) // While EN bit is high
    __NOP(); // Wait for disable to finish
#endif
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)data, LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, size);
  LL_DMA_ClearFlag_GI2(DMA1);
  LL_DMA_ClearFlag_HT2(DMA1);
  LL_DMA_ClearFlag_TC2(DMA1);
  LL_DMA_ClearFlag_TE2(DMA1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_LPUART_EnableDMAReq_TX(LPUART1);
  LL_LPUART_ClearFlag_TC(LPUART1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint64_t t1 = 1000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if 0
  cfg.addr = read_EE(EE_I2C_ADDR); if(cfg.addr < 0x10 || cfg.addr > 0x7F) cfg.addr = DEFAULT_I2C_ADDR;
  if(cfg.addr != DEFAULT_I2C_ADDR)
  {
    cfg.energy_k  = read_EE(EE_ENERGY_K);
    cfg.power_k   = read_EE(EE_POWER_K);
    cfg.voltage_k = read_EE(EE_VOLTAGE_K);
  }
  else
  {
    cfg.energy_k  = DEFAULT_ENERGY_K;
    cfg.power_k   = DEFAULT_POWER_K;
    cfg.voltage_k = DEFAULT_VOLTAGE_K;
  }
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  // MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  // MX_TIM2_Init();
  // MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_FLASH_Program()
  while (1)
  {
    static const uint32_t dummy = 0xCAFEBABE;
    static const char str[] = "abc\n";

    // delay(100000);
    if(t1 < tick)
    {
      t1 = tick + 500;
      LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      DMA_TransmitData(str, 4);
    }
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_HSI_EnableDivider();
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(4000000);
/* SysTick interrupt Init */
  LL_SYSTICK_EnableIT();

  LL_SetSystemCoreClock(4000000);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00000509;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 192;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**LPUART1 GPIO Configuration
  PA4   ------> LPUART1_TX
  */
  GPIO_InitStruct.Pin = DBG_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(DBG_TX_GPIO_Port, &GPIO_InitStruct);

  /* LPUART1 DMA Init */

  /* LPUART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_5);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
LL_LPUART_Disable(LPUART1);
  LPUART_InitStruct.BaudRate = 115200;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX;
  LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
  LL_LPUART_EnableHalfDuplex(LPUART1);
  LL_LPUART_DisableRTSHWFlowCtrl(LPUART1);
  LL_LPUART_DisableIT_CTS(LPUART1);
    LL_LPUART_DisableIT_ERROR(LPUART1);
  LL_LPUART_DisableCTSHWFlowCtrl(LPUART1);
  LL_LPUART_Enable(LPUART1);
  // LL_LPUART_IsActiveFlag_CTS(LPUART1);
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA0-CK_IN   ------> TIM2_ETR
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_ETRF);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableExternalClock(TIM2);
  LL_TIM_ConfigETR(TIM2, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_DisableIT_TRIG(TIM2);
  LL_TIM_DisableDMAReq_TRIG(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM21 GPIO Configuration
  PA1   ------> TIM21_ETR
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM21, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM21);
  LL_TIM_SetTriggerInput(TIM21, LL_TIM_TS_ETRF);
  LL_TIM_SetClockSource(TIM21, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableExternalClock(TIM21);
  LL_TIM_ConfigETR(TIM21, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_DisableIT_TRIG(TIM21);
  LL_TIM_DisableDMAReq_TRIG(TIM21);
  LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM21);
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(OUT_GPIO_Port, OUT_Pin);

  /**/
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OUT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
