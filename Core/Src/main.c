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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void vuart_handle(uint16_t Size);
static void uartProcessing (uint8_t *u8p_buffer, uint16_t u16_size);

/* Private user code ---------------------------------------------------------*/

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __GNUC__ */


/******************************************************************
  * @name   PUTCHAR_PROTOTYPE
  * @brief  Retargets the C library printf function to the USART.
  *****************************************************************/
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}


/**********************************************************
 * PARSING HEADER, Used in FW CONFIG - READ/WRITE Process
 **********************************************************/
#define CFG_LENGTH              10
#define CFG_HEADER_NUM          10
#define CFG_HEADER_CHARS_LEN    5           //num of char for header
#define CFG_HEADER_READ         5           //Max index for write, above this index is read command.
#define STRLENMAX               256

static char str_cfg_header[CFG_HEADER_NUM][CFG_HEADER_CHARS_LEN] =
{
    "{MSG:",
    "{CF1:",
    "{CF2:",
    "{CF3:",
    "{CFA:",
    "{RD1}",
    "{RD2}",
    "{RD3}",
    "{RD4}",
    "{RDA}"
};


/* Buffer for Master  */
static int32_t i32_resCF1[CFG_LENGTH] = {10,256,512,37,10,-45,123,46,-78,89};
static int32_t i32_resCF2[CFG_LENGTH] = {20,156,52,-37,20,145,367,46,-12,19};
static int32_t i32_resCF3[CFG_LENGTH] = {35,16,2022,-457,560,15,97,46,12,-67};


static uint8_t u8arr_eventBuff[UART_BUF_SZ];
static uint8_t u8arr_uartEvent[UART_BUF_SZ];
static uint16_t u16_oldPos = 0;
static uint16_t u16_lenCnt = 0;


/* bit flag */
static uint16_t bitFlag;



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  printf("Init OK\r\n");

  /* Infinite loop */



  while (1)
  {
      if (bitFlag & BFLAG_UART_RCV)                                                                 //UART Receiving Process.
      {
          uartProcessing (u8arr_uartEvent, u16_lenCnt - 2); // remove \r & \n
          memset(u8arr_uartEvent, 0, UART_BUF_SZ);
          u16_lenCnt = 0;

          bitFlag   &= ~BFLAG_UART_RCV;
      }
      else if (bitFlag & BFLAG_RD1)                                                                 //Process for Config 1 Value Read.
      {

      }
      else
      {
          HAL_Delay(500);
          HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin);
      }
  }

}





//================================= Peripherals Callback ============================================//

/************************************************************************************
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  ***********************************************************************************/
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{


}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{

}


/*************************************************************************************
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  ************************************************************************************/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{

}


/*************************************************************************************
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note
  * @retval None
  ************************************************************************************/
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{

}



 /*******************************************************************
   * @brief  TxRx Transfer completed callback.
   * @param  hspi: SPI handle
   * @retval None
   *****************************************************************/
 void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
 {

 }

 /*******************************************************************
   * @brief  Tx Transfer completed callback.
   * @param  hspi: SPI handle
   * @retval None
   *****************************************************************/
 void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
 {

 }


 /*******************************************************************
   * @brief  Rx Transfer completed callback.
   * @param  hspi: SPI handle
   * @retval None
   *****************************************************************/
 void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
 {

 }

 /********************************************************************
   * @brief  SPI error callbacks.
   * @param  hspi: SPI handle
   * @retval None
   *******************************************************************/
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
 {

 }


 /*****************************************************************
  * @name HAL_UARTEx_RxEventCallback
  * @brief
  ****************************************************************/
 void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 {
 #ifdef RX_EVENT_CB
     if (huart->Instance == USART3)
     {
         vuart_handle(Size);
         HAL_UARTEx_ReceiveToIdle_DMA(&huart3, u8arr_eventBuff, UART_BUF_SZ);
     }
 #endif

 }


 /************************************************************
   * @brief Button Callback
   * @param GPIO_Pin: Specifies the pins connected EXTI line
   * @retval None
   ***********************************************************/
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {


 }



//================================= End Of Peripherals Callback ===================================//




/*************************************************************************************
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  ************************************************************************************/
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}



/*********************************************************************
 * @name    : tinysh_dec
 * @brief   : string to decimal conversion (up to 15 chars).
 *********************************************************************/
static unsigned long tinysh_dec(char *s)
{
  unsigned long res=0;
  uint8_t index = 0;
  int8_t min    = 1;

  while(*s)
  {
      //printf("%c\r\n",*s);

      res*=10;

      if((*s == '-')&&(index == 0))
          min = -1;
      else if((*s == '0')&&(index == 0))
          res = 0;
      else if(*s>='0' && *s<='9')
          res+=*s-'0';
      else
          break;

      s++;
      index++;

      if(index > 15)
      {
         break;
      }
  }

  return (res * min);
}


/*********************************************************************
 * @name    : updateBufferValue
 * @brief   : Parsing receiving command from PC via UART
 *********************************************************************/
static void vUpdateBufferValue(char *input, char *pChar, char *pChar2, int32_t *pInt32)
{
    uint8_t u8_start    = 0;
    uint8_t u8_stop     = 0;
    uint8_t u8_cnt      = 0;

    char str_res[150];

    while (*pChar)
    {
        if(*pChar == ',')
        {
            memset(&str_res[0], 0, sizeof(str_res));
            memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
            pInt32[u8_cnt] = tinysh_dec(&str_res[0]);
            //printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

            u8_stop = u8_start + 1;
            u8_cnt++;
        }
        else if (*pChar == '}')
        {
            memset(&str_res[0], 0, sizeof(str_res));
            memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
            pInt32[u8_cnt] = tinysh_dec(&str_res[0]);
            //printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

            u8_cnt++;
            break;
        }

        pChar++;
        u8_start++;
    }
}


/*********************************************************************
 * @name    : updateBufferByte
 * @brief   : Parsing receiving command from PC via UART (in byte)
 *********************************************************************/
void vUpdateBufferByte(char *pChar, int32_t *pInt32, uint16_t u16_size)
{
    uint16_t u16_idx = 0;
    uint16_t u16_int = 0;
    uint8_t u8_headerNfooter = u16_size - (CFG_HEADER_CHARS_LEN + 1);
    uint8_t u8_cf_idx = 0;

    while (u16_idx < u8_headerNfooter)
    {
        pInt32[u16_int] = (pChar[u16_idx] << 24) | (pChar[u16_idx+1] << 16) |
                            (pChar[u16_idx+2] << 8) | pChar[u16_idx+3];

        u16_idx += 4;
        u16_int += 1;

        /* Change to CF2 or CF3 buffer */
        if (u16_int >= 10)
        {
            if (u8_cf_idx == 0)
            {
                pInt32  = &i32_resCF2[0];
                u16_int = 0;
            }
            else
            {
                pInt32  = &i32_resCF3[0];
                u16_int = 0;
            }

            u8_cf_idx++;
        }
    }

}


/*********************************************************************
 *  Parsing incoming message                                         *
 *  Example: {MSG:1,23,21009,45,67,-18,25}                           *
 *           {RD1}                                                   *
 *********************************************************************/
static void vShell_cmdParse(char *input, uint16_t u16_size)
{
    uint8_t u8cmd = 0;

    for(uint8_t u8_idx = 0; u8_idx < CFG_HEADER_NUM; u8_idx++)
    {
        if(!memcmp(input,(char*)&str_cfg_header[u8_idx][0], CFG_HEADER_CHARS_LEN))
        {
            char *pChar         = &input[CFG_HEADER_CHARS_LEN];     //for checking each char byte ASCII.
            char *pChar2        = &input[CFG_HEADER_CHARS_LEN];     //for copying start.


            if (u8_idx < CFG_HEADER_READ)
            {
                /* WRITE HEADER */
                switch(u8_idx)
                {
                    case CF1_HEADER:
                        u8cmd = 1;
                        vUpdateBufferValue(input, pChar, pChar2, i32_resCF1);
                        bitFlag |= BFLAG_WR1;
                        break;

                    case CF2_HEADER:
                        u8cmd = 1;
                        vUpdateBufferValue(input, pChar, pChar2, i32_resCF2);
                        bitFlag |= BFLAG_WR2;
                        break;

                    case CF3_HEADER:
                        u8cmd = 1;
                        vUpdateBufferValue(input, pChar, pChar2, i32_resCF3);
                        bitFlag |= BFLAG_WR3;
                        break;

                    case CFA_HEADER:
                        u8cmd = 1;
                        vUpdateBufferByte(pChar, i32_resCF1, u16_size);
                        break;

                    default:
                        break;
                }

                break;
            }
            else
            {
                /* READ HEADER */
                switch(u8_idx)
                {
                    case RD1_HEADER:
                        u8cmd = 1;
                        bitFlag |= BFLAG_RD1;
                        break;

                    case RD2_HEADER:
                        u8cmd = 1;
                        bitFlag |= BFLAG_RD2;
                        break;

                    case RD3_HEADER:
                        u8cmd = 1;
                        bitFlag |= BFLAG_RD3;
                        break;

                    case RDALL_HEADER:
                        u8cmd = 1;
                        bitFlag |= BFLAG_RDA;
                        break;

                    default:
                        break;
                }
            }
        }
    }


    if (u8cmd == 0)
    {
       printf("unknown CMD!!!\r\n");
    }

}




/*****************************************************************
 * @name    vuart_handle
 * @brief   handle afe uart data copy
 *****************************************************************/
static void vuart_handle(uint16_t Size)
{
    uint16_t u16_numData;
    //printf("S(%d): %s\r\n", Size, (char*)u8arr_eventBuff);

    /* Check if number of received data in reception buffer has changed */
    if (Size != u16_oldPos)
    {
        if (Size > u16_oldPos)
        {
            /* Current position is higher than previous one */
            u16_numData = Size - u16_oldPos;
            memcpy(&u8arr_uartEvent[u16_lenCnt],&u8arr_eventBuff[u16_oldPos],u16_numData);
            u16_lenCnt += u16_numData;
        }
        else
        {
            /* End of buffer has been reached */
            u16_numData = UART_BUF_SZ - u16_oldPos;

            memcpy (&u8arr_uartEvent[u16_lenCnt],           // copy data in that remaining space
                    &u8arr_eventBuff[u16_oldPos],
                    u16_numData);

            u16_lenCnt += u16_numData;

            memcpy (&u8arr_uartEvent[u16_lenCnt],           // copy the remaining data
                    &u8arr_eventBuff[0],
                    Size);

            u16_lenCnt += Size;
        }

        /* Check for ready to process */
        if(((u8arr_uartEvent[u16_lenCnt - 1] == '\n')&&(u8arr_uartEvent[u16_lenCnt - 2]== '\r')) ||
           ((u8arr_uartEvent[u16_lenCnt - 1] == '\r')&&(u8arr_uartEvent[u16_lenCnt - 2]== '\n')))
        {
            bitFlag |= BFLAG_UART_RCV;
            //printf("S(%d): %s\r\n", Size, (char*)u8arr_uartEvent);
        }

    }


    u16_oldPos = Size;
}



static void uartProcessing (uint8_t *u8p_buffer, uint16_t u16_size)
{
    //printf("UART RX(%d): %s\r\n", u16_size, (char*)u8p_buffer);
    vShell_cmdParse((char*)u8p_buffer, u16_size);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState        = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM        = 4;
  RCC_OscInitStruct.PLL.PLLN        = 168;
  RCC_OscInitStruct.PLL.PLLP        = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ        = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance                = I2C1;
  hi2c1.Init.ClockSpeed         = 100000;
  hi2c1.Init.DutyCycle          = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1        = 0;
  hi2c1.Init.AddressingMode     = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode    = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2        = 0;
  hi2c1.Init.GeneralCallMode    = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode      = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance            = SPI1;
  hspi1.Init.Mode           = SPI_MODE_MASTER;
  hspi1.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity    = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi1.Init.NSS            = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode         = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial  = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance           = USART3;
  huart3.Init.BaudRate      = 115200;
  huart3.Init.WordLength    = UART_WORDLENGTH_8B;
  huart3.Init.StopBits      = UART_STOPBITS_1;
  huart3.Init.Parity        = UART_PARITY_NONE;
  huart3.Init.Mode          = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling  = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, u8arr_eventBuff, UART_BUF_SZ);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin   = USER_Btn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin   = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin   = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/************************************************************
* @brief  This function is executed in case of error occurrence.
* @retval None
************************************************************/
void Error_Handler(char * file, int line)
{
__disable_irq();
printf("\r\nERROR: %s, line: %d \r\n",file,line);

// Configurable Fault Status Register, Consists of MMSR, BFSR and UFSR
volatile unsigned long  _CFSR = (*((volatile unsigned long *)(0xE000ED28)));
printf("CFSR: %lu \r\n",_CFSR);


// Hard Fault Status Register
volatile unsigned long _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));
printf("HFSR: %lu \r\n",_HFSR);

// Debug Fault Status Register
volatile unsigned long _DFSR = (*((volatile unsigned long *)(0xE000ED30)));
printf("DFSR: %lu \r\n",_DFSR);

// Auxiliary Fault Status Register
volatile unsigned long _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));
printf("AFSR: %lu \r\n",_AFSR);

// Check BFARVALID/MMARVALID to see if they are valid values
// MemManage Fault Address Register
volatile unsigned long _MMAR = (*((volatile unsigned long *)(0xE000ED34)));
printf("MMAR: %lu \r\n",_MMAR);

// Bus Fault Address Register
volatile unsigned long _BFAR = (*((volatile unsigned long *)(0xE000ED38)));
printf("BFAR: %lu \r\n",_BFAR);

//__asm("BKPT #0\n") ; // Break into the debugger

while (1)
{
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      HAL_Delay(100);
}
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
