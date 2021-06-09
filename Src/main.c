/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "strbuf.h"
#include <string.h>
#include "queue.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define TX_SIZE 256
#define RX_SIZE 256

//CharDeque uart2txdeque, uart2rxdeque;
//typedef queue <unsigned char, 256> CharDeque;

/*
typedef FIFO <unsigned char> charFIFO;

template <int Size>
using CharDeque = queue<charFIFO::value_type, Size>;
CharDeque <256> uart2txdeque, uart2rxdeque;

queue <CanTxMsgTypeDef, 16> canfifo;
*/

typedef queue <uint8_t> charFIFO;
charFIFO uart2txdeque(TX_SIZE), uart2rxdeque(RX_SIZE);

bool rx2fail = false;

queue <CanTxMsgTypeDef> cantxfifo (16);
queue <CanRxMsgTypeDef> canrxfifo (16);

volatile char uart2txready = 1;
volatile char cantxready = 1;
uint8_t txbuf[TX_SIZE] = {0};
volatile uint8_t uart2rxbyte, uart2strreceived = 0;

char digit(uint8_t data)
{
  if (data < 10)
   return data + '0';
  return data - 10 + 'A';
}

unsigned char* printdigits(unsigned char *p, uint32_t data, char digits, char radix)
{
  char str[32];
  signed char ind = 0;
  if (radix < 2) return 0;
  if (radix > 36) return 0;
  if (digits > 32) digits = 32;
  if (digits)
  {
   do
   {
    char dig = data % radix;
    str[ind++] = digit(dig);
    data /= radix;
    digits--;
   } while (digits);
  } else
  {
   do
   {
    char dig = data % radix;
    str[ind++] = digit(dig);
    data /= radix;
   } while (data);
  }
  
  while (ind > 0)
  { 
    --ind;
   *p++ = str[ind];
  }
  return p;
}

int gethexdigit(unsigned char t) // returns -1 if not a digit
{
  if ((t >= '0') && (t <= '9')) { return t - '0'; }
  if ((t >= 'a') && (t <= 'f')) { return t - 'a' + 10; }
  if ((t >= 'A') && (t <= 'F')) { return t - 'A' + 10; }
  return -1;
}

bool readvalue(const uint8_t*& from, size_t& result)
{
  result = 0;
  bool flag = false;
  while (true)
  {
    unsigned char symbol = *from;
    int digit = gethexdigit(symbol);
    if (symbol)
    {
      ++from;
    }
    if (digit < 0) 
    {
      return flag;
    }
    flag = true;
    result = (result << 4) | digit;
  }
}

bool makecanpacket(const uint8_t* from, CanTxMsgTypeDef& msg)
{
  size_t tmp;
  bool result;
  msg.DLC = 0;
  msg.RTR = 0;
  if (!readvalue(from, tmp))
  {
    return false;
  }
  if (tmp <= 0x1FFFFFFF)
  {
    result = true;
    if (tmp > 0x7ff)
    {
      msg.ExtId = tmp;
      msg.IDE = CAN_ID_EXT;
    }
     else
     {
       msg.StdId = tmp;
       msg.IDE = CAN_ID_STD;
     }
   if (msg.StdId > 3)
      {
        volatile int z = 0;
        z = z + 1;
      }    
    while (msg.DLC < 8)
    {
      if (!readvalue(from, tmp)) break;
      do 
      {
        if (msg.DLC > 7) break;
        msg.Data[msg.DLC++] = tmp & 0xff;
        tmp >>= 8;
      } while (tmp);
    }
  }
  return result;
}

void push_can_to_uart(CanRxMsgTypeDef& msg)
{
  unsigned char tmp[36];
  unsigned char* p = tmp;
  if (msg.IDE)
    p = printdigits(p, msg.ExtId, 8, 16);
  else
    p = printdigits(p, msg.StdId, 3, 16);
  if (msg.RTR)
  {
   *p++ = ' ';
   *p++ = 'R';
  }
  else
  {
    for (size_t i = 0; i < msg.DLC; i++)
    {
      *p++ = ' ';
      p = printdigits(p, msg.Data[i], 2, 16);
    }
  }
 *p++ = 13;
 *p++ = 10;
 *p = 0;
 uart2txdeque.push(tmp);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //const char uartdata1[10]="Started\n";

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN_Init();
//  MX_USART3_UART_Init();
//  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  DWT->CTRL |= DWT_CTRL_POSTPRESET_Msk;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    
  HAL_StatusTypeDef qq;
  
  CanTxMsgTypeDef msgtosend;
  CanRxMsgTypeDef msgfromcan;
  CanRxMsgTypeDef msgfromcan1;
  hcan.pTxMsg=&msgtosend;
  hcan.pRxMsg=&msgfromcan;
  hcan.pRx1Msg=&msgfromcan1;
  
  hcan.pTxMsg->Data[0]=(uint8_t)'S';
  hcan.pTxMsg->Data[1]=(uint8_t)'t';
  hcan.pTxMsg->Data[2]=(uint8_t)'a';
  hcan.pTxMsg->Data[3]=(uint8_t)'r';
  hcan.pTxMsg->Data[4]=(uint8_t)'t';
  hcan.pTxMsg->Data[5]=(uint8_t)'e';
  hcan.pTxMsg->Data[6]=(uint8_t)'d';
  hcan.pTxMsg->Data[7]=(uint8_t)'!';
  hcan.pTxMsg->DLC=8;
  hcan.pTxMsg->IDE=0;
  hcan.pTxMsg->StdId=0x7ff;
  hcan.pTxMsg->ExtId=0;
  hcan.pTxMsg->RTR=0;

  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000 << 5; 
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_FIFO0; 
  canFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

  int canrxcount = 0;
  qq = HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2rxbyte, 1);

  DWT->CYCCNT = 0;
  HAL_Delay(10);
  volatile uint32_t time1s = DWT->CYCCNT;
  time1s = time1s;// 72 MHz!!!
  
  for (size_t i = 0; i < 1000; i++)
  {
    do
    {
     qq = HAL_CAN_Transmit_IT(&hcan);
    } while (qq != HAL_OK);
  } 
  
  qq = HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
  uint32_t maxtime[20] = {};
  while (1)
  {
    uint32_t time[20] = {};
    DWT->CYCCNT = 0;
    //uart2txdeque.push((unsigned char)0);
    while (!canrxfifo.isempty())
    {
     //qq = HAL_CAN_Receive(&hcan, CAN_FIFO0, 0);
     //if (qq != HAL_OK) break;
     msgfromcan = canrxfifo.pop();
     push_can_to_uart(msgfromcan);
     canrxcount++;
     //canrxfifo.pop();
    }
    time[1] = DWT->CYCCNT - time[0];
    if (uart2txready)
    {
      size_t blen = uart2txdeque.size();
      if (blen)
      {
        if (blen > (TX_SIZE - 1)) blen = TX_SIZE - 1;
        size_t len = uart2txdeque.pop(blen, &txbuf[0]);
        if (len)
        {
         uart2txready = 0;
         HAL_UART_Transmit_IT(&huart2, (uint8_t*) &txbuf, len);
        }
      }
    }
    time[2] = DWT->CYCCNT - time[1];
    
    HAL_StatusTypeDef Result;
    if (uart2rxdeque.isfull() && !rx2fail)
    {
      __disable_irq();
      rx2fail = true;
      uart2strreceived = 0;
      __enable_irq();
      if (cantxfifo.isfull())
      {
         msgtosend.StdId = 0x7fe;
         msgtosend.Data[0] = (uint8_t)'R';
         msgtosend.Data[1] = (uint8_t)'x';
         msgtosend.Data[2] = (uint8_t)'O';
         msgtosend.Data[3] = (uint8_t)'v';
         msgtosend.Data[4] = (uint8_t)'f';
         msgtosend.Data[5] = (uint8_t)'l';
         msgtosend.Data[6] = (uint8_t)'w';
         msgtosend.Data[7] = (uint8_t)'!';
         msgtosend.DLC = 8;
         msgtosend.IDE = 0;
         msgtosend.RTR = 0;
         cantxfifo.clear();
         hcan.pTxMsg = &msgtosend;
         do
         {
           Result = HAL_CAN_Transmit_IT(&hcan);
         } while (Result != HAL_OK);
      }
    }
    time[3] = DWT->CYCCNT - time[2];
    
    while (uart2strreceived)
    {
      uint8_t line[64], tmp;
      __disable_irq();
      uart2strreceived--;
      __enable_irq();
      size_t len = 0;
      bool skip = false;
      while (uart2rxdeque.size())
      {
        tmp = uart2rxdeque.pop();
        line[len++] = tmp;
        if (0 == tmp) break;
        if (len == sizeof(line))
        {
          skip = true;
          break;
        }
      }
      if (skip)
      {
        while (uart2rxdeque.pop());
        continue;
      }
      
      bool result = makecanpacket(line, msgtosend);
      if (!result)
        continue;
      
      cantxfifo.push(msgtosend);
    } // while (uart2strreceived)
    
    time[4] = DWT->CYCCNT - time[3];
    
    if (cantxfifo.size())
    {
     msgtosend = cantxfifo.front();
     hcan.pTxMsg = &msgtosend;
     Result = HAL_CAN_Transmit_IT(&hcan);
     if (Result == HAL_OK)
     {
      cantxfifo.pop();
     } 
      //else break;
    }
   time[5] = DWT->CYCCNT - time[4];

   for (size_t i = 0; i < 6; i++)
     if (maxtime[i] < time[i]) maxtime[i] = time[i];
    
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

  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_4TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = ENABLE;   // No automatic retransmission if error
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = ENABLE;   // 
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    uart2txready = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    char rx = uart2rxbyte;
    HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2rxbyte, 1);
    if (!rx2fail)
    {
      if (rx >= ' ')
       uart2rxdeque.push(rx);
      if (rx == 13)
      {
       uart2strreceived++;
       uart2rxdeque.push((unsigned char)0);
      }
    } else
    {
      if (rx != 13) return;
      if (!uart2rxdeque.isfull()) return;
      uart2rxdeque.clear();
      rx2fail = false;
    }
  }  
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
  /* Prevent unused argument(s) compilation warning */
  cantxready = 1;
  //UNUSED(hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hcan);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxCpltCallback can be implemented in the user file
   */
  canrxfifo.push(*(hcan->pRxMsg));
  /*
  HAL_StatusTypeDef qq;
  qq = HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
  if (qq != HAL_OK)
  {
    volatile int x = 0;
    x = x + 1;
  }*/
  
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
