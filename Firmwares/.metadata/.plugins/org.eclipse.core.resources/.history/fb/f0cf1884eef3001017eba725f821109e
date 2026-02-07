/*
 * main.c
 *      Albertalooop CANbus normal 2 node test
 *      Created on: 06-25-2025
 *      Author: Jaspreet Chhabra
 *      Ref: kiran
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main_app.h"

/* Private function prototypes -----------------------------------------------*/
void GPIO_Init(void);
void Error_handler(void);
void UART2_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq);
void CAN1_Init(void);
void CAN1_Tx();
void CAN1_Rx(void);
void CAN_Filter_Config(void);
void TIMER6_Init(void);
void Send_response(uint32_t StdId);
void Manage_Solenoids();

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
CAN_HandleTypeDef  hcan1;
TIM_HandleTypeDef  htimer6;
uint8_t req_counter = 0;
uint8_t brakes_status = 1; // 1 = engaged, 0 = disengaged
CAN_RxHeaderTypeDef RxHeader;

int main(void)
{
  printf("Hello world!");
  HAL_Init();
  SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);
  GPIO_Init();
  UART2_Init();
  TIMER6_Init();
  CAN1_Init();
  CAN_Filter_Config();

  CAN1_Tx();

  if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
  {
    Error_handler();
  }

  if( HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_handler();
  }
  // Start the TIMER interrupt
  HAL_TIM_Base_Start_IT(&htimer6);

  while(1);

  return 0;
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
  RCC_OscInitTypeDef Osc_Init;
  RCC_ClkInitTypeDef Clock_Init;
  uint8_t flash_latency=0;

  Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE ;
  Osc_Init.HSEState = RCC_HSE_ON;
  Osc_Init.PLL.PLLState = RCC_PLL_ON;
  Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  switch(clock_freq) {
  case SYS_CLOCK_FREQ_50_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 50;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 1;
    break;

  case SYS_CLOCK_FREQ_84_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 84;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 2;
    break;

  case SYS_CLOCK_FREQ_120_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 120;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
    flash_latency = 3;
    break;

  default:
    return ;
  }

  if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
  {
    Error_handler();
  }

  if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
  {
    Error_handler();
  }

  /*Configure the systick timer interrupt frequency (for every 1 ms) */
  uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
  HAL_SYSTICK_Config(hclk_freq/1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  Transmit a message via CAN1.
  * @retval None
  */
void CAN1_Tx()
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t message;

  TxHeader.DLC = 1;
  TxHeader.StdId = 0x299;
  TxHeader.IDE   = CAN_ID_STD;

  TxHeader.RTR = CAN_RTR_DATA;

  message = brakes_status;

  // board led blinks on tx
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  // send the message

  if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
  {
    Error_handler();
  }
}

/**
  * @brief  Configures the CAN filter.
  * @retval None
  */
void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef can1_filter_init;

  can1_filter_init.FilterActivation = ENABLE;
  can1_filter_init.FilterBank  = 0;
  can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
  // CANid total bits 11
  // xxx xxxx xxxx
  // 100 xxxx xxxx
  // Accept only 4XX
  // id 1000 = 0x8
  // mask 1110 = 0xE
  can1_filter_init.FilterIdHigh = 0x0000;
  can1_filter_init.FilterIdLow = 0x0000;
  can1_filter_init.FilterMaskIdHigh = 0X0000;
  can1_filter_init.FilterMaskIdLow = 0x0000;
  can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

  if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)
  {
    Error_handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
  // enabling the clocks for the GPIO ports
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // === General Output Pin ===
  GPIO_InitTypeDef ledgpio;
  ledgpio.Pin = GPIO_PIN_5;
  ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
  ledgpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &ledgpio);

  // === Solenoid Control Pins ===
  GPIO_InitTypeDef solenoid;
  solenoid.Mode = GPIO_MODE_OUTPUT_PP;
  solenoid.Pull = GPIO_NOPULL;

  // PA8, PA9
  solenoid.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOA, &solenoid);

  // PB4, PB10
  solenoid.Pin = GPIO_PIN_4 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOB, &solenoid);

  // === Button Input Pin with Interrupt ===
  ledgpio.Pin = GPIO_PIN_13;
  ledgpio.Mode = GPIO_MODE_IT_FALLING;
  ledgpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &ledgpio);

  // enables the IRQ on pins 10-15
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief TIM6 Initialization Function
  * Generates an interrupt every 1 second
  * @param None
  * @retval None
  */
void TIMER6_Init(void)
{ // APB clock1 84 MHz
  // CNT_CLK = TIMx_CLK / (prescaler+1)
  htimer6.Instance = TIM6;
  //  Every one second:
  //  htimer6.Init.Prescaler = 4999;
  //  htimer6.Init.Period = 10000-1;

  // Every 5 seconds:
  htimer6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimer6.Init.Prescaler = 49999;  // Divides 50MHz to 1kHz
  htimer6.Init.Period = 4999;      // 1kHz → 5s interrupt
  if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
  {
    Error_handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void UART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  if ( HAL_UART_Init(&huart2) != HAL_OK )
  {
    //There is a problem
    Error_handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  //  Settings related to CAN bit timings
  //  hcan1.Init.Prescaler = 3;
  //  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  //  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  //  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

  //  Settings related to CAN bit timings
  //  Setting resulting bit rate to 250k as per DALY BMS reqs
  //  The prescalar, time values were derived from http://www.bittiming.can-wiki.info/
  hcan1.Init.Prescaler = 10;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

  if ( HAL_CAN_Init (&hcan1) != HAL_OK)
  {
    Error_handler();
  }
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M0\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M1\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M2\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//  char msg[50];
//  sprintf(msg,"Brake controller recieved a message!!!!:M0\r\n");
//  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

  uint8_t rcvd_msg[8]; // 8 is max cap for std CAN

  char msg[50];

  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
  {
    Error_handler();
  }

  if(RxHeader.StdId == 0x201 && RxHeader.RTR == 0 )
  {
    brakes_status = 0;
    Manage_Solenoids();
    sprintf(msg,"0x201 engage brakes : #%x\r\n",rcvd_msg[0]);
    HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
  }

  else if ( RxHeader.StdId == 0x202 && RxHeader.RTR == 0)
  {
    brakes_status = 1;
    Manage_Solenoids();

    sprintf(msg,"0x202 disengage brakes : #%x\r\n",rcvd_msg[0]);
    HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
  }



}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * This is called whenever the timer interrupt is raised
  * We will send the heartbeat signal from this function
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ // broadcasts heartbeat signal
  CAN1_Tx();
}

void Manage_Solenoids()
{
  // based on the global uni8_t brakes_states set the pins
  GPIO_PinState state = (brakes_status == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, state);  // PA8
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, state);  // PA9
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state);  // PB4
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, state); // PB10
}


//void Send_response(uint32_t StdId)
//{
//
//  CAN_TxHeaderTypeDef TxHeader;
//
//  uint32_t TxMailbox;
//
//  uint8_t response[2] = { 0xAB,0XCD};
//
//  TxHeader.DLC = 2;
//  TxHeader.StdId = StdId;
//  TxHeader.IDE   = CAN_ID_STD;
//  TxHeader.RTR = CAN_RTR_DATA;
//
//  if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,response,&TxMailbox) != HAL_OK)
//  {
//    Error_handler();
//  }
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
  while(1);
}

