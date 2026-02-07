/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h" // Wrapper for FreeRTOS (Standard in STM32)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

// --- RTOS Objects ---
TaskHandle_t hBrakeTask;
TaskHandle_t hHeartbeatTask;
QueueHandle_t CANRxQueue;    // The mailbox for incoming messages
SemaphoreHandle_t StatusMutex; // The lock for the shared variable

// --- Shared Data ---
// Protected by StatusMutex
uint8_t brakes_status = 1;

// --- Data Structure for Queue ---
typedef struct {
    uint32_t StdId;
    uint8_t Data[8];
} CAN_Msg_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE(uint8_t clock_freq);
void GPIO_Init(void);
void UART2_Init(void);
void CAN1_Init(void);
void CAN_Filter_Config(void);
void Manage_Solenoids(uint8_t state);

// --- Task Prototypes ---
void StartBrakeTask(void *argument);
void StartHeartbeatTask(void *argument);

int main(void)
{
  HAL_Init();
  SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);
  GPIO_Init();
  UART2_Init();
  CAN1_Init();
  CAN_Filter_Config();

  // 1. Create Synchronization Objects
  // Mutex to protect 'brakes_status'
  StatusMutex = xSemaphoreCreateMutex();

  // Queue to hold up to 10 CAN messages
  CANRxQueue = xQueueCreate(10, sizeof(CAN_Msg_t));

  // 2. Create Tasks
  xTaskCreate(StartBrakeTask,       // Function
              "BrakeControl",       // Name
              256,                  // Stack size (words)
              NULL,                 // Parameter
              osPriorityHigh,       // Priority: HIGH (Critical!)
              &hBrakeTask);         // Handle

  xTaskCreate(StartHeartbeatTask,   // Function
              "Heartbeat",          // Name
              128,                  // Stack size
              NULL,                 // Parameter
              osPriorityNormal,     // Priority: Low
              &hHeartbeatTask);     // Handle

  // 3. Start CAN Hardware
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  printf("RTOS Starting...\r\n");

  // 4. Hand over control to the Scheduler (This function never returns)
  vTaskStartScheduler();

  while (1); // We should never get here
}

/* ============================================================================== */
/* =============================   INIT FUNCTIONS   ============================= */
/* ============================================================================== */

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

  default:GPIO_Init
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



/* ============================================================================== */
/* =============================   RTOS TASKS   ================================= */
/* ============================================================================== */

/**
 * @brief High Priority Task.
 * Waits for data from the Queue (sent by ISR) and controls hardware.
 */
void StartBrakeTask(void *argument)
{
  CAN_Msg_t receivedMsg;
  char uart_buf[50];

  for(;;)
  {
    // Block (sleep) indefinitely until a message arrives in the Queue
    if (xQueueReceive(CANRxQueue, &receivedMsg, portMAX_DELAY) == pdTRUE)
    {
      // --- CRITICAL SECTION START ---
      // We are about to change the shared variable
      xSemaphoreTake(StatusMutex, portMAX_DELAY);

      if (receivedMsg.StdId == 0x201)
      {
        brakes_status = 0; // Engage
        sprintf(uart_buf, "CMD: Engage (0x201)\r\n");
      }
      else if (receivedMsg.StdId == 0x202)
      {
        brakes_status = 1; // Disengage
        sprintf(uart_buf, "CMD: Disengage (0x202)\r\n");
      }

      // Update hardware immediately
      Manage_Solenoids(brakes_status);

      xSemaphoreGive(StatusMutex);
      // --- CRITICAL SECTION END ---

      // Print is now safe here (unlike in the ISR)
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 10);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Blink LED
    }
  }
}

/**
 * @brief Low Priority Task.
 * Wakes up every 5 seconds to send status.
 */
void StartHeartbeatTask(void *argument)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t current_status;

  TxHeader.StdId = 0x299;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 1;

  for(;;)
  {
    // 1. Read the shared variable safely
    xSemaphoreTake(StatusMutex, portMAX_DELAY);
    current_status = brakes_status;
    xSemaphoreGive(StatusMutex);

    // 2. Send CAN Message
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &current_status, &TxMailbox) != HAL_OK)
    {
      // Handle error (optional)
    }

    // 3. Sleep for 5000ms
    // Unlike HAL_Delay, this releases the CPU for other tasks!
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/* ============================================================================== */
/* ===========================   INTERRUPTS   =================================== */
/* ============================================================================== */

/**
 * @brief  Rx FIFO 0 message pending callback.
 * NOW THIN: Only puts data in queue.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t rcvd_data[8];
  CAN_Msg_t msg_to_send;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rcvd_data) == HAL_OK)
  {
    msg_to_send.StdId = RxHeader.StdId;
    memcpy(msg_to_send.Data, rcvd_data, 8);

    // Send to Queue "From ISR"
    xQueueSendFromISR(CANRxQueue, &msg_to_send, &xHigherPriorityTaskWoken);
  }

  // Force a context switch if the BrakeTask (High Prio) was waiting for this
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*Helper to drive pins (Logic unchanged)*/
void Manage_Solenoids(uint8_t state)
{
  GPIO_PinState pin_state = (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, pin_state);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_10, pin_state);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
  while(1);
}

