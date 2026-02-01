/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Optimized: ISR Cleaned, Logic in Main)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SIZE 13                // Veri Paketi Boyutu
#define MOTOR_COUNT 2
#define CAN_SEND_PERIOD 10     // 10ms (100Hz)
#define WATCHDOG_TIMEOUT 500   // 500ms Timeout

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* BUFFER YONETIMI */
uint8_t uart_rx_dma_buffer[SIZE];   // DMA'nın surekli doldurdugu dizi
uint8_t uart_rx_process_buffer[SIZE+1]; // Mainde islenecek güvenli kopya

uint8_t uart_tx[8];                 
uint8_t heart_beat[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

/* FLAGLER VE KONTROL DEGISKENLERI */
volatile uint8_t new_uart_data_arrived = 0;   // ISR'den Main'e haber veren bayrak
volatile float motor_velocities[MOTOR_COUNT];


float desiredduty_sag = 0.0f;
float desiredduty_sol = 0.0f;
uint8_t txReady = 1;
uint8_t mode = 0;       
uint8_t e_stop_state = 0;


uint32_t last_can_send_time = 0;   
uint32_t last_valid_data_time = 0; // Watchdog icin

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* MOTOR ID TANIMLAMALARI */
CAN_TxHeaderTypeDef motor1_duty = {  
    .ExtId = 0x02050081, 
		.IDE = CAN_ID_EXT,
		.RTR = CAN_RTR_DATA, 
		.DLC = 8, .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor2_duty = {  
    .ExtId = 0x02050082, 
		.IDE = CAN_ID_EXT,
		.RTR = CAN_RTR_DATA, 
		.DLC = 8, .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor1_heartbeat = {  
    .ExtId = 0x02052C81 ,  // Motor ID = 1
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor2_heartbeat = {  
    .ExtId = 0x02052C82 ,  // Motor ID = 2
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

/*  DUTY CYCLE GONDERME */
void sparkmax_send_duty(CAN_TxHeaderTypeDef *motor, float duty_cycle)
{
    /* Eger 3 mailbox da doluysa en fazla 2ms boyunca yer açılmasını bekle */
    uint32_t tickstart = HAL_GetTick();
    
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        // Eger 2ms gectiyse ve hala yer acilmadiysa donguden cik.
        if ((HAL_GetTick() - tickstart) > 2) 
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Kirmizi LED Timeout hatasi.
            return; 
        }
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) // Mail box musitse veri gonderimi yapar.
    {
        uint8_t TxData[8]={0};
        uint32_t TxMailbox;
        memcpy(TxData, &duty_cycle, sizeof(float));
        
        if (HAL_CAN_AddTxMessage(&hcan1, motor, TxData, &TxMailbox) == HAL_OK)
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // turuncu led (Basarili)
        }
        else  
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // kirmizi led (Hata)
        }
    }
    else 
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // kirmizi led (Yer yok)
    }
}

/* MOTORUN CALİSİR DURUMDA KALMASI ICIN HEARTBEAT FONKSIYONU */
void send_heartbeat(CAN_HandleTypeDef *hcan ,CAN_TxHeaderTypeDef *motorn_heartbeat) 
{ 

    uint32_t tickstart = HAL_GetTick();
    
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if ((HAL_GetTick() - tickstart) > 2) return;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) // Mail box musitse veri gonderimi yapar.
    {
        uint32_t TxMailbox;

        if (HAL_CAN_AddTxMessage(hcan, motorn_heartbeat, heart_beat, &TxMailbox) != HAL_OK) 
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // kirmizi led
        }
    }      
}

/* CAN FILTRE AYARI */
void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef filterConfig;
    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14; 

	uint32_t baseID = 0x2051840; // beklenen veri seti : 0x2051841 ve 0x2051842.
    uint32_t mask = 0xFFFFFFFC; // son bit 0-3 arasi ise veri filtreden gecer. (0x205184x)
    uint32_t id_shifted = baseID << 3;
    uint32_t mask_shifted = mask << 3;

    filterConfig.FilterIdHigh= (id_shifted >> 16) & 0xFFFF;
    filterConfig.FilterIdLow = (id_shifted & 0xFFFF) | (1 << 2); 
    filterConfig.FilterMaskIdHigh = (mask_shifted >> 16) & 0xFFFF;
    filterConfig.FilterMaskIdLow = (mask_shifted & 0xFFFF) | (1 << 2);

    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

void process_velocity(uint32_t ext_id, uint8_t *data)
{
    uint8_t motor_id = ext_id - 0x2051840;   
    if (motor_id >= 1 && motor_id <= MOTOR_COUNT)
    {
        float rpm = 0;
        memcpy(&rpm, data, sizeof(float));  
        motor_velocities[motor_id - 1] = rpm;
        for (int i = 0; i < 2; i++) {    
            uint8_t *p = (uint8_t *)&motor_velocities[i];  
            for (int j = 0; j < 4; j++) uart_tx[i * 4 + j] = p[j];  
        } 
    }  
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t CAN_rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, CAN_rxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOD, Ye_il_LED_Pin); 
        if (rxHeader.IDE == CAN_ID_EXT && (rxHeader.ExtId & 0xFFFFFFC0) == 0x2051840) 
            process_velocity(rxHeader.ExtId, CAN_rxData);
    }
}

/* UART VERI ALMA */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)   
    {
			 
       memcpy(uart_rx_process_buffer, uart_rx_dma_buffer, SIZE); // Gelen veri islenmesi icin baska buffer'a kopyalanir.
       uart_rx_process_buffer[SIZE] = '\0'; 
       new_uart_data_arrived = 1;   // uart bayragi kaldirilir.
       HAL_UART_Receive_DMA(&huart2, uart_rx_dma_buffer, SIZE); // veri alımı tekrar başlatılır.
       __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); 
			 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    }
}

/* UART VERI GONDERME */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
		{	
			txReady = 1;  // Yeni gonderim icin bayrak kaldirilir.
		}
}
/* FORMAT KONTROLU */
int checkdata_format(uint8_t *ptr) 
{
    if((ptr[0]=='S' || ptr[0]=='E' || ptr[0]=='A' || ptr[0]=='M')&& (ptr[SIZE - 2] == 'C')&& (ptr[SIZE - 1] == 'F'))               
    {
        return 1; // Format Doğru
    }
		else 		return 0; // Hatalı
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    
    /* UART KONFIGURASYONU */
    HAL_UART_Receive_DMA(&huart2, uart_rx_dma_buffer, SIZE);
    
	 /* CAN KONFIGURASYONU */
    CAN_Filter_Config();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
    /* WATCHDOG KONFIGURASYONU  */
    last_can_send_time = HAL_GetTick();
    last_valid_data_time = HAL_GetTick();  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
								
		
       /* UART VERI ISLEME */
        if (new_uart_data_arrived == 1)
        {
            new_uart_data_arrived = 0; // Bayragı indir
            
            // Format Kontrolu
            if (checkdata_format(uart_rx_process_buffer) == 1)
            {
                last_valid_data_time = HAL_GetTick(); // Watchdog'u resetlenir. 
                //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

                char start_char = uart_rx_process_buffer[0];
                if (start_char == 'E') mode = 2;
                else if (start_char == 'A') mode = 1;
                else mode = 0;

                char temp_buff[5];
                
                // Sag Motor Parse
                int dir_sag = uart_rx_process_buffer[1] - '0';
                strncpy(temp_buff, (char*)&uart_rx_process_buffer[2], 4);
                temp_buff[4] = '\0';
                float val_sag = atof(temp_buff);
                
                // Sol Motor Parse
                int dir_sol = uart_rx_process_buffer[6] - '0';
                strncpy(temp_buff, (char*)&uart_rx_process_buffer[7], 4);
                temp_buff[4] = '\0';
                float val_sol = atof(temp_buff);

                // Yonlerin belirlenmesi
                if (dir_sag == 1) desiredduty_sag = -val_sag;
                else if (dir_sag == 0) desiredduty_sag = val_sag;
                else desiredduty_sag = 0.0f;

                if (dir_sol == 1) desiredduty_sol = val_sol; 
                else if (dir_sol == 0) desiredduty_sol = -val_sol;
                else desiredduty_sol = 0.0f;

                // Geri Bildirim Gonderimi
                if (txReady) 
                {
                    txReady = 0; 
                    HAL_UART_Transmit_DMA(&huart2, uart_tx, sizeof(uart_tx));
									  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
                }
            }
						
        }

        /* GUVENLIK & WATCHDOG */
       
				if (HAL_GetTick() - last_valid_data_time > WATCHDOG_TIMEOUT)
        {
             HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // kirmizi led.
             
             if (HAL_GetTick() - last_can_send_time >= CAN_SEND_PERIOD)
             {
                 last_can_send_time = HAL_GetTick();
							   send_heartbeat(&hcan1,&motor1_heartbeat);
						     send_heartbeat(&hcan1,&motor2_heartbeat);
                 sparkmax_send_duty(&motor1_duty, 0.0f);
                 sparkmax_send_duty(&motor2_duty, 0.0f);
                 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); // Röle Kes
								 
             }

        }     

        /* MOTOR SURME */
        if (HAL_GetTick() - last_can_send_time >= CAN_SEND_PERIOD)
        {
            last_can_send_time = HAL_GetTick(); 
            e_stop_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);


            if (e_stop_state == 1 || mode == 2) // Donanimsal veya Yazilimsal E-Stop
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); // Röle
								send_heartbeat(&hcan1,&motor1_heartbeat);
								send_heartbeat(&hcan1,&motor2_heartbeat);
                sparkmax_send_duty(&motor1_duty, 0.0f);
                sparkmax_send_duty(&motor2_duty, 0.0f);
            }
            else
            {
                
                if (mode == 0)     // Manuel
									{
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                  } 
								else               // Otonom
									{ 
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                  }
                send_heartbeat(&hcan1,&motor1_heartbeat);
								send_heartbeat(&hcan1,&motor2_heartbeat);             
                sparkmax_send_duty(&motor1_duty, desiredduty_sag);
                sparkmax_send_duty(&motor2_duty, desiredduty_sol);
            }
        }
				

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_Trigger_Pin|LED_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Ye_il_LED_Pin|Turuncu_LED_Pin|K_rm_z__LED_Pin|Mavi_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : E_stop_read_Pin */
  GPIO_InitStruct.Pin = E_stop_read_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_stop_read_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_Trigger_Pin LED_Trigger_Pin */
  GPIO_InitStruct.Pin = Relay_Trigger_Pin|LED_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Ye_il_LED_Pin Turuncu_LED_Pin K_rm_z__LED_Pin Mavi_LED_Pin */
  GPIO_InitStruct.Pin = Ye_il_LED_Pin|Turuncu_LED_Pin|K_rm_z__LED_Pin|Mavi_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        HAL_UART_Receive_DMA(&huart2, uart_rx_dma_buffer, SIZE); 
				
    }
}

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
