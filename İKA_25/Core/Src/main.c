/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define SIZE 16    //S00.1522ECF  M00.5210.79021CF
#define MOTOR_COUNT 6

uint8_t uart_rx_buffer[SIZE];        // Jetson'dan gelen veri.
uint8_t uart_rx[SIZE+1];             // Islenmis veri
uint8_t uart_tx[24];                 // 6 motor * 4 byte = 24 byte
uint8_t desired_duty_buffer[5]; 
uint8_t desired_duty_buffer_sag[5];
uint8_t desired_duty_buffer_sol[5];    // Hiz verisini integera d�n�st�rmekte kullanilir. 0.18
uint8_t mode_x;
uint8_t mode_y;
uint8_t laser_en;
//uint8_t target_x_buffer[5];        
//uint8_t target_y_buffer[5];
uint8_t rf_rx[1]={0};                // Wireless e-stop.
uint8_t velocity[4]={0};
uint8_t txReady = 1;
uint8_t flag;
uint8_t e_stop;
uint8_t autonomous;
volatile uint8_t e_stop_state;                // Butona basilinca 1 (input pull-up) normalde 0 ile beslenir.
uint8_t check_format; 
float value =0.0f;  
float desiredduty_sag =0.0f;
float desiredduty_sol =0.0f;
float desired_rcorners =0.0f;
float desired_lcorners=0.0f;
float desired_rmid=0.0f;
float desired_lmid =0.0f;
int desireddirection;
int desireddirection_sag;
int desireddirection_sol;
volatile float motor_velocities[MOTOR_COUNT];
//volatile int current_pos_x = 0;
//volatile int current_pos_y = 0;
//volatile int target_x = 0;
//volatile int target_y = 0;
volatile int dir_x = 0;
volatile int dir_y = 0;
int a=0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* MOTORLARIN ID TANIMLAMALARI*/

CAN_TxHeaderTypeDef motor1_duty = {  
    .ExtId = 0x02050081 ,  // Motor ID = 1
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor2_duty = {  
    .ExtId = 0x02050082 ,  // Motor ID = 2
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor3_duty = {  
    .ExtId = 0x02050083 ,  // Motor ID = 3
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor4_duty = {  
    .ExtId = 0x02050084 ,  // Motor ID = 4
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor5_duty = {  
    .ExtId = 0x02050085 ,  // Motor ID = 5
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef motor6_duty = {  
    .ExtId = 0x02050086 ,  // Motor ID = 6
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

/* DUTY CYCLE G�NDERME FONKSIYONU */

void sparkmax_send_duty(CAN_TxHeaderTypeDef *motor, float duty_cycle)
{
    
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox;
    memcpy(&TxData[0], &duty_cycle, sizeof(float));
    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, motor, TxData, &TxMailbox);

    if (result == HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
}

/* MOTORUN DINLEME MODUNDA KALMASINI SAGLAYAN FONKSIYON */

void send_heartbeat(CAN_HandleTypeDef *hcan, uint32_t device_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t TxMailbox;

    TxHeader.StdId = 0;
    TxHeader.ExtId = 0x2052C80 + device_id;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        // hata yonetimi
    }
}

void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef filterConfig;

    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;            // Mask mod
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;           // 32-bit filtre
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14; // CAN2 varsa �akismasin diye

    // Base ID ve mask (extended ID'ler STM32 i�in 3 bit sola kaydirilmali)
    uint32_t baseID = 0x2051840;
    uint32_t mask   = 0xFFFFFFC0;

    uint32_t id_shifted   = baseID << 3;
    uint32_t mask_shifted = mask << 3;

    filterConfig.FilterIdHigh     = (id_shifted >> 16) & 0xFFFF;
    filterConfig.FilterIdLow      = (id_shifted & 0xFFFF) | (1 << 2);  // IDE=1 (extended)
    filterConfig.FilterMaskIdHigh = (mask_shifted >> 16) & 0xFFFF;
    filterConfig.FilterMaskIdLow  = (mask_shifted & 0xFFFF) | (1 << 2);

    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

void process_velocity(uint32_t ext_id, uint8_t *data)
{
    uint8_t motor_id = ext_id - 0x2051840;

    if (motor_id >= 1 && motor_id <= MOTOR_COUNT)
    {
        float rpm = 0;
        memcpy(&rpm, data, sizeof(float));  // Ilk 4 byte: velocity (float RPM)
        motor_velocities[motor_id - 1] = rpm;
			
        for (int i = 0; i < 6; i++)  // Her motor icin
  {    uint8_t *p = (uint8_t *)&motor_velocities[i];  // motor i'nin adresi byte pointer olarak
       for (int j = 0; j < 4; j++)  // float 4 byte oldugu icin 4 kere kopyala
       uart_tx[i * 4 + j] = p[j];  // uart_tx dizisine byte byte kopyalama
    } 
     }  }
    

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t CAN_rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, CAN_rxData) != HAL_OK)
        return;

    // Extended ID ve Status 1 mesaji mi?
    if (rxHeader.IDE == CAN_ID_EXT && (rxHeader.ExtId & 0xFFFFFFC0) == 0x2051840) // son 6 bit kontrol edilmez!
    {
        process_velocity(rxHeader.ExtId, CAN_rxData);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)//M00.5210.790(FİRST STEP)2(2ND STEP)1(LAZER ON)CF
{
	if(huart->Instance == USART1)
    {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rf_rx, sizeof(rf_rx));
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		}
	
	if(huart -> Instance == USART2)   /* S00.0064006400CF */
	{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);    /* Veri gelince mavi led yanar */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buffer,sizeof(uart_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		
	  memcpy(uart_rx, uart_rx_buffer, SIZE);
		uart_rx[SIZE]= '\0';
		memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer)); /*rxBufferini yeni veri alimi i�in bosalt*/
			
		desireddirection_sag=uart_rx[1]-'0'; /* ASCII->integer donusumu */
		desireddirection_sol=uart_rx[6]-'0'; /* ASCII->integer donusumu */
		mode_x   = uart_rx[11]-'0';
		mode_y   = uart_rx[12]-'0';
		laser_en = uart_rx[13]-'0';
	
		
		for	(int i=0;i<=3;i++)               
		desired_duty_buffer_sag[i]=uart_rx[i+2];
		
		desired_duty_buffer_sag[4]='\0';
		desiredduty_sag=atof((char*)desired_duty_buffer_sag);   /* Desired rpm float olarak yazilir.*/
		
		for	(int i=0;i<=3;i++)
		desired_duty_buffer_sol[i]=uart_rx[i+7];
		
		desired_duty_buffer_sol[4]='\0';
		desiredduty_sol=atof((char*)desired_duty_buffer_sol);   /* Desired rpm float olarak yazilir.*/
		
   if (desireddirection_sag == 1)
	 desiredduty_sag =-desiredduty_sag;
	 else if (desireddirection_sag == 0 )
	 desiredduty_sag=(desiredduty_sag); 
	 else desiredduty_sag=0.0f;
	 
	 if (desireddirection_sol == 1)
	 desiredduty_sol = desiredduty_sol;
	 else if (desireddirection_sol == 0 )
	 desiredduty_sol=-(desiredduty_sol); 
	 else desiredduty_sol=0.0f;
	 
	                 
    		
		
		if (txReady) 
			{
        txReady = 0; /* Gonderme islemi devam ederken yeni gonderme yapilmamasi icin */
        HAL_UART_Transmit_DMA(&huart2, uart_tx, sizeof(uart_tx));
		  } 
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) 
    {
        txReady = 1; // Gonderme tamamlandi, tekrar gondermeye hazir
    }
}

int checkdata(uint8_t *uart_rx_ptr) // S00.15221CF
{
	if((*uart_rx_ptr=='S' || *uart_rx_ptr=='E' || *uart_rx_ptr=='A' || *uart_rx_ptr=='M') && (*(uart_rx_ptr + 9) == 'C') && (*(uart_rx_ptr + 10) == 'F'))
	{
		flag = 1;
		
	  autonomous=(*uart_rx_ptr == 'A')? 1 : 2;
		e_stop=(*uart_rx_ptr == 'E' || *rf_rx=='1')? 1 : 2;   }
		
  else  flag = 1;
	
	return flag;	 }


/*	void step_motor_x (int target_x)
	{
		    if(current_pos_x==target_x)	 
				{  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	         HAL_TIM_Base_Stop_IT(&htim3);  }	
				
				else if(current_pos_x<target_x){
				 dir_x=1;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	       HAL_TIM_Base_Start_IT(&htim3);  	}	
				
			   else {
				 dir_x=0;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	       HAL_TIM_Base_Start_IT(&htim3);   }
	  		
			}
		
void step_motor_y (int target_y)
	{
		    if(current_pos_y==target_y)	 
				{  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	         HAL_TIM_Base_Stop_IT(&htim5);  }	
				
				else if(current_pos_y<target_y){
				 dir_y=1;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				 HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	       HAL_TIM_Base_Start_IT(&htim5);  	}	
				
			   else {
				 dir_y=0;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				 HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	       HAL_TIM_Base_Start_IT(&htim5);   }
	  		
			}		*/
			
void step_motor_x (void)
	{
			
		    if(mode_x ==0 )	 
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	           	
				
				else if(mode_x ==1 ){
				 dir_x=1;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	       	}	
				
			   else if(mode_x ==2 ){
				 dir_x=0;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	        }
	  		
			}
		
void step_motor_y (void)
	{
		    if(mode_y == 0)	 
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	       
				
				else if(mode_y == 1){
				 dir_y=1;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 	}	
	       
				
			   else if(mode_y == 2){
				 dir_y=0;
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); }
	   
	  		
			}					

void laser_enable	(void){
	
	if(laser_en==1)
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);

	else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
 //	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 500);
 // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);

	
	/* UART2 BASLATILIR */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buffer,sizeof(uart_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	
	/* UART1 BASLATILIR */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rf_rx,sizeof(rf_rx));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	
	/* TIM2 IT BASLATILIR */
		HAL_TIM_Base_Start_IT(&htim2);
		
	/* CAN BASLATILIR */	
	  CAN_Filter_Config();
    HAL_CAN_Start(&hcan1);
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // RX0 kesmesi aktiflestirilir

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		e_stop_state = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);  
		
		check_format = checkdata(uart_rx);
		
		if ( e_stop_state ==1)   {
			
			e_stop = 1;
			desired_rcorners=desired_lcorners=0.00f;
		  desired_rmid=desired_lmid=0.00f;
		  }
		
		else e_stop=2;
			
			
		
		if(check_format==1){
		
		if(autonomous==2) 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET); // Otonomda degilse LED s�rekli yanar.
		
    if(e_stop == 1 || *rf_rx=='1')			
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	// E-stop'a girilmisse 1 verir cikisa. */
		
		else
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET); // E-stop'a girilmemisse 0 verir cikisa.
		
		step_motor_x();
		step_motor_y();
		laser_enable();
		
		
		if (e_stop == 1 || *rf_rx == '1') {
    for (int i = 1; i <= 6; i++)
    send_heartbeat(&hcan1, i);  // SparkMAX dinleme modunda kalir
    sparkmax_send_duty(&motor1_duty, 0.0f);
    sparkmax_send_duty(&motor2_duty, 0.0f);
    sparkmax_send_duty(&motor3_duty, 0.0f);
    sparkmax_send_duty(&motor4_duty, 0.0f);
    sparkmax_send_duty(&motor5_duty, 0.0f);
    sparkmax_send_duty(&motor6_duty, 0.0f);  }

		else {

    for (int i=1; i<=6; i++)
		send_heartbeat(&hcan1,i);
		sparkmax_send_duty(&motor1_duty,desiredduty_sol);
		sparkmax_send_duty(&motor2_duty,desiredduty_sag);
		sparkmax_send_duty(&motor3_duty,desiredduty_sol);
		sparkmax_send_duty(&motor4_duty,desiredduty_sag);
		sparkmax_send_duty(&motor5_duty,desiredduty_sol);
		sparkmax_send_duty(&motor6_duty,desiredduty_sag);  }
		
		
			  
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, E_STOP_TRIGGER_Pin|LED_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LazerX_DIR_Pin|LazerY_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LASER_OUT_Pin|Ye_il_LED_Pin|Turuncu_LED_Pin|K_rm_z__LED_Pin
                          |Mavi_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : E_STOP_TRIGGER_Pin LED_OUTPUT_Pin */
  GPIO_InitStruct.Pin = E_STOP_TRIGGER_Pin|LED_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : E_STOP_IN_Pin */
  GPIO_InitStruct.Pin = E_STOP_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_STOP_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LazerX_DIR_Pin LazerY_DIR_Pin */
  GPIO_InitStruct.Pin = LazerX_DIR_Pin|LazerY_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_OUT_Pin */
  GPIO_InitStruct.Pin = LASER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LASER_OUT_GPIO_Port, &GPIO_InitStruct);

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
