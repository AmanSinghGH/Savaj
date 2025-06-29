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
#include "Cytron.h"
#include "WheelCalculation.h"
#include "String.h"
#include "stdlib.h"
#include <math.h>
#include <bno055_stm32.h>
#include"servo_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HPS166_CMD_START_RANGING  {0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72}
#define HPS166_RESPONSE_SIZE       15
#define RESPONSE_HEADER            0x0A

#define MAX_INTEGRAL 3.0f  // Example threshold

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float heading;
float raw_heading;
uint8_t RX_BUFFER[32];
uint16_t buf_size = 0;
int pot1, pot2, pot3, but1, but2;
int dirx, diry, passbtn, shootbtn, basketbtn, dribbleturnbtn;
int m1,m2,m3;
float target_angle = 0;
float current_angle = 0;
float error, correction,r1;
int stopSignalReceived = 0;
int fatakflag=0;
float distance_from_basket = 0;
float lower_motor_speed = 0;
int lower_ms = 0;

float deadZone = 2;

float degree = 0.0;

// === IMU Control ===
bool imu_initialized = false;
float initial_heading_offset = 0.0; // To store starting heading offset
float target_heading = 0.0;          // Target heading relative to offset
float Kp = 0.2, Ki = 0.0002, Kd = 0.014;                     // Proportional gain


double dx, dy, angle_calc;
int basket_x = 4000; // Replace with actual values
int basket_y = 13770;

uint8_t MSG[35] = {'\0'};
int iX,iY,checkValue;
int up=0;
//int iX,iY;

uint8_t response1[HPS166_RESPONSE_SIZE], response2[HPS166_RESPONSE_SIZE], response3[HPS166_RESPONSE_SIZE];
volatile uint8_t data_ready1 = 0, data_ready2 = 0, data_ready3 = 0;
uint16_t distance_mm1, distance_mm2, distance_mm3;
uint16_t Tdistance_mm1, Tdistance_mm2, Tdistance_mm3;
float distance_meters1, distance_meters2, distance_meters3;
int toggleState = 0;          // Current toggle state
int previousToggleState = 0;  // Previous toggle state
uint32_t lastUpdate;

// Define stateFlags for different conditions
bool stateFlagPass = false;
bool stateFlagBasket = false;

// Variables to lock the distance values
int locked_distance_mm3 = 0;
int locked_distance_mm2 = 0;

volatile int32_t M1_count = 0;
volatile uint8_t M1_lastEncoded = 0;
volatile int32_t M2_count = 0;
volatile uint8_t M2_lastEncoded = 0;
volatile int32_t M3_count = 0;
volatile uint8_t M3_lastEncoded = 0;

int32_t last1, last2, last3;
#define PI 3.141592653589793
#define TICKS_PER_REV 2400
#define WHEEL_DIAMETER_M 0.152
#define DISTANCE_PER_TICK ((PI * WHEEL_DIAMETER_M) / TICKS_PER_REV)
#define L 0.316 // distance from robot center to wheel in meters

float posX = 0.0, posY = 0.0, theta2 = 0.0;
long last_m1 = 0, last_m2 = 0, last_m3 = 0;

int broadcastAngle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void parseData(uint8_t *buffer, uint16_t size);
void sendContinuousRangingCommand(UART_HandleTypeDef *huart);
void processResponse(UART_HandleTypeDef *huart, uint8_t *response, uint16_t *distance_mm, float *distance_meters);
void restartReception(UART_HandleTypeDef *huart, uint8_t *response);
float get_yaw(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void sendContinuousRangingCommand(UART_HandleTypeDef *huart) {
    uint8_t startRangingCmd[] = HPS166_CMD_START_RANGING;
    HAL_UART_Transmit(huart, startRangingCmd, sizeof(startRangingCmd), HAL_MAX_DELAY);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1 && Size == HPS166_RESPONSE_SIZE && response1[0] == RESPONSE_HEADER) {
	        data_ready1 = 1;
	        processResponse(huart, response1, &distance_mm1, &distance_meters1);
	        data_ready1 = 0;
	        restartReception(&huart1, response1);
	    } else if (huart == &huart4 && Size == HPS166_RESPONSE_SIZE && response2[0] == RESPONSE_HEADER) {
	        data_ready2 = 1;
	        processResponse(huart, response2, &distance_mm2, &distance_meters2);
	        data_ready2 = 0;
	        restartReception(&huart4, response2);
	    } else if (huart == &huart7 && Size == HPS166_RESPONSE_SIZE && response3[0] == RESPONSE_HEADER) {
	        data_ready3 = 1;
	        processResponse(huart, response3, &distance_mm3, &distance_meters3);
	        data_ready3 = 0;
	        restartReception(&huart7, response3);
	    } else{
	    	buf_size = Size;
	    	RX_BUFFER[buf_size] = '\0';
	    	parseData(RX_BUFFER, buf_size);
	    	HAL_UARTEx_ReceiveToIdle_IT(huart, RX_BUFFER, 32);
	    }
}

void parseData(uint8_t *buffer, uint16_t size)
{
	char *token;
	int values[6]; // Array to hold parsed integers
	int index = 0;

	// Tokenize the buffer using comma as a delimiter
	token = strtok((char *)buffer, ",");
	while (token != NULL && index < 6)
	{
		values[index++] = atoi(token);
		token = strtok(NULL, ",");
	}

	// Assign values to corresponding variables (ensure the order matches)
	if (index >= 6) // Ensure enough data is received
	{
		dirx = values[0];
		diry = values[1];
		basketbtn = values[2];
		shootbtn = values[3];
		passbtn = values[4];
		dribbleturnbtn = values[5];
//		xtraRotate = values[6];
//		broadcastAngle = values[7];

	}
}

float get_yaw(void)
{
    bno055_vector_t euler = bno055_getVectorEuler();
    float yaw = euler.x;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    return yaw;
}

void processResponse(UART_HandleTypeDef *huart, uint8_t *response, uint16_t *distance_mm, float *distance_meters) {
    *distance_mm = (response[5] << 8) | response[6];
    *distance_meters = *distance_mm / 1000.0;
}

void restartReception(UART_HandleTypeDef *huart, uint8_t *response) {
    HAL_UARTEx_ReceiveToIdle_IT(huart, response, HPS166_RESPONSE_SIZE);
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
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart8, RX_BUFFER, 32);

  restartReception(&huart1, response1);
  restartReception(&huart4, response2);
  restartReception(&huart7, response3);

  sendContinuousRangingCommand(&huart1);
  sendContinuousRangingCommand(&huart4);
  sendContinuousRangingCommand(&huart7);

  bno055_assignI2C(&hi2c2);
  bno055_setup();
  bno055_setOperationModeNDOF();

  CytronMotor_t motor1;
  CytronMotor_t motor2;
  CytronMotor_t motor3;
  CytronMotor_t motor4;

  CytronMotor_t shooterM1;
  CytronMotor_t shooterM2;

  CytronMotor_t tandomM1;
  CytronMotor_t tandomM2;

  CytronMotor_Init(&motor1, &htim8, TIM_CHANNEL_4, GPIOF, GPIO_PIN_14);
  CytronMotor_Init(&motor2, &htim8, TIM_CHANNEL_3, GPIOF, GPIO_PIN_3);
  CytronMotor_Init(&motor3, &htim8, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15);
  CytronMotor_Init(&motor4, &htim8, TIM_CHANNEL_2, GPIOF, GPIO_PIN_13);

  CytronMotor_Init(&shooterM1, &htim1, TIM_CHANNEL_4, GPIOF, GPIO_PIN_11);
  CytronMotor_Init(&shooterM2, &htim1, TIM_CHANNEL_1, GPIOF, GPIO_PIN_12);

  CytronMotor_Init(&tandomM1, &htim1, TIM_CHANNEL_3, GPIOG, GPIO_PIN_5);
  CytronMotor_Init(&tandomM2, &htim1, TIM_CHANNEL_2, GPIOG, GPIO_PIN_8);

  Servo_t fatak;
  Servo_Init(&fatak, &htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  initial_heading_offset = get_yaw(); // Set target angle as current angle
  target_heading = 0.0; // start at zero relative heading
  degree = target_heading;
  imu_initialized = true;

//  uint32_t lastUpdate = 0;

//  CytronMotor_Speed(&shooterM1, -800);
//  CytronMotor_Speed(&shooterM2, 960);
//  HAL_Delay(3000);
//  CytronMotor_Speed(&shooterM1, 0);
//  CytronMotor_Speed(&shooterM2, 0);
Servo_SetAngle(&fatak, 90);   // Move to 90°
  while (1)
  {


	HAL_Delay(100);  // or timer-based update

  	sprintf(MSG,"%d,%d,%d\n",distance_mm3,7600,broadcastAngle);
 	HAL_UART_Transmit(&huart8, MSG, sizeof(MSG), 100);

	uint32_t currentTick = HAL_GetTick();
      if (currentTick - lastUpdate >= 100)
      {
        lastUpdate = currentTick;

	    // Check for emergency stop signal from the controller
//        if (shootbtn) // Replace with actual function to detect stop signal
//        {
//            stopSignalReceived = 1;
//        }
//
//        if (stopSignalReceived)
//        {
//        	m1=0, m2=0, m3=0;
//            // Set all motor speeds to zero
//            CytronMotor_Speed(&motor1, m1);
//            CytronMotor_Speed(&motor2, m2);
//            CytronMotor_Speed(&motor3, m3);
//            continue; // Skip further processing
//        }

	        raw_heading = get_yaw();
	        float current_heading = raw_heading - initial_heading_offset;

	        // Normalize heading to [-180, 180]
	        if (current_heading > 180) current_heading -= 360;
	        if (current_heading < -180) current_heading += 360;
	        broadcastAngle  = target_heading;
	        if ((passbtn > 1450 && passbtn <= 1550) && (dribbleturnbtn < 1550 && dribbleturnbtn > 1450) && (basketbtn == 0)) {
	            target_heading = 0.0;
	            stateFlagPass = false;
	            stateFlagBasket = false;
//	      	    Servo_SetAngle(&fatak, 90);   // Move to 90°

	            if(up==1){
//		            CytronMotor_Speed(&tandomM1,-400);
//		            CytronMotor_Speed(&tandomM2,-400);
//		            HAL_Delay(500);
//		            CytronMotor_Speed(&tandomM1,0);
//		            CytronMotor_Speed(&tandomM2,0);
		            up=0;
	            }


	        } else if ((passbtn < 1450) && (basketbtn == 0)) {
	            // Check if the stateFlag is false
	            if (!stateFlagPass) {
	                // Lock the variable values
	                locked_distance_mm3 = distance_mm3;	//X lidar
	                locked_distance_mm2 = distance_mm2; //Y lidar
	                stateFlagPass = true;
	            }
	            up=1;
//	            CytronMotor_Speed(&tandomM1,800);
//	            CytronMotor_Speed(&tandomM2,800);
//	            HAL_Delay(700);
//	            CytronMotor_Speed(&tandomM1,50);
//	            CytronMotor_Speed(&tandomM2,50);

	        } else if ((passbtn > 1550) && (basketbtn == 0)) {
	            // Check if the stateFlag is false
	            if (!stateFlagPass) {
	                // Lock the variable values
	                locked_distance_mm3 = distance_mm3;
	                locked_distance_mm2 = distance_mm2;
	                stateFlagPass = true;
	            }
//	            target_heading = broadcastAngle;


	        } else if (basketbtn ) {
	            // Check if the stateFlag is false
	            if (!stateFlagBasket) {
	                // Lock the variable values
	                locked_distance_mm3 = distance_mm3;
	                locked_distance_mm2 = distance_mm2;
	                stateFlagBasket = true;
	            }

	            // Calculate heading using locked values
	            int dx = basket_x - locked_distance_mm3 + 100 ;
	            int dy = basket_y - 7800;
	            angle_calc = atan2(dx, dy) * 180.0 / M_PI;
	            angle_calc = fmod((angle_calc + 360.0), 360.0);
	            target_heading = angle_calc - 5;
	            distance_from_basket = sqrt((dx*dx) +(dy*dy));
		        broadcastAngle  = target_heading;
	        }
//	        else {
//
//	        }


	        if(shootbtn==1 )
	        {
	          lower_motor_speed = 1000 * (1 - pow(M_E, (-0.39 * (distance_from_basket / 1000))));
//	       	  lower_motor_speed = 1000 * (1 - pow(M_E, (-0.475 * 6))); //0.48 for 3-4 mtrs
	          lower_ms = (int)lower_motor_speed;
	          CytronMotor_Speed(&shooterM1, -lower_ms);
	          CytronMotor_Speed(&shooterM2, (0.82*lower_ms));
//	        	CytronMotor_Speed(&shooterM1,-1000);
//				CytronMotor_Speed(&shooterM2,1000);
	          HAL_Delay(4000);
	          if(fatakflag==0)
	          {
	    	  Servo_SetAngle(&fatak, 32);
	    	  fatakflag=1;
	          }

	        }
	        else if(shootbtn==0)
	        {
		          CytronMotor_Speed(&shooterM1, 0);
		          CytronMotor_Speed(&shooterM2, 0);
		          Servo_SetAngle(&fatak, 90);
		          fatakflag=0;
	        }
	        if((dribbleturnbtn >1800 || dribbleturnbtn < 1400) && (passbtn>=1450 && passbtn<=1550) )
	        {
	        	target_heading = 90.00;
	        }
	          float error = target_heading - current_heading;
	          if (error > 180) error -= 360;
	          if (error < -180) error += 360;
	          // Integral component
	          static float integral = 0;  // Retain value between iterations
	                  // Accumulate error over time
	          static float previous_error = 0;
	          static uint32_t lastPIDUpdate = 0;


	          uint32_t currentTime = HAL_GetTick();
	          float dt = (currentTime - lastPIDUpdate)/ 1000.0f;
	          lastPIDUpdate = currentTime;
	          float derivative = (error - previous_error) / dt;
	          previous_error = error;

	          integral += error*dt;

	          // Proportional and integral control
	          r1 = Kp * error + Ki * integral + Kd* derivative;

	          // Optional: Anti-windup for integral component
	          if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
	          if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
	          degree = -target_heading;

	        calculateWheelSpeeds(dirx, diry, &m1, &m2, &m3, &r1, &degree);
	        CytronMotor_Speed(&motor1, m1);
	        CytronMotor_Speed(&motor2, m2);
	        CytronMotor_Speed(&motor3, m3);
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x6000030D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 216-1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 216-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 216-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 216-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DIR8_Pin|DIR3_Pin|DIR4_Pin|DIR5_Pin
                          |DIR6_Pin|DIR7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, P1_Pin|DIR1_Pin|P2_Pin|P3_Pin
                          |DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR8_Pin DIR3_Pin DIR4_Pin DIR5_Pin
                           DIR6_Pin DIR7_Pin */
  GPIO_InitStruct.Pin = DIR8_Pin|DIR3_Pin|DIR4_Pin|DIR5_Pin
                          |DIR6_Pin|DIR7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : E1A_Pin */
  GPIO_InitStruct.Pin = E1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E1A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E2A_Pin E1B_Pin E3B_Pin E3A_Pin
                           E2B_Pin */
  GPIO_InitStruct.Pin = E2A_Pin|E1B_Pin|E3B_Pin|E3A_Pin
                          |E2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_Pin DIR1_Pin P2_Pin P3_Pin
                           DIR2_Pin */
  GPIO_InitStruct.Pin = P1_Pin|DIR1_Pin|P2_Pin|P3_Pin
                          |DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
