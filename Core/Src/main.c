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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "distance_buffer.h"

/*
Jorge Larach, September 28 2025
Ball and Beam Project
 */
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HCSR04_TRIG_DWT_init(void);
void HCSR04_TRIG_DWTDelay_us(uint32_t uSec);
void HCSR04_trigger_pulse(void);
uint32_t SERVO_set_angle(uint8_t angle);
uint32_t SERVO_calculate_pulse_width(uint8_t angle);
void UART_print_distance(float distance);
void UART_print_average(void);
uint32_t PID_proportional(float measured_distance);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t capture_idx = 0;
volatile uint32_t edge_1_time = 0, edge_2_time = 0;
volatile float latest_distance = -1.0f;  // shared between ISR and main
volatile uint8_t distance_ready = 0;     // flag to tell main loop new data is ready

const float SPEED_OF_SOUND_CM_US = 0.0343f; // cm per microsecond
const uint8_t setpoint_cm = 20;

const float Kp = 8.0f;
const float Ki = 0.8f;
const float Kd = 1.2f;

const float dt = 0.05f;   // 50ms update rate = 20Hz

float prev_error = 0.0f;
float integral = 0.0f;

DBUF distance_buffer;

char uart_buf[100] = {'\0'};

#define TRIG_LOW_US          3
#define TRIG_HIGH_US         10
#define MIN_SERVO_PW         250  // 0.5ms duty cycle, 0 degrees
#define MAX_SERVO_PW         1250 // 2.5s  duty cycle, 180 degrees
#define MIN_ROTATION_DEGREES 0
#define MAX_ROTATION_DEGREES 180  // Max angle a servo can rotate
#define HCSR04_MIN_DIST_CM   3
#define HCSR04_MAX_DIST_CM   34

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(capture_idx == 0){
        edge_1_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_idx = 1;
    } else {
        edge_2_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_idx = 0;
        uint32_t edge_diff = (edge_2_time >= edge_1_time) ?
                             (edge_2_time - edge_1_time) :
                             ((htim3.Init.Period - edge_1_time) + edge_2_time);

        float distance_cm = ((float)edge_diff) * (SPEED_OF_SOUND_CM_US / 2.0f);
        if(distance_cm > HCSR04_MAX_DIST_CM) distance_cm = HCSR04_MAX_DIST_CM;
        if(distance_cm < HCSR04_MIN_DIST_CM) distance_cm = HCSR04_MIN_DIST_CM;
        latest_distance = distance_cm;

        distance_ready = 1;
    }
}

/* Initialize DWT for microsecond delays */
void HCSR04_TRIG_DWT_init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Microsecond delay using DWT */
void HCSR04_TRIG_DWTDelay_us(uint32_t us) {
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void HCSR04_trigger_pulse(void){
    // Trigger pulse
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    HCSR04_TRIG_DWTDelay_us(TRIG_LOW_US);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    HCSR04_TRIG_DWTDelay_us(TRIG_HIGH_US);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

uint32_t SERVO_set_angle(uint8_t angle){
	uint32_t pulse_width = SERVO_calculate_pulse_width(angle);
	htim4.Instance->CCR1 = pulse_width;
	return pulse_width;
}

// Prototyping function
uint8_t SERVO_map_distance_to_angle(float prec){
	return (uint8_t)((prec-HCSR04_MIN_DIST_CM)/
		    (HCSR04_MAX_DIST_CM - HCSR04_MIN_DIST_CM)*
			(MAX_ROTATION_DEGREES-MIN_ROTATION_DEGREES))+
			 MIN_ROTATION_DEGREES;
}

uint32_t SERVO_calculate_pulse_width(uint8_t angle){
	return MIN_SERVO_PW + (((MAX_SERVO_PW - MIN_SERVO_PW) * angle)/MAX_ROTATION_DEGREES);
}

/* Helper to send float distance over UART */
void UART_print_distance(float distance) {
    if(distance < 0) snprintf(uart_buf, sizeof(uart_buf), "Distance: ERR\r\n");
    else snprintf(uart_buf, sizeof(uart_buf), "Distance: %.1f cm\r\n", distance);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

void UART_print_average(void) {
	float avg = DBUF_average(&distance_buffer);
    if(avg < 0) snprintf(uart_buf, sizeof(uart_buf), "Average: ERR\r\n");
    else snprintf(uart_buf, sizeof(uart_buf), "Average: %.1f cm\r\n", avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

uint32_t PID_proportional(float measured_distance){
	float error = measured_distance - setpoint_cm;

	if(error < 2 && error > -2) error = 0;

    float P = Kp * error;

    integral += error * dt;

    if(integral > 50.0f) integral = 50.0f;
    if(integral < -50.0f) integral = -50.0f;

    float I = Ki * integral;

    float derivative = (error - prev_error) / dt;
    float D = Kd * derivative;

    float output = P + I + D;
    float angle = 90.0f + output;

    prev_error = error;

    if(angle < MIN_ROTATION_DEGREES) angle = MIN_ROTATION_DEGREES;
    if(angle > MAX_ROTATION_DEGREES) angle = MAX_ROTATION_DEGREES;

//    SERVO_set_angle((uint8_t)angle);
    snprintf(uart_buf, sizeof(uart_buf), "dist: %.2f error: %.2f output: %.2f p: %.2f i: %.2f d: %.2f angle: %.2f \r\n", measured_distance, error, output, P, I, D, angle);
//    snprintf(uart_buf, sizeof(uart_buf), "dist: %.2f error: %.2f output: %.2f angle: %.2f \r\n", measured_distance, error, output, angle);
//    snprintf(uart_buf, sizeof(uart_buf), "error: %.5f output: %.5f angle: %.5f \r\n", error, output, angle);

    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    return (uint32_t)angle;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // Input capture on ECHO
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   // PWM for servo
  HCSR04_TRIG_DWT_init();                     // us delay for TRIG pulse
  DBUF_init(&distance_buffer);                // Ring buffer for distance readings

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  SERVO_set_angle(90);

  uint32_t last_trigger = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	uint32_t now = HAL_GetTick();
	if (now - last_trigger >= 50) {   // 50 ms interval
	    HCSR04_trigger_pulse();       // just sends 10 µs pulse
	    last_trigger = now;
	}

	if (distance_ready) {
	    distance_ready = 0;
	    DBUF_add(&distance_buffer, latest_distance);

	    float current = DBUF_average(&distance_buffer);
//	    float current = latest_distance;

	    uint32_t action = PID_proportional(current);

	    SERVO_set_angle(action);
//	    UART_print_distance((float)action);





//	    uint8_t angle = SERVO_map_distance_to_angle(desired_prec);
//	    SERVO_set_angle(angle);



//	    UART_print_average();

	}

      // 1/seconds = hertz
      // 1/0.05 seconds = 20hz - iterate while loop 20 times per sec
//      HAL_Delay(50); // Sensor measurement interval ~20Hz

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 168-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = LED_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
