/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_FULL_SCALE          4095.0f
#define ADC_REFERENCE_VOLTAGE   3.3f
#define VOLTAGE_DIVIDER_RATIO   30.0f

#define VOUT_TARGET_V           12.0f
#define VOUT_OVERVOLTAGE_V      13.5f

#define PWM_PERIOD_COUNTS       720.0f
#define DUTY_MIN                0.00f
#define DUTY_MAX                0.60f
#define DUTY_START              0.00f

/* The voltage loop runs at 1 kHz. Tune these values on the real hardware. */
#define VOLTAGE_KP              0.015f
#define VOLTAGE_KI              0.40f
#define CONTROL_PERIOD_S        0.001f
#define SOFT_START_STEP_V       0.024f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc_raw[4];
static float vin_measured;
static float vout_measured;
static float vout_reference;
static float voltage_integrator;
static float duty_command = DUTY_START;
static uint8_t buck_fault;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t ADC_ReadSequence(uint16_t *result);
static void Buck_SetDuty(float duty);
static void Buck_Stop(void);
static void Buck_VoltageControl(void);
static void OLED_UpdateStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float Clamp(float value, float lower, float upper)
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

static uint8_t ADC_ReadSequence(uint16_t *result)
{
  uint8_t index;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  for (index = 0U; index < 4U; index++)
  {
    if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }
    result[index] = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  (void)HAL_ADC_Stop(&hadc1);
  return 1U;
}

static void Buck_SetDuty(float duty)
{
  uint32_t compare;

  duty = Clamp(duty, DUTY_MIN, DUTY_MAX);
  compare = (uint32_t)(duty * PWM_PERIOD_COUNTS + 0.5f);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

static void Buck_Stop(void)
{
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
}

static void Buck_VoltageControl(void)
{
  float error;
  float duty_feedforward;

  if (ADC_ReadSequence(adc_raw) == 0U)
  {
    buck_fault = 1U;
    Buck_Stop();
    return;
  }

  /* PA0 and PA1 are PADOUT1 and PADOUT2, each divided by about 30. */
  vin_measured = ((float)adc_raw[0] * ADC_REFERENCE_VOLTAGE /
                  ADC_FULL_SCALE) * VOLTAGE_DIVIDER_RATIO;
  vout_measured = ((float)adc_raw[1] * ADC_REFERENCE_VOLTAGE /
                   ADC_FULL_SCALE) * VOLTAGE_DIVIDER_RATIO;

  if (vout_measured > VOUT_OVERVOLTAGE_V)
  {
    buck_fault = 1U;
    Buck_Stop();
    return;
  }

  if (vout_reference < VOUT_TARGET_V)
  {
    vout_reference += SOFT_START_STEP_V;
    if (vout_reference > VOUT_TARGET_V)
    {
      vout_reference = VOUT_TARGET_V;
    }
  }

  if (vin_measured < 1.0f)
  {
    buck_fault = 1U;
    Buck_Stop();
    return;
  }

  error = vout_reference - vout_measured;
  voltage_integrator += VOLTAGE_KI * error * CONTROL_PERIOD_S;
  voltage_integrator = Clamp(voltage_integrator, -0.20f, 0.20f);

  duty_feedforward = vout_reference / vin_measured;
  duty_command = duty_feedforward + VOLTAGE_KP * error + voltage_integrator;
  duty_command = Clamp(duty_command, DUTY_MIN, DUTY_MAX);
  Buck_SetDuty(duty_command);
}

static void OLED_UpdateStatus(void)
{
  OLED_ShowString(0, 0, (uint8_t *)"VIN:", 16);
  OLED_ShowFolatNum(40, 0, vin_measured, 4, 16);
  OLED_ShowString(80, 0, (uint8_t *)"V", 16);
  OLED_ShowString(0, 2, (uint8_t *)"VOUT:", 16);
  OLED_ShowFolatNum(48, 2, vout_measured, 4, 16);
  OLED_ShowString(88, 2, (uint8_t *)"V", 16);
  OLED_ShowString(0, 4, (uint8_t *)"DUTY:", 16);
  OLED_ShowFolatNum(48, 4, duty_command * 100.0f, 4, 16);
  OLED_ShowString(88, 4, (uint8_t *)"%", 16);
  OLED_ShowString(0, 6, (uint8_t *)(buck_fault ? "FAULT" : "VLOOP"), 16);
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	{
	  Error_Handler();
	}
	Buck_SetDuty(DUTY_START);
	OLED_Clear(0x00);
	OLED_UpdateStatus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t control_tick;
    static uint32_t oled_tick;
    uint32_t now = HAL_GetTick();

    if ((now - control_tick) >= 1U)
    {
      control_tick = now;
      if (buck_fault == 0U)
      {
        Buck_VoltageControl();
      }
    }

    if ((now - oled_tick) >= 200U)
    {
      oled_tick = now;
      OLED_UpdateStatus();
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
