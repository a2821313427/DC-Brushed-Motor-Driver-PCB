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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ?????? */
typedef enum
{
  MOTOR_STOPPED = 0,
  MOTOR_FORWARD,
  MOTOR_REVERSE
} MotorState_t;

/* ??????:A/B/C */
typedef enum
{
  SPEED_A = 0,   // 30%
  SPEED_B,       // 50%
  SPEED_C        // 70%
} SpeedLevel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ??:PB4~PB7,active low */
#define BUTTON_DIR_PIN       GPIO_PIN_4   // PB4: ????
#define BUTTON_SPEED_UP_PIN  GPIO_PIN_5   // PB5: ??(??)
#define BUTTON_SPEED_DN_PIN  GPIO_PIN_6   // PB6: ??(??)
#define BUTTON_STRT_STP_PIN  GPIO_PIN_7   // PB7: ??/??

/* LED(???3.3V,MCU????????) */
#define LED1_GPIO_Port       GPIOA
#define LED1_Pin             GPIO_PIN_15  // LED1: ????
#define LED2_GPIO_Port       GPIOB
#define LED2_Pin             GPIO_PIN_3   // LED2: ????

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

MotorState_t g_motorState = MOTOR_STOPPED;
SpeedLevel_t g_speedLevel = SPEED_B;  // ?? B ? 50%

uint32_t g_pwmMax   = 0;      // PWM ????(ARR)
uint32_t g_pwmValue = 0;      // ?? PWM ?????????

/* LED ????(?? HAL_GetTick() ?????) */
static uint32_t g_led1LastTick = 0;
static uint8_t  g_led1On       = 0;

static uint32_t g_led2LastTick = 0;
static uint8_t  g_led2On       = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_UpdateSpeed(void);
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Stop(void);
void Update_LEDs(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /* ?? PWM ???(TIM2/TIM3 ? ARR) */
  g_pwmMax = __HAL_TIM_GET_AUTORELOAD(&htim2);
  if (g_pwmMax == 0) g_pwmMax = 2132;   // ??:?????0,????? 2132

  /* ?????? B(50%) ??????? */
  Motor_UpdateSpeed();

  /* ?????????,PB7 ???? */
  Motor_Stop();

  /* LED ????(??:?????,?????????) */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ???? active low(??=GPIO_PIN_RESET) */

    /* PB4: ????(????????????,????????,???????????) */
    if (HAL_GPIO_ReadPin(GPIOB, BUTTON_DIR_PIN) == GPIO_PIN_RESET)
    {
      if (g_motorState == MOTOR_FORWARD)
      {
        Motor_Reverse();
      }
      else if (g_motorState == MOTOR_REVERSE)
      {
        Motor_Forward();
      }
      /* ?????????,???????? */

      HAL_Delay(200);   // ????
    }

    /* PB5: ??(??:A -> B -> C,? C ????) */
    if (HAL_GPIO_ReadPin(GPIOB, BUTTON_SPEED_UP_PIN) == GPIO_PIN_RESET)
    {
      if (g_speedLevel < SPEED_C)
      {
        g_speedLevel++;
        Motor_UpdateSpeed();   // ?????
      }
      /* ??? C ??,????? */

      HAL_Delay(200);
    }

    /* PB6: ??(??:C -> B -> A,? A ????) */
    if (HAL_GPIO_ReadPin(GPIOB, BUTTON_SPEED_DN_PIN) == GPIO_PIN_RESET)
    {
      if (g_speedLevel > SPEED_A)
      {
        g_speedLevel--;
        Motor_UpdateSpeed();
      }
      /* ??? A ??,????? */

      HAL_Delay(200);
    }

    /* PB7: ??/?? */
    if (HAL_GPIO_ReadPin(GPIOB, BUTTON_STRT_STP_PIN) == GPIO_PIN_RESET)
    {
      if (g_motorState == MOTOR_STOPPED)
      {
        /* ??? -> ????? */
        Motor_Forward();
      }
      else
      {
        /* ??/?? -> ?? */
        Motor_Stop();
      }

      HAL_Delay(200);
    }

    /* LED ?????????????? */
    Update_LEDs();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ??????????? */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* ???????????,???????? CCR */
void Motor_UpdateSpeed(void)
{
  /* ???????:A=30%,B=50%,C=70% */
  switch (g_speedLevel)
  {
    case SPEED_A:
      g_pwmValue = g_pwmMax * 3 / 10;  // 30%
      break;
    case SPEED_B:
      g_pwmValue = g_pwmMax * 5 / 10;  // 50%
      break;
    case SPEED_C:
      g_pwmValue = g_pwmMax * 7 / 10;  // 70%
      break;
    default:
      g_pwmValue = g_pwmMax * 5 / 10;
      break;
  }

  if (g_motorState == MOTOR_FORWARD)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, g_pwmValue);  // TIM2_CH4 -> PB11
  }
  else if (g_motorState == MOTOR_REVERSE)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, g_pwmValue);  // TIM3_CH4 -> PB1
  }
  /* ??????? CCR ????,???????? */
}

/* ??:TIM2_CH4 ?? PWM,PB10=?,PB0=? */
void Motor_Forward(void)
{
  /* ????? PWM,?? MOS,?????? */
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(1);   // ????

  /* ????:PB10 = 1, PB0 = 0 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);

  /* ?? TIM2_CH4 PWM(PB11) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  g_motorState = MOTOR_FORWARD;
  Motor_UpdateSpeed();
}

/* ??:TIM3_CH4 ?? PWM,PB0=?,PB10=? */
void Motor_Reverse(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(1);

  /* ????:PB0 = 1, PB10 = 0 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /* ?? TIM3_CH4 PWM(PB1) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  g_motorState = MOTOR_REVERSE;
  Motor_UpdateSpeed();
}

/* ??:?? PWM,??/??(?? H ???) */
void Motor_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  g_motorState = MOTOR_STOPPED;
}

/* ??????????? LED ??
 * ??:LED ??? 3.3V,???????
 * A ?:?1s?1s
 * B ?:?0.5s?0.5s
 * C ?:??(???)
 */
void Update_LEDs(void)
{
  uint32_t now = HAL_GetTick();

  /* ????(???) */
  GPIO_PinState led1State = GPIO_PIN_SET;
  GPIO_PinState led2State = GPIO_PIN_SET;

  if (g_motorState == MOTOR_FORWARD)
  {
    /* ??,??? LED1,LED2 ??? */
    uint32_t interval_ms = 0;

    if (g_speedLevel == SPEED_A)
    {
      interval_ms = 1000; // 1s
    }
    else if (g_speedLevel == SPEED_B)
    {
      interval_ms = 500;  // 0.5s
    }
    else /* SPEED_C */
    {
      /* C ?:??,??? */
      g_led1On = 1;
    }

    if (g_speedLevel == SPEED_A || g_speedLevel == SPEED_B)
    {
      if (now - g_led1LastTick >= interval_ms)
      {
        g_led1LastTick = now;
        g_led1On = !g_led1On;
      }
    }

    led1State = (g_led1On ? GPIO_PIN_RESET : GPIO_PIN_SET); // ?????
    led2State = GPIO_PIN_SET; // ?? LED ??
  }
  else if (g_motorState == MOTOR_REVERSE)
  {
    /* ??,??? LED2,LED1 ??? */
    uint32_t interval_ms = 0;

    if (g_speedLevel == SPEED_A)
    {
      interval_ms = 1000; // 1s
    }
    else if (g_speedLevel == SPEED_B)
    {
      interval_ms = 500;  // 0.5s
    }
    else /* SPEED_C */
    {
      /* C ?:?? */
      g_led2On = 1;
    }

    if (g_speedLevel == SPEED_A || g_speedLevel == SPEED_B)
    {
      if (now - g_led2LastTick >= interval_ms)
      {
        g_led2LastTick = now;
        g_led2On = !g_led2On;
      }
    }

    led2State = (g_led2On ? GPIO_PIN_RESET : GPIO_PIN_SET);
    led1State = GPIO_PIN_SET; // ?? LED ??
  }
  else
  {
    /* ??:?? LED ??,?????? */
    g_led1On = 0;
    g_led2On = 0;
    g_led1LastTick = now;
    g_led2LastTick = now;

    led1State = GPIO_PIN_SET;
    led2State = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led1State);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led2State);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* User can add his own implementation to report the file name and line number */
}
#endif /* USE_FULL_ASSERT */
