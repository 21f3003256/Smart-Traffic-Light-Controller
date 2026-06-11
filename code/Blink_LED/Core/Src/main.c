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

void SystemClock_Config(void);
/**
 * Two HC-SR04 Ultrasonic Sensors + Traffic Light LED Control
 * Timer-based state machine using TIM2 interrupt
 * Target: STM32 Cortex-M4 (NUCLEO-F446RE)
 *
 * Wiring:
 *   Sensor 1 (Near):  TRIG -> PA10,  ECHO -> PA9
 *   Sensor 2 (Far):   TRIG -> PA2,   ECHO -> PA3
 *
 *   RED LED    -> PB0
 *   YELLOW LED -> PB1
 *   GREEN LED  -> PB2
 *
 * How it works:
 *   TIM2 fires an interrupt after a set duration.
 *   Each interrupt advances the light to the next state.
 *   CPU is free between interrupts to read sensors.
 *
 * State machine:
 *   RED -> GREEN (duration based on traffic) -> YELLOW -> RED -> ...
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* ── Sensor 1 pins (near) ── */
#define TRIG1_PIN    GPIO_PIN_10
#define TRIG1_PORT   GPIOA
#define ECHO1_PIN    GPIO_PIN_9
#define ECHO1_PORT   GPIOA

/* ── Sensor 2 pins (far sensor) ── */
#define TRIG2_PIN    GPIO_PIN_4
#define TRIG2_PORT   GPIOB
#define ECHO2_PIN    GPIO_PIN_3
#define ECHO2_PORT   GPIOB

/* ── LED pins ── */
#define RED_PIN      GPIO_PIN_7
#define YELLOW_PIN   GPIO_PIN_6
#define GREEN_PIN    GPIO_PIN_5
#define LED_PORT     GPIOA

/* ── Detection thresholds ── */
#define SENSOR1_THRESHOLD   10    /* Near sensor: object within 150cm */
#define SENSOR2_THRESHOLD   10    /* Far sensor:  object within 300cm */

/* ── Green light durations (ms) ── */
#define GREEN_NORMAL        2000
#define GREEN_LOW           4000
#define GREEN_HEAVY         6000

/* ── Transition durations (ms) ── */
#define YELLOW_DURATION     2000
#define RED_DURATION        1000

/* ── Traffic states ── */
typedef enum {
    NO_TRAFFIC   = 0,
    LOW_TRAFFIC  = 1,
    HIGH_TRAFFIC = 2
} TrafficState;

/* ── Light states ── */
typedef enum {
    LIGHT_RED    = 0,
    LIGHT_GREEN  = 1,
    LIGHT_YELLOW = 2
} LightState;

/* ── Global variables ── */
TIM_HandleTypeDef htim2;
volatile LightState current_light = LIGHT_RED;
volatile uint8_t    timer_expired  = 0;
TrafficState        traffic        = NO_TRAFFIC;

/* ────────────────────────────────────────────
   DWT microsecond timer
   ──────────────────────────────────────────── */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ────────────────────────────────────────────
   TIM2 setup
   Prescaler set so timer ticks every 1ms.
   Period set to desired duration, then timer fires interrupt.
   ──────────────────────────────────────────── */
void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Tick every 1ms: prescaler = (SystemCoreClock / 1000) - 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = (HAL_RCC_GetHCLKFreq() / 1000) - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFF; /* Set properly before start */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* Start timer for a given duration in milliseconds */
void Timer_Start(uint32_t duration_ms)
{
    HAL_TIM_Base_Stop_IT(&htim2);        /* Stop any running timer first  */
    __HAL_TIM_SET_COUNTER(&htim2, 0);    /* Reset counter to 0            */
    __HAL_TIM_SET_AUTORELOAD(&htim2, duration_ms - 1); /* Set period      */
    timer_expired = 0;
    HAL_TIM_Base_Start_IT(&htim2);       /* Start, fires interrupt at end */
}

/* TIM2 interrupt handler — called when timer period elapses */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/* HAL timer period elapsed callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HAL_TIM_Base_Stop_IT(&htim2);  /* Stop timer, one-shot behavior  */
        timer_expired = 1;             /* Signal main loop                */
    }
}

/* ────────────────────────────────────────────
   GPIO init
   ──────────────────────────────────────────── */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PA10 — Sensor 1 TRIG */
    gpio.Pin   = TRIG1_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG1_PORT, &gpio);

    /* PA9 — Sensor 1 ECHO */
    gpio.Pin  = ECHO1_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECHO1_PORT, &gpio);

    /* PA2 — Sensor 2 TRIG */
    gpio.Pin   = TRIG2_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG2_PORT, &gpio);

    /* PA3 — Sensor 2 ECHO */
    gpio.Pin  = ECHO2_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECHO2_PORT, &gpio);

    /* PB0, PB1, PB2 — RED, YELLOW, GREEN LEDs */
    gpio.Pin   = RED_PIN | YELLOW_PIN | GREEN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    gpio.Pin   = GPIO_PIN_8 | GPIO_PIN_6;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* All TRIG pins LOW, all LEDs off */
    HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, RED_PIN | YELLOW_PIN | GREEN_PIN, GPIO_PIN_RESET);

    DWT_Init();
}

/* ────────────────────────────────────────────
   Ultrasonic read
   ──────────────────────────────────────────── */
uint32_t Ultrasonic_ReadCm(GPIO_TypeDef *trig_port, uint16_t trig_pin,
                            GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    uint32_t start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET) {
        if ((DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000U) > 30000)
            return 9999;
    }

    start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
        if ((DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000U) > 30000)
            return 9999;
    }

    uint32_t echo_us = (DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000U);
    return (echo_us * 343UL) / 20000UL;
}

/* ────────────────────────────────────────────
   Read both sensors, return traffic state
   Called during RED phase so data is ready
   before GREEN starts
   ──────────────────────────────────────────── */
TrafficState ReadTraffic(void)
{
    uint32_t dist1 = Ultrasonic_ReadCm(TRIG1_PORT, TRIG1_PIN,
                                        ECHO1_PORT, ECHO1_PIN);
    HAL_Delay(60);  /* 60ms gap between sensors to avoid crosstalk */

    uint32_t dist2 = Ultrasonic_ReadCm(TRIG2_PORT, TRIG2_PIN,
                                        ECHO2_PORT, ECHO2_PIN);

    uint8_t count = 0;
    if (dist1 < SENSOR1_THRESHOLD) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    	count++;
    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    if (dist2 < SENSOR2_THRESHOLD){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    	count++;
    }
    else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    if      (count == 2) return HIGH_TRAFFIC;
    else if (count == 1) return LOW_TRAFFIC;
    else                 return NO_TRAFFIC;
}

/* ────────────────────────────────────────────
   LED control
   ──────────────────────────────────────────── */
void LED_Red(void)
{
    HAL_GPIO_WritePin(LED_PORT, GREEN_PIN | YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, RED_PIN,                GPIO_PIN_SET);
}

void LED_Yellow(void)
{
    HAL_GPIO_WritePin(LED_PORT, GREEN_PIN | RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, YELLOW_PIN,          GPIO_PIN_SET);
}

void LED_Green(void)
{
    HAL_GPIO_WritePin(LED_PORT, RED_PIN | YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, GREEN_PIN,             GPIO_PIN_SET);
}

/* ────────────────────────────────────────────
   Main
   ──────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    TIM2_Init();

    /* Start on RED, read sensors immediately */
    LED_Red();
    traffic = ReadTraffic();
    Timer_Start(RED_DURATION);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);


    while (1)
    {
        /* Wait here until TIM2 fires */
        if (timer_expired)
        {
            timer_expired = 0;

            /* Advance to next light state */
            if (current_light == LIGHT_RED)
            {
                /* RED done — move to GREEN */
                /* Green duration depends on traffic read during RED */
                uint32_t green_time;
                if      (traffic == HIGH_TRAFFIC) green_time = GREEN_HEAVY;
                else if (traffic == LOW_TRAFFIC)  green_time = GREEN_LOW;
                else                              green_time = GREEN_NORMAL;

                current_light = LIGHT_GREEN;
                LED_Green();
                Timer_Start(green_time);
            }
            else if (current_light == LIGHT_GREEN)
            {
                /* GREEN done — move to YELLOW */
                current_light = LIGHT_YELLOW;
                LED_Yellow();
                Timer_Start(YELLOW_DURATION);
            }
            else if (current_light == LIGHT_YELLOW)
            {
                /* YELLOW done — move to RED */
                /* Read sensors NOW during RED so data is
                   ready for the next GREEN phase           */
                current_light = LIGHT_RED;
                LED_Red();
                traffic = ReadTraffic();   /* CPU free to do this! */
                Timer_Start(RED_DURATION);
            }
        }

        /* CPU is free here — add other tasks if needed */
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
