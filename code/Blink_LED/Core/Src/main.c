/**
 * Two HC-SR04 Sensors + Traffic Light + 7-Segment Countdown Display
 * Timer-based state machine using TIM2 interrupt
 * 7-Segment synced directly to TIM2 counter — no TIM3 needed
 * Target: STM32 Cortex-M4 (NUCLEO-F446RE)
 *
 * Wiring:
 *   Sensor 1 (Near): TRIG -> PA10, ECHO -> PA9
 *   Sensor 2 (Far):  TRIG -> PA8,  ECHO -> PB5
 *
 *   Traffic LEDs:
 *   RED LED    -> PA7
 *   YELLOW LED -> PA6
 *   GREEN LED  -> PA5
 *
 *   7-Segment (common cathode):
 *   SEG_A -> PC0   (fill in your pin)
 *   SEG_B -> PC1   (fill in your pin)
 *   SEG_C -> PC2   (fill in your pin)
 *   SEG_D -> PC3   (fill in your pin)
 *   SEG_E -> PC4   (fill in your pin)
 *   SEG_F -> PC5   (fill in your pin)
 *   SEG_G -> PC6   (fill in your pin)
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

void SystemClock_Config(void);

/* ── Sensor 1 pins (near) ── */
#define TRIG1_PIN    GPIO_PIN_10
#define TRIG1_PORT   GPIOA
#define ECHO1_PIN    GPIO_PIN_9
#define ECHO1_PORT   GPIOA

/* ── Sensor 2 pins (far) ── */
#define TRIG2_PIN    GPIO_PIN_8
#define TRIG2_PORT   GPIOA
#define ECHO2_PIN    GPIO_PIN_5
#define ECHO2_PORT   GPIOB

/* ── Traffic LED pins ── */
#define RED_PIN      GPIO_PIN_7
#define YELLOW_PIN   GPIO_PIN_6
#define GREEN_PIN    GPIO_PIN_5
#define LED_PORT     GPIOA

/* ── 7-Segment pins — fill in your port and pins ── */
#define SEG_PORT     GPIOC
#define SEG_A        GPIO_PIN_9
#define SEG_B        GPIO_PIN_8
#define SEG_C        GPIO_PIN_6
#define SEG_D        GPIO_PIN_5
#define SEG_E        GPIO_PIN_7
#define SEG_F        GPIO_PIN_4
#define SEG_G        GPIO_PIN_11
#define SEG_ALL      (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G)

/* ── Detection thresholds (cm) ── */
#define SENSOR1_THRESHOLD   10
#define SENSOR2_THRESHOLD   10

/* ── Green light durations (ms) ── */
#define GREEN_NORMAL        5000    /* No traffic:    5 seconds */
#define GREEN_LOW           7000    /* Low traffic:   7 seconds */
#define GREEN_HEAVY         9000    /* Heavy traffic: 9 seconds */

/* ── Transition durations (ms) ── */
#define YELLOW_DURATION     3000    /* Yellow: 3 seconds */
#define RED_DURATION        4000    /* Red:    2 seconds */

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
volatile uint8_t    timer_expired = 0;
volatile uint32_t   current_phase_ms = 0;   /* Total ms of current phase  */
TrafficState        traffic = NO_TRAFFIC;

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
   TIM2 — light phase timer (one-shot)
   ──────────────────────────────────────────── */
void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = (HAL_RCC_GetHCLKFreq() / 100) - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFF;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void Timer_Start(uint32_t duration_ms)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, duration_ms - 1);
    current_phase_ms = duration_ms;       /* Save total duration           */
    timer_expired    = 0;
    HAL_TIM_Base_Start_IT(&htim2);
}

/* ── TIM2 IRQ handler ── */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HAL_TIM_Base_Stop_IT(&htim2);
        timer_expired = 1;
    }
}

/* ────────────────────────────────────────────
   GPIO init
   ──────────────────────────────────────────── */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

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

    /* PA8 — Sensor 2 TRIG */
    gpio.Pin   = TRIG2_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG2_PORT, &gpio);

    /* PB5 — Sensor 2 ECHO */
    gpio.Pin  = ECHO2_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECHO2_PORT, &gpio);

    /* PA5, PA6, PA7 — GREEN, YELLOW, RED LEDs */
    gpio.Pin   = RED_PIN | YELLOW_PIN | GREEN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    /* 7-Segment pins */
    gpio.Pin   = SEG_ALL;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SEG_PORT, &gpio);

    /* Debug indicator PB6 PB8 */
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* All outputs start LOW */
    HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT,   RED_PIN | YELLOW_PIN | GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_PORT,   SEG_ALL, GPIO_PIN_RESET);

    DWT_Init();
}

/* ────────────────────────────────────────────
   7-Segment digit encoding (common cathode)
   ──────────────────────────────────────────── */
void SEG_Display(uint8_t digit)
{
    HAL_GPIO_WritePin(SEG_PORT, SEG_ALL, GPIO_PIN_RESET);

    switch (digit) {
        case 0: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(SEG_PORT, SEG_B|SEG_C,                           GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,        GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,        GPIO_PIN_SET); break;
        case 4: HAL_GPIO_WritePin(SEG_PORT, SEG_B|SEG_C|SEG_F|SEG_G,              GPIO_PIN_SET); break;
        case 5: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,        GPIO_PIN_SET); break;
        case 6: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,  GPIO_PIN_SET); break;
        case 7: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_B|SEG_C,                     GPIO_PIN_SET); break;
        case 8: HAL_GPIO_WritePin(SEG_PORT, SEG_ALL,                               GPIO_PIN_SET); break;
        case 9: HAL_GPIO_WritePin(SEG_PORT, SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G,  GPIO_PIN_SET); break;
        default: HAL_GPIO_WritePin(SEG_PORT, SEG_ALL, GPIO_PIN_RESET); break;
    }
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
   ──────────────────────────────────────────── */
TrafficState ReadTraffic(void)
{
    uint32_t dist1 = Ultrasonic_ReadCm(TRIG1_PORT, TRIG1_PIN,
                                        ECHO1_PORT, ECHO1_PIN);
    HAL_Delay(60);

    uint32_t dist2 = Ultrasonic_ReadCm(TRIG2_PORT, TRIG2_PIN,
                                        ECHO2_PORT, ECHO2_PIN);

    uint8_t count = 0;
    if (dist1 < SENSOR1_THRESHOLD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        count++;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    if (dist2 < SENSOR2_THRESHOLD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        count++;
    } else {
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
   Get remaining seconds from TIM2 directly
   Perfectly synced — same timer as the light
   ──────────────────────────────────────────── */
uint8_t Get_Remaining_Seconds(void)
{
    uint32_t elapsed_ms   = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t remaining_ms = (current_phase_ms > elapsed_ms)
                            ? (current_phase_ms - elapsed_ms)
                            : 0;
    return (uint8_t)(remaining_ms / 1000);
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

    /* Start on RED */
    LED_Red();
    traffic = ReadTraffic();
    Timer_Start(RED_DURATION);

    uint8_t last_sec = 0xFF;    /* Track last displayed second to avoid
                                   redundant GPIO writes                   */

    while (1)
    {
        /* ── Update 7-seg display from TIM2 counter ── */
        uint8_t remaining = Get_Remaining_Seconds();
        if (remaining != last_sec) {
            last_sec = remaining;
            SEG_Display(remaining % 10);
        }

        /* ── TIM2 expired → advance light state ── */
        if (timer_expired)
        {
            timer_expired = 0;
            last_sec      = 0xFF;   /* Force display refresh on new phase  */

            if (current_light == LIGHT_RED)
            {
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
                current_light = LIGHT_YELLOW;
                LED_Yellow();
                Timer_Start(YELLOW_DURATION);
            }
            else if (current_light == LIGHT_YELLOW)
            {
                current_light = LIGHT_RED;
                LED_Red();
                traffic = ReadTraffic();
                Timer_Start(RED_DURATION);
            }
        }
    }
}

/* ────────────────────────────────────────────
   System Clock Configuration
   ──────────────────────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ────────────────────────────────────────────
   Error Handler
   ──────────────────────────────────────────── */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
