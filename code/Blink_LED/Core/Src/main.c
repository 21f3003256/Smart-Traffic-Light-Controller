/**
 * 4-Way Traffic Management System — NUCLEO-F446RE
 * (Final build, 7-segment display omitted)
 *
 * Subsystems:
 *   - 4 traffic lights (R/Y/G per direction)            : 12 GPIO
 *   - 4 pedestrian push buttons (active HIGH, EXTI)     :  4 GPIO
 *   - 8 HC-SR04 sensors (4 dir x 2 near/far, 1 TRIG)    :  9 GPIO
 *
 * Cycle: N -> E -> S -> W -> N ...
 * Per direction: RED (sample sensors) -> GREEN -> YELLOW -> next direction.
 * Other directions are held at RED throughout.
 *
 * Green duration adapts to sensor occupancy:
 *   0 blocked -> 5 s   1 blocked -> 7 s   2 blocked -> 9 s
 * Pressing a direction's pedestrian button before its RED phase extends
 * that RED by PED_EXTEND_MS (extra crossing time).
 *
 * Sensor reading uses median-of-3 with 60 ms settle delay between firings
 * to reject single-shot spikes and acoustic crosstalk from the shared TRIG.
 */

#include "main.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);

/* ============================================================ */
/*  Directions / timing                                         */
/* ============================================================ */
#define DIR_N    0
#define DIR_E    1
#define DIR_S    2
#define DIR_W    3
#define NUM_DIRS 4

#define GREEN_NORMAL_MS  2000u    /* 0 blocked -> 2 blinks (car-free)   */
#define GREEN_LOW_MS     5000u    /* 1 blocked -> 5 blinks (light)      */
#define GREEN_HEAVY_MS  10000u    /* 2 blocked -> 10 blinks (heavy)     */
#define YELLOW_MS        3000u
#define RED_MS           2000u
#define PED_EXTEND_MS    3000u

#define SENSOR_TIMEOUT_US   30000u
#define SENSOR_SETTLE_MS    60u
#define NOISE_FLOOR_CM      2u   /* readings below this are treated as noise */

/* Per-direction trigger distance (cm). */
static const uint16_t SENSOR_THRESHOLD_CM[NUM_DIRS] = {
    3,   /* N */
    3,   /* E */
    3,   /* S */
    3,   /* W */
};

/* ============================================================ */
/*  Pin map                                                     */
/* ============================================================ */
typedef struct { GPIO_TypeDef *port; uint16_t pin; } GpioPin;

static const GpioPin LED_RED[NUM_DIRS] = {
    { GPIOA, GPIO_PIN_7  },
    { GPIOB, GPIO_PIN_6  },
    { GPIOA, GPIO_PIN_8  },
    { GPIOB, GPIO_PIN_5  },
};
static const GpioPin LED_YEL[NUM_DIRS] = {
    { GPIOA, GPIO_PIN_6  },
    { GPIOC, GPIO_PIN_7  },
    { GPIOB, GPIO_PIN_10 },
    { GPIOB, GPIO_PIN_3  },
};
static const GpioPin LED_GRN[NUM_DIRS] = {
    { GPIOA, GPIO_PIN_5  },
    { GPIOA, GPIO_PIN_9  },
    { GPIOB, GPIO_PIN_4  },
    { GPIOA, GPIO_PIN_10 },
};

#define TRIG_PORT   GPIOB
#define TRIG_PIN    GPIO_PIN_0

static const GpioPin ECHO_NEAR[NUM_DIRS] = {
    { GPIOB, GPIO_PIN_7  },
    { GPIOB, GPIO_PIN_12 },
    { GPIOB, GPIO_PIN_14 },
    { GPIOA, GPIO_PIN_11 },
};
static const GpioPin ECHO_FAR[NUM_DIRS] = {
    { GPIOC, GPIO_PIN_12 },
    { GPIOB, GPIO_PIN_13 },
    { GPIOB, GPIO_PIN_15 },
    { GPIOA, GPIO_PIN_12 },
};

/* Buttons remapped so each physical button controls its own direction
 * (N<->S and E<->W swapped relative to pinlist rev 2). */
static const GpioPin BTN[NUM_DIRS] = {
    { GPIOA, GPIO_PIN_4 },   /* N - EXTI4   (was S pin) */
    { GPIOB, GPIO_PIN_8 },   /* E - EXTI9_5 (was W pin) */
    { GPIOA, GPIO_PIN_0 },   /* S - EXTI0   (was N pin) */
    { GPIOA, GPIO_PIN_1 },   /* W - EXTI1   (was E pin) */
};
static volatile uint8_t ped_pending[NUM_DIRS] = {0};

/* ============================================================ */
/*  Microsecond timer (DWT cycle counter)                       */
/* ============================================================ */
static uint32_t cycles_per_us;

static inline uint32_t micros(void) { return DWT->CYCCNT / cycles_per_us; }
static inline void delay_us(uint32_t us) {
    uint32_t s = DWT->CYCCNT, t = us * cycles_per_us;
    while ((DWT->CYCCNT - s) < t) { }
}
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us     = HAL_RCC_GetHCLKFreq() / 1000000u;
}

/* ============================================================ */
/*  GPIO helpers                                                */
/* ============================================================ */
static inline void pin_write(GpioPin p, GPIO_PinState s) {
    HAL_GPIO_WritePin(p.port, p.pin, s);
}
static inline GPIO_PinState pin_read(GpioPin p) {
    return HAL_GPIO_ReadPin(p.port, p.pin);
}
static void gpio_out(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pin; g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
static void gpio_in_pulldown(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pin; g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}
static void gpio_exti_rising(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pin; g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

/* ============================================================ */
/*  Traffic LED helpers                                         */
/* ============================================================ */
typedef enum { LIGHT_RED = 0, LIGHT_YELLOW, LIGHT_GREEN } LightColor;

static void set_light(uint8_t dir, LightColor c) {
    pin_write(LED_RED[dir], c == LIGHT_RED    ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pin_write(LED_YEL[dir], c == LIGHT_YELLOW ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pin_write(LED_GRN[dir], c == LIGHT_GREEN  ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void all_red(void) {
    for (uint8_t d = 0; d < NUM_DIRS; d++) set_light(d, LIGHT_RED);
}

/* ============================================================ */
/*  HC-SR04: single ECHO measurement, wraparound-safe           */
/* ============================================================ */
static uint16_t read_one_echo(GpioPin echo)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(11);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    uint32_t start = micros();
    while (pin_read(echo) == GPIO_PIN_RESET) {
        if ((micros() - start) > SENSOR_TIMEOUT_US) return 0;
    }
    uint32_t t0 = micros();
    while (pin_read(echo) == GPIO_PIN_SET) {
        if ((micros() - t0) > SENSOR_TIMEOUT_US) return 0;
    }
    return (uint16_t)((micros() - t0) / 58u);
}

/* Median of 3 readings — rejects single-shot spikes. */
static uint16_t read_echo_median(GpioPin echo)
{
    uint16_t a = read_one_echo(echo);
    HAL_Delay(2);
    uint16_t b = read_one_echo(echo);
    HAL_Delay(2);
    uint16_t c = read_one_echo(echo);

    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

/* Returns count of sensors reading inside the lane threshold (0..2). */
static uint8_t read_sensor_pair(uint8_t dir)
{
    uint16_t thr = SENSOR_THRESHOLD_CM[dir];

    uint16_t near_cm = read_echo_median(ECHO_NEAR[dir]);
    HAL_Delay(SENSOR_SETTLE_MS);
    uint16_t far_cm  = read_echo_median(ECHO_FAR[dir]);
    HAL_Delay(SENSOR_SETTLE_MS);

    uint8_t blocked = 0;
    if (near_cm >= NOISE_FLOOR_CM && near_cm < thr) blocked++;
    if (far_cm  >= NOISE_FLOOR_CM && far_cm  < thr) blocked++;
    return blocked;
}

static uint32_t green_duration(uint8_t blocked) {
    switch (blocked) {
        case 0:  return GREEN_NORMAL_MS;
        case 1:  return GREEN_LOW_MS;
        default: return GREEN_HEAVY_MS;
    }
}

/* ============================================================ */
/*  Init                                                        */
/* ============================================================ */
static void GPIO_Init_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    for (uint8_t d = 0; d < NUM_DIRS; d++) {
        gpio_out(LED_RED[d].port, LED_RED[d].pin);
        gpio_out(LED_YEL[d].port, LED_YEL[d].pin);
        gpio_out(LED_GRN[d].port, LED_GRN[d].pin);
    }

    gpio_out(TRIG_PORT, TRIG_PIN);
    for (uint8_t d = 0; d < NUM_DIRS; d++) {
        gpio_in_pulldown(ECHO_NEAR[d].port, ECHO_NEAR[d].pin);
        gpio_in_pulldown(ECHO_FAR[d].port,  ECHO_FAR[d].pin);
    }

    for (uint8_t d = 0; d < NUM_DIRS; d++) {
        gpio_exti_rising(BTN[d].port, BTN[d].pin);
    }
    HAL_NVIC_SetPriority(EXTI0_IRQn,   5, 0);
    HAL_NVIC_SetPriority(EXTI1_IRQn,   5, 0);
    HAL_NVIC_SetPriority(EXTI4_IRQn,   5, 0);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ============================================================ */
/*  Main state machine                                          */
/* ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    DWT_Init();
    GPIO_Init_All();

    all_red();

    uint8_t dir = DIR_N;

    while (1)
    {
        /* ---- RED phase: sample sensors, honor pedestrian extend ---- */
        all_red();

        /* Cross-direction control: sensors on the opposite lane decide
         * this lane's green duration (N<->S, E<->W). */
        uint8_t sense_dir = (dir + 2u) % NUM_DIRS;
        uint8_t blocked   = read_sensor_pair(sense_dir);
        uint32_t green_ms = green_duration(blocked);

        /* Pedestrian sequence:
         *   Phase 1 (2 s): YELLOW blink   -> request acknowledged
         *   Phase 2 (2 s): RED    blink   -> pedestrian walk window
         * RED gap then runs normally before GREEN. */
        if (ped_pending[dir]) {
            ped_pending[dir] = 0;

            pin_write(LED_RED[dir], GPIO_PIN_RESET);
            uint32_t t = HAL_GetTick();
            while ((HAL_GetTick() - t) < 2000u) {
                pin_write(LED_YEL[dir], GPIO_PIN_SET);
                HAL_Delay(250);
                pin_write(LED_YEL[dir], GPIO_PIN_RESET);
                HAL_Delay(250);
            }

            t = HAL_GetTick();
            while ((HAL_GetTick() - t) < 2000u) {
                pin_write(LED_RED[dir], GPIO_PIN_SET);
                HAL_Delay(250);
                pin_write(LED_RED[dir], GPIO_PIN_RESET);
                HAL_Delay(250);
            }
            pin_write(LED_RED[dir], GPIO_PIN_SET);   /* steady red for the gap */
        }

        uint32_t t_start = HAL_GetTick();
        while ((HAL_GetTick() - t_start) < RED_MS) { }

        /* ---- GREEN phase ----
         * Blink GREEN at 1 Hz for the whole phase. Count the blinks:
         *   5 = normal, 7 = low traffic, 9 = heavy traffic. */
        pin_write(LED_RED[dir], GPIO_PIN_RESET);
        pin_write(LED_YEL[dir], GPIO_PIN_RESET);
        uint32_t blinks = green_ms / 1000u;
        for (uint32_t i = 0; i < blinks; i++) {
            pin_write(LED_GRN[dir], GPIO_PIN_SET);
            HAL_Delay(500);
            pin_write(LED_GRN[dir], GPIO_PIN_RESET);
            HAL_Delay(500);
        }

        /* ---- YELLOW phase ---- */
        set_light(dir, LIGHT_YELLOW);
        t_start = HAL_GetTick();
        while ((HAL_GetTick() - t_start) < YELLOW_MS) { }

        dir = (dir + 1u) % NUM_DIRS;
    }
}

/* ============================================================ */
/*  Interrupt handlers                                          */
/* ============================================================ */
static void handle_button(uint16_t pin)
{
    for (uint8_t d = 0; d < NUM_DIRS; d++) {
        if (BTN[d].pin == pin) { ped_pending[d] = 1; return; }
    }
}

void EXTI0_IRQHandler(void)   { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI1_IRQHandler(void)   { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }
void EXTI4_IRQHandler(void)   { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }
void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8); }

void HAL_GPIO_EXTI_Callback(uint16_t pin) { handle_button(pin); }

/* ============================================================ */
/*  Clocks / fault handler                                      */
/* ============================================================ */
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

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
