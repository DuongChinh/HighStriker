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
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "tm_stm32f4_mfrc522.h"
#include "HX711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} LED_PinTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WEIGHT_STABILITY_THRESHOLD  0.01f
#define WEIGHT_STABILITY_COUNT      5
#define MEASUREMENT_INTERVAL        100
#define DISPLAY_UPDATE_INTERVAL     500
#define WEIGHT_FILTER_ALPHA         0.8f
#define HX711_DT_PORT GPIOA
#define HX711_DT_PIN  GPIO_PIN_11
#define HX711_SCK_PORT GPIOA
#define HX711_SCK_PIN  GPIO_PIN_12
#define SCALE_FACTOR      20000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct {
    float current_weight;
    float previous_weight;
    uint8_t weight_stable_count;
    uint8_t system_ready;
    uint32_t last_measurement_time;
    uint32_t last_display_time;
    float weight_threshold;
} scale_state_t;

scale_state_t scale_state = {0};
hx711_t hx711;
char uart_buffer[128];
const LED_PinTypeDef LED_Pins[6] = {
    {GPIOE, GPIO_PIN_8},  // LED1 - PE8
    {GPIOE, GPIO_PIN_9},  // LED2 - PE9
    {GPIOE, GPIO_PIN_10}, // LED3 - PE10
    {GPIOE, GPIO_PIN_11}, // LED4 - PE11
    {GPIOE, GPIO_PIN_12}, // LED5 - PE12
    {GPIOE, GPIO_PIN_13}  // LED6 - PE13
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void scale_init(void);
void scale_calibrate(void);
float scale_read_weight(void);
float scale_filter_weight(float raw_weight);
uint8_t scale_is_weight_stable(float weight);
void scale_display_weight(float weight);
void scale_process_rfid(float weight);
void scale_send_uart_data(const char* format, ...);
void scale_handle_error(const char* error_msg);
void LED_Control(uint8_t led_num, uint8_t state);
void DisplayNumber(uint8_t num);
void BlinkLEDs(uint8_t times, uint16_t delay_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LED_Control(uint8_t led_num, uint8_t state) {
    if (led_num >= 1 && led_num <= 6) {
        HAL_GPIO_WritePin(LED_Pins[led_num-1].port,
                         LED_Pins[led_num-1].pin,
                         (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void DisplayNumber(uint8_t num) {
    if (num > 6) num = 6; // Giới hạn tối đa 6 LED

    for (int i = 0; i < 6; i++) {
        HAL_GPIO_WritePin(LED_Pins[i].port, LED_Pins[i].pin,
                         (i < num) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void BlinkLEDs(uint8_t times, uint16_t delay_ms) {
    for (uint8_t i = 0; i < times; i++) {
        // Bật tất cả LED
        for (int j = 0; j < 6; j++) {
            HAL_GPIO_WritePin(LED_Pins[j].port, LED_Pins[j].pin, GPIO_PIN_SET);
        }
        HAL_Delay(delay_ms);

        // Tắt tất cả LED
        for (int j = 0; j < 6; j++) {
            HAL_GPIO_WritePin(LED_Pins[j].port, LED_Pins[j].pin, GPIO_PIN_RESET);
        }
        HAL_Delay(delay_ms);
    }
}

void scale_init(void) {
    scale_send_uart_data("Initializing HX711 load cell...\r\n");
    hx711_init(&hx711, HX711_SCK_PORT, HX711_SCK_PIN, HX711_DT_PORT, HX711_DT_PIN);
    set_gain(&hx711, 128, 32);

    if (is_ready(&hx711)) {
        scale_send_uart_data("HX711 ready!\r\n");
    } else {
        scale_send_uart_data("HX711 not ready!\r\n");
    }

    set_scale(&hx711, SCALE_FACTOR, SCALE_FACTOR);
    scale_calibrate();

    scale_state.current_weight = 0.0f;
    scale_state.previous_weight = 0.0f;
    scale_state.weight_stable_count = 0;
    scale_state.weight_threshold = WEIGHT_STABILITY_THRESHOLD;
    scale_state.system_ready = 0;
}

void scale_calibrate(void) {
    scale_send_uart_data("Calibrating...\r\n");
    HAL_Delay(2000);
    tare_all(&hx711, 10);
    scale_send_uart_data("Calibration complete\r\n");
}

float scale_read_weight(void) {
    if (!is_ready(&hx711)) return 0.0f;

    float weight = get_weight(&hx711, 1, CHANNEL_A);
    return (weight < 0.0f) ? 0.0f : weight;
}

float scale_filter_weight(float raw_weight) {
    static float filtered_weight = 0.0f;
    static uint8_t first_reading = 1;

    if (first_reading) {
        filtered_weight = raw_weight;
        first_reading = 0;
    } else {
        filtered_weight = WEIGHT_FILTER_ALPHA * raw_weight + (1.0f - WEIGHT_FILTER_ALPHA) * filtered_weight;
    }
    return filtered_weight;
}

uint8_t scale_is_weight_stable(float weight) {
    float weight_diff = fabsf(weight - scale_state.previous_weight);

    if (weight_diff < scale_state.weight_threshold) {
        scale_state.weight_stable_count++;
        if (scale_state.weight_stable_count >= WEIGHT_STABILITY_COUNT) {
            scale_state.weight_stable_count = WEIGHT_STABILITY_COUNT;
            scale_state.previous_weight = weight;
            return 1;
        }
    } else {
        scale_state.weight_stable_count = 0;
        scale_state.previous_weight = weight;
    }
    return 0;
}

void scale_display_weight(float weight) {
    if (weight < 0.0f) weight = 0.0f;

    // Giảm ngưỡng và thêm deadzone
    if (weight < 0.02f) {  // Ngưỡng thấp hơn để tránh nhiễu
        DisplayNumber(0);
        scale_send_uart_data("Weight: 0.00 kg (LEDs: 0)\r\n");
        return;
    }

    if (weight > 9.9f) weight = 9.9f;

    // Tính số LED cần sáng (0-6)
    uint8_t leds_to_light = (uint8_t)((weight / 9.9f) * 6 + 0.5f);
    DisplayNumber(leds_to_light);
    scale_send_uart_data("Weight: %.2f kg (LEDs: %d)\r\n", weight, leds_to_light);
}
void scale_process_rfid(float weight) {
    uint8_t CardID[5];

    if (TM_MFRC522_Check(CardID) == MI_OK) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
        scale_send_uart_data("RFID: %02X%02X%02X%02X%02X | Weight: %.3f kg\r\n",
                           CardID[0], CardID[1], CardID[2], CardID[3], CardID[4], weight);
    } else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

void scale_send_uart_data(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int len = vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(uart_buffer)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, 1000);
    }
}

void scale_handle_error(const char* error_msg) {
    scale_send_uart_data("ERROR: %s\r\n", error_msg);
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI4_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim6);
    TM_MFRC522_Init();
    scale_init();
    BlinkLEDs(3, 200); // Nhấp nháy LED 3 lần khi khởi động
    DisplayNumber(0);

    scale_state.system_ready = 1;
    scale_state.last_measurement_time = HAL_GetTick();
    scale_state.last_display_time = HAL_GetTick();
    /* USER CODE END 2 */

    while (1) {
        uint32_t current_time = HAL_GetTick();

        if (current_time - scale_state.last_measurement_time >= MEASUREMENT_INTERVAL) {
            scale_state.last_measurement_time = current_time;
            float raw_weight = scale_read_weight();
            float filtered_weight = scale_filter_weight(raw_weight);
            scale_state.current_weight = filtered_weight;

            if (scale_is_weight_stable(filtered_weight)) {
                if (current_time - scale_state.last_display_time >= DISPLAY_UPDATE_INTERVAL) {
                    scale_state.last_display_time = current_time;
                    scale_display_weight(scale_state.current_weight);
                    scale_process_rfid(scale_state.current_weight);
                }
            } else {
                if (current_time - scale_state.last_display_time >= (DISPLAY_UPDATE_INTERVAL / 2)) {
                    scale_state.last_display_time = current_time;
                    scale_display_weight(scale_state.current_weight);
                }
            }
        }
        HAL_Delay(10);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                            |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE8 PE9 PE10 PE11 PE12 PE13 */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PG13 PG14 */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

static void MX_SPI4_Init(void) {
    hspi4.Instance = SPI4;
    hspi4.Init.Mode = SPI_MODE_MASTER;
    hspi4.Init.Direction = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi4.Init.NSS = SPI_NSS_SOFT;
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi4);
}

static void MX_TIM6_Init(void) {
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
