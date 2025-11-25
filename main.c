/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define ADXL_ADDR_7BIT 0x53 // SDO=GND → 0x53, SDO=VCC → 0x1D
#define ADXL_ADDR (ADXL_ADDR_7BIT << 1) // HAL expects 8-bit address
// ADXL3x3 / 345 key registers
#define REG_DEVID 0x00
#define REG_BW_RATE 0x2C
#define REG_POWER_CTL 0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0 0x32

static uint8_t adxl_read_u8(uint8_t reg);
static HAL_StatusTypeDef adxl_write_u8(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef adxl_init(void);
static HAL_StatusTypeDef adxl_read_xyz(int16_t *x, int16_t *y, int16_t *z);


#define G_PER_LSB 0.0039f // ADXL full-res ≈ 3.9 mg/LSB
#define CAL_SAMPLES 200 // samples for calibration (board kept still)
#define READ_PERIOD_MS 20 // ~50 Hz main loop
#define ST_ALPHA 0.50f // short-term EWMA weight
#define LT_ALPHA 0.01f // long-term EWMA weight
#define TRIGGER_DELTA_G 0.03f // threshold: ST - LT > this → quake
#define LED_HOLD_MS 10000U // 10 seconds
#define COOLDOWN_MS 2000U // ignore re-triggers right after hold

#define LED_POST_HOLD_MS 10000U // keep blinking for 10 s after activity
#define BLINK_PERIOD_MS 150U // blink cadence while alarming


#define ALARM_DURATION_MS 5000U
#define ALARM_OFF_DELAY_MS 1000U

static float g_bias_x = 0, g_bias_y = 0, g_bias_z = 0;
static float st = 0.0f; // short-term EWMA
static float lt = 0.0f; // long-term EWMA

typedef enum
{
Setup_State, // 00
Detection_State, // 01
Detected_State, // 10
Alarm_Off_State,
} eSystemState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t adxl_read_u8(uint8_t reg)
{
uint8_t v=0;
HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 50);
return v;
}

static HAL_StatusTypeDef adxl_write_u8(uint8_t reg, uint8_t val)
{
return HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 50);
}

static HAL_StatusTypeDef adxl_init(void)
{
// 1) Check device ID
uint8_t who = adxl_read_u8(REG_DEVID);
if (who != 0xE5) {
return HAL_ERROR;
}
// 2) Data rate ~100 Hz
if (adxl_write_u8(REG_BW_RATE, 0x0A) != HAL_OK) return HAL_ERROR;
// 3) Full-resolution, ±2g (0x08)
if (adxl_write_u8(REG_DATA_FORMAT, 0x08) != HAL_OK) return HAL_ERROR;
// 4) Measurement mode (set Measure bit)
if (adxl_write_u8(REG_POWER_CTL, 0x08) != HAL_OK) return HAL_ERROR;

return HAL_OK;
}

static HAL_StatusTypeDef adxl_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
uint8_t buf[6];
HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, REG_DATAX0,
I2C_MEMADD_SIZE_8BIT, buf, 6, 50);
if (st != HAL_OK) return st;
// little-endian: low byte then high byte
*x = (int16_t)((buf[1] << 8) | buf[0]);
*y = (int16_t)((buf[3] << 8) | buf[2]);
*z = (int16_t)((buf[5] << 8) | buf[4]);
return HAL_OK;
}


static void led_on(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); }
static void led_off(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); }
static void led_blink_times(int times, int on_ms, int off_ms){
for(int i=0;i<times;i++){ led_on(); HAL_Delay(on_ms); led_off(); HAL_Delay(off_ms); }
}

// Average a bunch of samples while board is still; stores baseline g-vector
static HAL_StatusTypeDef adxl_calibrate(void)
{
int32_t sx=0, sy=0, sz=0;
int16_t x,y,z;

for (int i=0;i<CAL_SAMPLES;i++){
if (adxl_read_xyz(&x,&y,&z) != HAL_OK) return HAL_ERROR;
sx += x; sy += y; sz += z;
HAL_Delay(5);
}
float ax = (sx / (float)CAL_SAMPLES) * G_PER_LSB;
float ay = (sy / (float)CAL_SAMPLES) * G_PER_LSB;
float az = (sz / (float)CAL_SAMPLES) * G_PER_LSB;

// Save baseline so we can measure motion as deviation from this
g_bias_x = ax; g_bias_y = ay; g_bias_z = az;
return HAL_OK;
}

static inline float f_abs(float v){ return (v>=0)? v : -v; }



int CheckForQuake(void)
{
static uint32_t last_check = 0;

// 1. RATE LIMITING: Only check every 20ms (50Hz)
// If we check too fast, we read duplicate data and break the math.
if ((HAL_GetTick() - last_check) < 20) {
return 0;
}
last_check = HAL_GetTick();

// 2. Read Sensor
int16_t rx, ry, rz;
if (adxl_read_xyz(&rx, &ry, &rz) != HAL_OK) return 0;

// 3. Calculate Magnitude
float xg = rx * G_PER_LSB - g_bias_x;
float yg = ry * G_PER_LSB - g_bias_y;
float zg = rz * G_PER_LSB - g_bias_z;
float mag = f_abs(xg) + f_abs(yg) + f_abs(zg);

// 4. Update Averages
st = ST_ALPHA * mag + (1.0f - ST_ALPHA) * st;
lt = LT_ALPHA * mag + (1.0f - LT_ALPHA) * lt;

// 5. Compare
if ((st - lt) > TRIGGER_DELTA_G) return 1; // Quake!
return 0; // No Quake
}

// --- STATE HANDLER FUNCTIONS ---

eSystemState SetupStateHandler(void)
{
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

if (adxl_init() != HAL_OK) {
while (1){ HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); HAL_Delay(500); }
}

// IMPORTANT: Keep board still during this delay!
if (adxl_calibrate() != HAL_OK) {
while (1){ HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); HAL_Delay(300); }
}
led_blink_times(3, 120, 120);
return Detection_State;
}

eSystemState DetectionStateHandler(void)
{
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

if (CheckForQuake()) {
return Detected_State;
} else {
return Detection_State;
}
}

eSystemState DetectedStateHandler(void)
{
static uint32_t entry_time = 0;

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

// 1. Entry Logic
if (entry_time == 0) {
entry_time = HAL_GetTick();
led_on();
}

// 2. Continuous Shaking Check
// If quake continues, we reset the timer so the alarm stays on longer
if (CheckForQuake()) {
entry_time = HAL_GetTick();
}

// 3. Exit Logic (Time passed?)
if ((HAL_GetTick() - entry_time) > ALARM_DURATION_MS) {
entry_time = 0;
led_off();
return Alarm_Off_State;
}

return Detected_State;
}

eSystemState AlarmOffStateHandler(void)
{
static uint32_t off_start_time = 0;

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

if (off_start_time == 0) {
off_start_time = HAL_GetTick();
led_off();
}

// Check for quake even while "cooling down"
if (CheckForQuake()) {
off_start_time = 0;
return Detected_State;
}

if ((HAL_GetTick() - off_start_time) > ALARM_OFF_DELAY_MS) {
off_start_time = 0;
return Detection_State;
}

return Alarm_Off_State;
}

/* USER CODE END 0 */

/**
* @brief The application entry point.
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
MX_I2C1_Init();
/* USER CODE BEGIN 2 */


// --- Runtime state for the main loop ---
float st = 0.0f; // short-term EWMA
float lt = 0.0f; // long-term EWMA
uint32_t now = HAL_GetTick();
uint32_t next_read = now + READ_PERIOD_MS;

// Alarm/blink control
uint32_t hold_until = 0; // blink while (now < hold_until)
uint32_t next_blink_toggle = 0; // non-blocking LED toggle timer
uint8_t led_state = 0;

// Try to initialize the accelerometer
eSystemState systemState = Setup_State;




/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
// State Machine Logic
switch (systemState)
{
case Setup_State:
systemState = SetupStateHandler();
break;

case Detection_State:
systemState = DetectionStateHandler();
break;

case Detected_State:
systemState = DetectedStateHandler();
break;

case Alarm_Off_State:
systemState = AlarmOffStateHandler();
break;

default:
systemState = Setup_State; // Should not happen
break;
}

// Small idle so loop timing stays stable
HAL_Delay(1);
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
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
{
Error_Handler();
}

/** Initializes the CPU, AHB and APB buses clocks
*/
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
{
Error_Handler();
}
PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
{
Error_Handler();
}
}

/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C1_Init(void)
{

/* USER CODE BEGIN I2C1_Init 0 */

/* USER CODE END I2C1_Init 0 */

/* USER CODE BEGIN I2C1_Init 1 */

/* USER CODE END I2C1_Init 1 */
hi2c1.Instance = I2C1;
hi2c1.Init.Timing = 0x00201D2B;
hi2c1.Init.OwnAddress1 = 0;
hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hi2c1.Init.OwnAddress2 = 0;
hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
if (HAL_I2C_Init(&hi2c1) != HAL_OK)
{
Error_Handler();
}

/** Configure Analogue filter
*/
if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
{
Error_Handler();
}

/** Configure Digital filter
*/
if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN I2C1_Init 2 */

/* USER CODE END I2C1_Init 2 */

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
huart2.Init.BaudRate = 38400;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
__HAL_RCC_GPIOF_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

/*Configure GPIO pins : PA0 PA1 */
GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pin : PB5 */
GPIO_InitStruct.Pin = GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @brief This function is executed in case of error occurrence.
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
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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
