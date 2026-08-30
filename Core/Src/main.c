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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Hardware configuration
#define NUM_JOINTS          6
#define NUM_LIMIT_SWITCHES  6
#define NUM_HALL_SENSORS    6

// Shared EN pin
#define EN_PORT   GPIOB
#define EN_PIN    GPIO_PIN_0

// DIR/STEP pins
#define JOINT0_DIR_PORT   GPIOC
#define JOINT0_DIR_PIN    GPIO_PIN_0
#define JOINT0_STEP_TIM   htim2
#define JOINT0_STEP_CH    TIM_CHANNEL_2

#define JOINT1_DIR_PORT   GPIOA             // PLACEHOLDER (TMC5160)
#define JOINT1_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT1_STEP_TIM   htim3             // PLACEHOLDER - not generated in CubeMX yet
#define JOINT1_STEP_CH    TIM_CHANNEL_1     // PLACEHOLDER

#define JOINT2_DIR_PORT   GPIOA             // PLACEHOLDER
#define JOINT2_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT2_STEP_TIM   htim3             // PLACEHOLDER
#define JOINT2_STEP_CH    TIM_CHANNEL_2     // PLACEHOLDER

#define JOINT3_DIR_PORT   GPIOA             // PLACEHOLDER
#define JOINT3_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT3_STEP_TIM   htim3             // PLACEHOLDER
#define JOINT3_STEP_CH    TIM_CHANNEL_3     // PLACEHOLDER

#define JOINT4_DIR_PORT   GPIOA             // PLACEHOLDER
#define JOINT4_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT4_STEP_TIM   htim3             // PLACEHOLDER
#define JOINT4_STEP_CH    TIM_CHANNEL_4     // PLACEHOLDER

#define JOINT5_DIR_PORT   GPIOA             // PLACEHOLDER
#define JOINT5_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT5_STEP_TIM   htim5             // PLACEHOLDER - TIM3 fully used by joints 1-4
#define JOINT5_STEP_CH    TIM_CHANNEL_1     // PLACEHOLDER

// UART for TMC2209
#define TMC_UART1_TX_PORT  GPIOA
#define TMC_UART1_TX_PIN   GPIO_PIN_9       // CONFIRMED (USART1_TX)
#define TMC_UART1_RX_PORT  GPIOA
#define TMC_UART1_RX_PIN   GPIO_PIN_10      // CONFIRMED (USART1_RX)

#define TMC_UART2_TX_PORT  GPIOA            // PLACEHOLDER - not generated in CubeMX yet
#define TMC_UART2_TX_PIN   GPIO_PIN_0       // PLACEHOLDER
#define TMC_UART2_RX_PORT  GPIOA            // PLACEHOLDER
#define TMC_UART2_RX_PIN   GPIO_PIN_0       // PLACEHOLDER

// UART Bus and Slave Addresses
#define JOINT0_UART_BUS     1
#define JOINT0_SLAVE_ADDR   0

#define JOINT2_UART_BUS     2                // PLACEHOLDER
#define JOINT2_SLAVE_ADDR   0                // PLACEHOLDER

#define JOINT3_UART_BUS     2                // PLACEHOLDER
#define JOINT3_SLAVE_ADDR   1                // PLACEHOLDER

#define JOINT4_UART_BUS     2                // PLACEHOLDER
#define JOINT4_SLAVE_ADDR   2                // PLACEHOLDER

#define JOINT5_UART_BUS     2                // PLACEHOLDER
#define JOINT5_SLAVE_ADDR   3                // PLACEHOLDER

// SPI for TMC5160
#define TMC5160_SPI_MOSI_PORT  GPIOA        // PLACEHOLDER - not generated in CubeMX yet
#define TMC5160_SPI_MOSI_PIN   GPIO_PIN_0   // PLACEHOLDER
#define TMC5160_SPI_MISO_PORT  GPIOA        // PLACEHOLDER
#define TMC5160_SPI_MISO_PIN   GPIO_PIN_0   // PLACEHOLDER
#define TMC5160_SPI_SCK_PORT   GPIOA        // PLACEHOLDER
#define TMC5160_SPI_SCK_PIN    GPIO_PIN_0   // PLACEHOLDER
#define TMC5160_SPI_CS_PORT    GPIOA        // PLACEHOLDER
#define TMC5160_SPI_CS_PIN     GPIO_PIN_0   // PLACEHOLDER


// Run current per joint
#define JOINT0_CURRENT_A    1.2f             // CONFIRMED
#define JOINT1_CURRENT_A    1.2f             // PLACEHOLDER (TMC5160)
#define JOINT2_CURRENT_A    1.2f             // PLACEHOLDER
#define JOINT3_CURRENT_A    1.2f             // PLACEHOLDER
#define JOINT4_CURRENT_A    1.2f             // PLACEHOLDER
#define JOINT5_CURRENT_A    1.2f             // PLACEHOLDER

// Limit switches
#define LIMIT0_PORT   GPIOC
#define LIMIT0_PIN    GPIO_PIN_4
#define LIMIT1_PORT   GPIOA                 // PLACEHOLDER - wiring pin TBD
#define LIMIT1_PIN    GPIO_PIN_0            // PLACEHOLDER
#define LIMIT2_PORT   GPIOA                 // PLACEHOLDER
#define LIMIT2_PIN    GPIO_PIN_0            // PLACEHOLDER
#define LIMIT3_PORT   GPIOA                 // PLACEHOLDER
#define LIMIT3_PIN    GPIO_PIN_0            // PLACEHOLDER
#define LIMIT4_PORT   GPIOA                 // PLACEHOLDER
#define LIMIT4_PIN    GPIO_PIN_0            // PLACEHOLDER
#define LIMIT5_PORT   GPIOA                 // PLACEHOLDER
#define LIMIT5_PIN    GPIO_PIN_0            // PLACEHOLDER

// Hall effect sensors
#define HALL0_PORT    GPIOB
#define HALL0_PIN     GPIO_PIN_13
#define HALL1_PORT    GPIOA                 // PLACEHOLDER
#define HALL1_PIN     GPIO_PIN_0            // PLACEHOLDER
#define HALL2_PORT    GPIOA                 // PLACEHOLDER
#define HALL2_PIN     GPIO_PIN_0            // PLACEHOLDER
#define HALL3_PORT    GPIOA                 // PLACEHOLDER
#define HALL3_PIN     GPIO_PIN_0            // PLACEHOLDER
#define HALL4_PORT    GPIOA                 // PLACEHOLDER
#define HALL4_PIN     GPIO_PIN_0            // PLACEHOLDER
#define HALL5_PORT    GPIOA                 // PLACEHOLDER
#define HALL5_PIN     GPIO_PIN_0            // PLACEHOLDER

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// SERVO BEGIN
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle){
    uint32_t pulse_length = 225 + ((uint32_t)angle * (1125 - 225)) / 180;
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}
// SERVO END

// TMC2209 STEP/DIR BEGIN
// DIR  -> PC0
// EN   -> PB0  (TMC2209 ENN pin is ACTIVE-LOW: LOW = driver enabled)
// STEP -> TIM2 CH2 (PWM output, one rising edge per microstep)

void Stepper_Enable(uint8_t enable){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Stepper_SetDirection(uint8_t dir){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Run(uint8_t run){
    if (run)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    }
}

// Change step rate on the fly. htim2 clocks at 1 MHz (84MHz / 84 prescaler),
// so step_freq_hz can range roughly 1 Hz - a few hundred kHz depending on driver limits.
void Stepper_SetStepFrequency(uint32_t step_freq_hz)
{
    if (step_freq_hz == 0) return;
    uint32_t period = (1000000U / step_freq_hz);
    if (period < 2) period = 2; // keep a valid duty cycle
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period / 2); // ~50% duty
}
// TMC2209 STEP/DIR END

// TMC2209 UART CONFIG BEGIN
// Single-wire UART on PA2(TX)/PA3(RX), driver address 0 (MS1=MS2=GND)

// !!! SET THIS to your driver's sense resistor value (ohms). Common TMC2209
// breakout boards (BTT/FYSETC/Watterott) use 0.11 ohm - check your board's
// silkscreen/datasheet, this directly scales the current calculation below.
#define TMC_RSENSE      0.11f
#define TMC_VSENSE_BIT  0        // 0 = Vfs 0.325V (normal), 1 = Vfs 0.180V (more res, less headroom)
#define TMC_SLAVE_ADDR  0x00     // MS1=MS2=GND -> address 0

#define TMC_REG_GCONF      0x00
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_CHOPCONF   0x6C

static uint8_t TMC_CalcCRC(uint8_t *datagram, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            currentByte >>= 1;
        }
    }
    return crc;
}

// Reads one byte by polling the RXNE flag directly instead of HAL_UART_Receive -
// proven reliable in testing where HAL_UART_Receive was flaky on this setup.
// Reading SR (via the flag check) then DR also auto-clears any ORE/FE/NE error.
static HAL_StatusTypeDef TMC_ReceiveByte(uint8_t *byte, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        if ((HAL_GetTick() - start) > timeout_ms) return HAL_TIMEOUT;
    }
    *byte = (uint8_t)(huart1.Instance->DR);
    return HAL_OK;
}

static HAL_StatusTypeDef TMC_ReceiveBytes(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (TMC_ReceiveByte(&buf[i], timeout_ms) != HAL_OK) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

// Builds and sends the 8-byte write datagram: sync, addr, reg|write, 4 data bytes MSB-first, crc
static void TMC_WriteRegister(uint8_t reg, uint32_t data)
{
    uint8_t datagram[8];
    datagram[0] = 0x05; // sync byte
    datagram[1] = TMC_SLAVE_ADDR;
    datagram[2] = reg | 0x80; // write bit set
    datagram[3] = (uint8_t)(data >> 24);
    datagram[4] = (uint8_t)(data >> 16);
    datagram[5] = (uint8_t)(data >> 8);
    datagram[6] = (uint8_t)(data);
    datagram[7] = TMC_CalcCRC(datagram, 7);

    HAL_UART_Transmit(&huart1, datagram, 8, 10);
    // this 8-byte datagram echoes back on RX since TX/RX are tied together -
    // flush it before reading the driver's actual reply (see TMC_ReadRegister)
    uint8_t echo[8];
    TMC_ReceiveBytes(echo, 8, 50);
}

// Reads a register back so you can VERIFY a write actually took effect.
// Returns 1 on success (valid reply + good CRC) and fills *data_out.
static uint8_t TMC_ReadRegister(uint8_t reg, uint32_t *data_out)
{
    uint8_t request[4];
    request[0] = 0x05;
    request[1] = TMC_SLAVE_ADDR;
    request[2] = reg & 0x7F; // read bit clear
    request[3] = TMC_CalcCRC(request, 3);

    uint8_t echo[4];
    HAL_UART_Transmit(&huart1, request, 4, 10);
    TMC_ReceiveBytes(echo, 4, 50); // flush the 4-byte echo of our own request

    uint8_t reply[8];
    if (TMC_ReceiveBytes(reply, 8, 50) != HAL_OK)
        return 0; // timed out - no reply came back (check wiring/resistor/baud)

    if (TMC_CalcCRC(reply, 7) != reply[7])
        return 0; // CRC mismatch - reply is corrupted, don't trust it

    *data_out = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16)
              | ((uint32_t)reply[5] << 8)  | (uint32_t)reply[6];
    return 1;
}

// Converts a desired RMS run current (amps) into the 5-bit IRUN current-scale value (0-31)
static uint8_t TMC_CurrentToCS(float irun_amps)
{
    float vfs = TMC_VSENSE_BIT ? 0.180f : 0.325f;
    float cs = (irun_amps * 32.0f * 1.41421356f * (TMC_RSENSE + 0.02f) / vfs) - 1.0f;
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    return (uint8_t)(cs + 0.5f);
}

// Reads CHOPCONF back and checks MRES/en_spreadcycle actually match what we wrote.
// Returns 1 if config verified good, 0 if the read failed or values don't match.
uint8_t Stepper_UART_VerifyConfig(void)
{
    uint32_t chopconf = 0, gconf = 0;
    if (!TMC_ReadRegister(TMC_REG_CHOPCONF, &chopconf)) return 0;
    if (!TMC_ReadRegister(TMC_REG_GCONF, &gconf)) return 0;

    uint32_t mres = (chopconf >> 24) & 0x0F;
    uint32_t en_spreadcycle = (gconf >> 2) & 0x01;

    return (mres == 0) && (en_spreadcycle == 1);
}

void Stepper_UART_Config(void)
{
    // GCONF: pdn_disable=1 (required for UART control), mstep_reg_select=1
    // (CHOPCONF.MRES sets microstepping, overriding the grounded MS1/MS2 pins),
    // en_spreadcycle=1 -> SpreadCycle (robot arm: better torque/accuracy under
    // load and less chance of missed steps than StealthChop, at the cost of noise)
    uint32_t gconf = (1UL << 6) | (1UL << 7) | (1UL << 2);
    TMC_WriteRegister(TMC_REG_GCONF, gconf);

    // CHOPCONF: MRES=0000 -> 1/256 microstepping (finest), INTPOL=1 (interpolate
    // to 256 steps internally for smoothness), plus standard chopper timing bits
    uint32_t toff = 3, hstrt = 5, hend = 2, tbl = 2, mres = 0, intpol = 1;
    uint32_t chopconf = (toff << 0) | (hstrt << 4) | (hend << 7) | (tbl << 15)
                       | ((uint32_t)TMC_VSENSE_BIT << 17) | (mres << 24) | (intpol << 28);
    TMC_WriteRegister(TMC_REG_CHOPCONF, chopconf);

    // IHOLD_IRUN: run current = 1.2A, hold current = 50% of run (reduces heat when idle)
    uint8_t irun_cs  = TMC_CurrentToCS(1.2f);
    uint8_t ihold_cs = TMC_CurrentToCS(0.6f);
    uint32_t ihold_irun = ((uint32_t)ihold_cs << 0) | ((uint32_t)irun_cs << 8) | (4UL << 16); // IHOLDDELAY=4
    TMC_WriteRegister(TMC_REG_IHOLD_IRUN, ihold_irun);
}
// TMC2209 UART CONFIG END
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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // SERVO BEGIN
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  Set_Servo_Angle(&htim4, TIM_CHANNEL_2, 0);
  //SERVO END

  // TMC2209 STEP/DIR BEGIN
  Stepper_UART_Config();     // set 1/256 microstepping, SpreadCycle, 1.2A run current
  HAL_Delay(10);             // let the driver settle before reading back

//  // ---- TEMPORARY DEBUG 3: raw echo/reply capture from the REAL driver ----
//  // Put a breakpoint on the __NOP() line and inspect: dbg3_echo, dbg3_echo_ok,
//  // dbg3_reply, dbg3_reply_ok, dbg3_crc_calc (compare against dbg3_reply[7]).
//  static uint8_t dbg3_request[4];
//  dbg3_request[0] = 0x05;
//  dbg3_request[1] = TMC_SLAVE_ADDR;
//  dbg3_request[2] = TMC_REG_CHOPCONF & 0x7F;
//  dbg3_request[3] = TMC_CalcCRC(dbg3_request, 3);
//
//  static uint8_t dbg3_echo[4]  = {0xAA, 0xAA, 0xAA, 0xAA};
//  static uint8_t dbg3_reply[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
//
//  HAL_UART_Transmit(&huart1, dbg3_request, 4, 10);
//  HAL_StatusTypeDef dbg3_echo_ok  = TMC_ReceiveBytes(dbg3_echo, 4, 50);
//  HAL_StatusTypeDef dbg3_reply_ok = TMC_ReceiveBytes(dbg3_reply, 8, 50);
//  uint8_t dbg3_crc_calc = TMC_CalcCRC(dbg3_reply, 7); // compare to dbg3_reply[7]
//
//   __NOP(); // <-- BREAKPOINT HERE
////   ---- END TEMPORARY DEBUG 3 ----
//
//  if (!Stepper_UART_VerifyConfig())
//  {
//	  // config read-back failed or didn't match -> blink PA5 fast forever so
//	  // it's obvious at a glance, instead of silently running on wrong settings
//	  while (1)
//	  {
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(100);
//	  }
//  }
  Stepper_SetDirection(1);   // pick a direction to start; flip to 0 to reverse
  Stepper_Enable(1);         // pulls EN (PB0) low -> driver enabled
  Stepper_Run(1);            // starts STEP pulses on TIM2 CH2 -> motor spins continuously
  // TMC2209 STEP/DIR END
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	// SERVO BEGIN
//	Set_Servo_Angle(&htim4, TIM_CHANNEL_2, 0);
//	HAL_Delay(1000);
//
//	Set_Servo_Angle(&htim4, TIM_CHANNEL_2, 40);
//	HAL_Delay(1000);
//
//	Set_Servo_Angle(&htim4, TIM_CHANNEL_2, 80);
//	HAL_Delay(1000);
//	//SERVO END

//	// LIMIT SWITCH BEGIN
//	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET){
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	} else{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//	}
//	// LIMIT SWITCH END
//
//	// HALL EFFECT BEGIN
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET){
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	} else{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//	}
//	// HALL EFFECT END
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 90-1;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Prescaler = 200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
