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
#include <math.h>
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
#define JOINT0_DIR_PORT   GPIOA             // PLACEHOLDER
#define JOINT0_DIR_PIN    GPIO_PIN_0        // PLACEHOLDER
#define JOINT0_STEP_TIM   htim5             // PLACEHOLDER - TIM3 fully used by joints 1-4
#define JOINT0_STEP_CH    TIM_CHANNEL_1     // PLACEHOLDER

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

#define JOINT5_DIR_PORT   GPIOC
#define JOINT5_DIR_PIN    GPIO_PIN_0
#define JOINT5_STEP_TIM   htim2
#define JOINT5_STEP_CH    TIM_CHANNEL_2

// UART for TMC2209
#define TMC_UART1_TX_PORT  GPIOA
#define TMC_UART1_TX_PIN   GPIO_PIN_9
#define TMC_UART1_RX_PORT  GPIOA
#define TMC_UART1_RX_PIN   GPIO_PIN_10

#define TMC_UART2_TX_PORT  GPIOA            // PLACEHOLDER - not generated in CubeMX yet
#define TMC_UART2_TX_PIN   GPIO_PIN_0       // PLACEHOLDER
#define TMC_UART2_RX_PORT  GPIOA            // PLACEHOLDER
#define TMC_UART2_RX_PIN   GPIO_PIN_0       // PLACEHOLDER

// UART Bus and Slave Addresses
#define JOINT0_UART_BUS     2
#define JOINT0_SLAVE_ADDR   0

#define JOINT2_UART_BUS     1                // PLACEHOLDER
#define JOINT2_SLAVE_ADDR   0                // PLACEHOLDER

#define JOINT3_UART_BUS     1                // PLACEHOLDER
#define JOINT3_SLAVE_ADDR   1                // PLACEHOLDER

#define JOINT4_UART_BUS     1                // PLACEHOLDER
#define JOINT4_SLAVE_ADDR   2                // PLACEHOLDER

#define JOINT5_UART_BUS     1                // PLACEHOLDER
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

// Joint gear ratios
#define JOINT0_GEAR_RATIO   200.0f/30.0f
#define JOINT1_GEAR_RATIO   25.0f
#define JOINT2_GEAR_RATIO   ((16.0f*6.0f) / (14.0f*2.0f)) * ((16.0f*6.0f) / 16.0f)
#define JOINT3_GEAR_RATIO   5.5f
#define JOINT4_GEAR_RATIO   4.0f
#define JOINT5_GEAR_RATIO   1.0f

// Joint microsteps
#define JOINT0_MICROSTEPS   256             // CONFIRMED (matches Stepper_UART_Config's MRES=0)
#define JOINT1_MICROSTEPS   256
#define JOINT2_MICROSTEPS   256
#define JOINT3_MICROSTEPS   256
#define JOINT4_MICROSTEPS   256
#define JOINT5_MICROSTEPS   256

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

// JOINT STEP/DIR/EN BEGIN
#define MOTOR_FULL_STEPS_PER_REV  200.0f   // 1.8deg/step across the board for the NEMA 17s and the 23

typedef struct {
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    TIM_HandleTypeDef *step_tim;   // NULL until this joint's timer is generated in CubeMX
    uint32_t           step_ch;
    float              gear_ratio;
    uint16_t           microsteps;
    uint8_t            configured; // 1 = real hardware, 0 = placeholder (calls no-op)
} JointConfig;

static JointConfig joint_cfg[NUM_JOINTS] = {
    // Joint 0
    { JOINT0_DIR_PORT, JOINT0_DIR_PIN, NULL, JOINT0_STEP_CH,
      JOINT0_GEAR_RATIO, JOINT0_MICROSTEPS, 0 },
    // Joint 1 (TMC5160)
    { JOINT1_DIR_PORT, JOINT1_DIR_PIN, NULL, JOINT1_STEP_CH,
      JOINT1_GEAR_RATIO, JOINT1_MICROSTEPS, 0 },
    // Joint 2
    { JOINT2_DIR_PORT, JOINT2_DIR_PIN, NULL, JOINT2_STEP_CH,
      JOINT2_GEAR_RATIO, JOINT2_MICROSTEPS, 0 },
    // Joint 3
    { JOINT3_DIR_PORT, JOINT3_DIR_PIN, NULL, JOINT3_STEP_CH,
      JOINT3_GEAR_RATIO, JOINT3_MICROSTEPS, 0 },
    // Joint 4
    { JOINT4_DIR_PORT, JOINT4_DIR_PIN, NULL, JOINT4_STEP_CH,
      JOINT4_GEAR_RATIO, JOINT4_MICROSTEPS, 0 },
    // Joint 5
    { JOINT5_DIR_PORT, JOINT5_DIR_PIN, &JOINT5_STEP_TIM, JOINT5_STEP_CH,
      JOINT5_GEAR_RATIO, JOINT5_MICROSTEPS, 1 },
};

void Joint_Enable(uint8_t enable){
	// Active-low EN
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Joint_SetDirection(uint8_t joint, uint8_t dir){
	// REMEMBER TO GET RID OF THE !joint_cfg[joint[.configure LATER WHEN EVERYTHING IS PROPERLY CONFIGURED (waste of calcs)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (joint >= NUM_JOINTS || !joint_cfg[joint].configured){
    	return;
    }

    HAL_GPIO_WritePin(joint_cfg[joint].dir_port, joint_cfg[joint].dir_pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Joint_Run(uint8_t joint, uint8_t run){
    if (joint >= NUM_JOINTS || !joint_cfg[joint].configured){
    	return;
    }

    if (run){
    	HAL_TIM_PWM_Start(joint_cfg[joint].step_tim, joint_cfg[joint].step_ch);
    } else{
    	HAL_TIM_PWM_Stop(joint_cfg[joint].step_tim, joint_cfg[joint].step_ch);
    }
}

// Sets STEP pulse frequency directly.
void Joint_SetStepFrequency(uint8_t joint, uint32_t step_freq_hz){
    if (joint >= NUM_JOINTS || !joint_cfg[joint].configured || step_freq_hz == 0){
    	return;
    }

    uint32_t period = (1000000U / step_freq_hz);
    if (period < 2){
    	period = 2; // keep a valid duty cycle
    }

    __HAL_TIM_SET_AUTORELOAD(joint_cfg[joint].step_tim, period - 1);
    __HAL_TIM_SET_COMPARE(joint_cfg[joint].step_tim, joint_cfg[joint].step_ch, period / 2);
}

// Drives a joint at a given OUTPUT SHAFT speed (deg/s, sign sets direction)
void Joint_MoveAtSpeed(uint8_t joint, float output_deg_per_sec){
    if (joint >= NUM_JOINTS || !joint_cfg[joint].configured){
    	return;
    }

    // Stop the joint
    if (output_deg_per_sec == 0.0f){
        Joint_Run(joint, 0);
        return;
    }

    // Joint specific speed
    float steps_per_output_rev = MOTOR_FULL_STEPS_PER_REV * (float)joint_cfg[joint].microsteps * joint_cfg[joint].gear_ratio;
    float output_rev_per_sec = fabsf(output_deg_per_sec) / 360.0f;
    uint32_t step_freq_hz = (uint32_t)(output_rev_per_sec * steps_per_output_rev);

    Joint_SetDirection(joint, output_deg_per_sec > 0 ? 1 : 0);
    Joint_SetStepFrequency(joint, step_freq_hz);
    Joint_Run(joint, 1);
}
// JOINT STEP/DIR/EN END

// TMC2209 UART
static UART_HandleTypeDef *tmc_uart_bus[3] = { NULL, &huart1, NULL }; // [1]=bus1, [2]=bus2, [0] unused
#define TMC_RSENSE      0.11f
#define TMC_VSENSE_BIT  0        // 0 = Vfs 0.325V (normal), 1 = Vfs 0.180V (more res, less headroom)

#define TMC_REG_GCONF      0x00
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_CHOPCONF   0x6C

typedef struct {
    uint8_t bus;         // 1 or 2 (index into tmc_uart_bus), 0 = not UART-driven (TMC5160)
    uint8_t slave_addr;  // 0-3, set via that driver's MS1/MS2
    float   current_a;
    uint8_t configured;  // 1 = physically wired right now, DELETE THIS LATER ON ASWELLL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
} TMC2209_JointConfig;

static TMC2209_JointConfig tmc2209_cfg[NUM_JOINTS] = {
    { JOINT0_UART_BUS, JOINT0_SLAVE_ADDR, JOINT0_CURRENT_A, 0 },
    { 0,               0,                 JOINT1_CURRENT_A, 0 }, // joint 1 = TMC5160, not UART
    { JOINT2_UART_BUS, JOINT2_SLAVE_ADDR, JOINT2_CURRENT_A, 0 },
    { JOINT3_UART_BUS, JOINT3_SLAVE_ADDR, JOINT3_CURRENT_A, 0 },
    { JOINT4_UART_BUS, JOINT4_SLAVE_ADDR, JOINT4_CURRENT_A, 0 },
    { JOINT5_UART_BUS, JOINT5_SLAVE_ADDR, JOINT5_CURRENT_A, 1 }, // CONFIRMED - testing this joint now
};

static uint8_t TMC_CalcCRC(uint8_t *datagram, uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++){
            if ((crc >> 7) ^ (currentByte & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            currentByte >>= 1;
        }
    }
    return crc;
}

static HAL_StatusTypeDef TMC_ReceiveByte(UART_HandleTypeDef *huart, uint8_t *byte, uint32_t timeout_ms){
    uint32_t start = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)){
        if ((HAL_GetTick() - start) > timeout_ms){
        	return HAL_TIMEOUT;
        }
    }
    *byte = (uint8_t)(huart->Instance->DR);
    return HAL_OK;
}

static HAL_StatusTypeDef TMC_ReceiveBytes(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len, uint32_t timeout_ms){
    for (uint16_t i = 0; i < len; i++){
        if (TMC_ReceiveByte(huart, &buf[i], timeout_ms) != HAL_OK) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

// Builds and sends the 8-byte write datagram: sync, addr, reg|write, 4 data bytes MSB-first, crc
static void TMC_WriteRegister(UART_HandleTypeDef *huart, uint8_t addr, uint8_t reg, uint32_t data){
    uint8_t datagram[8];
    datagram[0] = 0x05; // sync byte
    datagram[1] = addr;
    datagram[2] = reg | 0x80; // write bit set
    datagram[3] = (uint8_t)(data >> 24);
    datagram[4] = (uint8_t)(data >> 16);
    datagram[5] = (uint8_t)(data >> 8);
    datagram[6] = (uint8_t)(data);
    datagram[7] = TMC_CalcCRC(datagram, 7);

    HAL_UART_Transmit(huart, datagram, 8, 10);
    // this 8-byte datagram echoes back on RX since TX/RX are tied together -
    // flush it before reading the driver's actual reply (see TMC_ReadRegister)
    uint8_t echo[8];
    TMC_ReceiveBytes(huart, echo, 8, 50);
}

// Reads a register back so you can VERIFY a write actually took effect.
// Returns 1 on success (valid reply + good CRC) and fills *data_out.
static uint8_t TMC_ReadRegister(UART_HandleTypeDef *huart, uint8_t addr, uint8_t reg, uint32_t *data_out){
    uint8_t request[4];
    request[0] = 0x05;
    request[1] = addr;
    request[2] = reg & 0x7F; // read bit clear
    request[3] = TMC_CalcCRC(request, 3);

    uint8_t echo[4];
    HAL_UART_Transmit(huart, request, 4, 10);
    TMC_ReceiveBytes(huart, echo, 4, 50); // flush the 4-byte echo of our own request

    uint8_t reply[8];
    if (TMC_ReceiveBytes(huart, reply, 8, 50) != HAL_OK)
        return 0; // timed out - no reply came back (check wiring/resistor/baud)

    if (TMC_CalcCRC(reply, 7) != reply[7])
        return 0; // CRC mismatch - reply is corrupted, don't trust it

    *data_out = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16)
              | ((uint32_t)reply[5] << 8)  | (uint32_t)reply[6];
    return 1;
}

// Converts a desired RMS run current (amps) into the 5-bit IRUN current-scale value (0-31)
static uint8_t TMC_CurrentToCS(float irun_amps){
    float vfs = TMC_VSENSE_BIT ? 0.180f : 0.325f;
    float cs = (irun_amps * 32.0f * 1.41421356f * (TMC_RSENSE + 0.02f) / vfs) - 1.0f;
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    return (uint8_t)(cs + 0.5f);
}

// Configures one TMC2209 joint over UART: SpreadCycle, finest microstepping, run current.
void TMC2209_ConfigJoint(uint8_t joint){
    if (joint >= NUM_JOINTS || !tmc2209_cfg[joint].configured) return;
    UART_HandleTypeDef *huart = tmc_uart_bus[tmc2209_cfg[joint].bus];
    if (huart == NULL) return; // that bus isn't wired/generated yet
    uint8_t addr = tmc2209_cfg[joint].slave_addr;

    // GCONF: pdn_disable=1 (required for UART control), mstep_reg_select=1
    // (CHOPCONF.MRES sets microstepping, overriding the MS1/MS2 pins),
    // en_spreadcycle=1 -> SpreadCycle (robot arm: better torque/accuracy under
    // load and less chance of missed steps than StealthChop, at the cost of noise)
    uint32_t gconf = (1UL << 6) | (1UL << 7) | (1UL << 2);
    TMC_WriteRegister(huart, addr, TMC_REG_GCONF, gconf);

    // CHOPCONF: MRES=0000 -> 1/256 microstepping (finest), INTPOL=1 (interpolate
    // to 256 steps internally for smoothness), plus standard chopper timing bits
    uint32_t toff = 3, hstrt = 5, hend = 2, tbl = 2, mres = 0, intpol = 1;
    uint32_t chopconf = (toff << 0) | (hstrt << 4) | (hend << 7) | (tbl << 15)
                       | ((uint32_t)TMC_VSENSE_BIT << 17) | (mres << 24) | (intpol << 28);
    TMC_WriteRegister(huart, addr, TMC_REG_CHOPCONF, chopconf);

    // IHOLD_IRUN: run current from that joint's config, hold current = 50% of run
    uint8_t irun_cs  = TMC_CurrentToCS(tmc2209_cfg[joint].current_a);
    uint8_t ihold_cs = TMC_CurrentToCS(tmc2209_cfg[joint].current_a * 0.5f);
    uint32_t ihold_irun = ((uint32_t)ihold_cs << 0) | ((uint32_t)irun_cs << 8) | (4UL << 16); // IHOLDDELAY=4
    TMC_WriteRegister(huart, addr, TMC_REG_IHOLD_IRUN, ihold_irun);
}

// Reads CHOPCONF/GCONF back and checks MRES/en_spreadcycle actually match what we wrote.
uint8_t TMC2209_VerifyJoint(uint8_t joint){
    if (joint >= NUM_JOINTS || !tmc2209_cfg[joint].configured) return 0;
    UART_HandleTypeDef *huart = tmc_uart_bus[tmc2209_cfg[joint].bus];
    if (huart == NULL) return 0;
    uint8_t addr = tmc2209_cfg[joint].slave_addr;

    uint32_t chopconf = 0, gconf = 0;
    if (!TMC_ReadRegister(huart, addr, TMC_REG_CHOPCONF, &chopconf)) return 0;
    if (!TMC_ReadRegister(huart, addr, TMC_REG_GCONF, &gconf)) return 0;

    uint32_t mres = (chopconf >> 24) & 0x0F;
    uint32_t en_spreadcycle = (gconf >> 2) & 0x01;
    return (mres == 0) && (en_spreadcycle == 1);
}

// TMC5160 SPI
// TMC5160 uses a 5-byte (40-bit) datagram: [addr|RW][data3][data2][data1][data0].
// Reads are delayed by one transaction on this chip - the data you want comes back
// on the NEXT transfer, not the one that requested it - see TMC5160_ReadRegister.
// NOTE: won't compile until you add an SPI peripheral in CubeMX - SPI_HandleTypeDef
// doesn't exist in this project until that HAL module is enabled.

#define TMC5160_REG_GCONF        0x00
#define TMC5160_REG_GLOBALSCALER 0x0B
#define TMC5160_REG_IHOLD_IRUN   0x10
#define TMC5160_REG_CHOPCONF     0x6C

//static void TMC5160_CS(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t active){
//    HAL_GPIO_WritePin(cs_port, cs_pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
//}
//
//static void TMC5160_WriteRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint32_t data){
//    uint8_t tx[5], rx[5];
//    tx[0] = reg | 0x80;
//    tx[1] = (uint8_t)(data >> 24);
//    tx[2] = (uint8_t)(data >> 16);
//    tx[3] = (uint8_t)(data >> 8);
//    tx[4] = (uint8_t)(data);
//
//    TMC5160_CS(cs_port, cs_pin, 1);
//    HAL_SPI_TransmitReceive(hspi, tx, rx, 5, 10);
//    TMC5160_CS(cs_port, cs_pin, 0);
//}
//
//static uint8_t TMC5160_ReadRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint32_t *data_out){
//    uint8_t tx[5] = { (uint8_t)(reg & 0x7F), 0, 0, 0, 0 };
//    uint8_t rx[5];
//
//    // first transfer just requests the register - its rx data is stale/irrelevant
//    TMC5160_CS(cs_port, cs_pin, 1);
//    if (HAL_SPI_TransmitReceive(hspi, tx, rx, 5, 10) != HAL_OK) { TMC5160_CS(cs_port, cs_pin, 0); return 0; }
//    TMC5160_CS(cs_port, cs_pin, 0);
//
//    // second transfer is what actually returns the requested register's data
//    TMC5160_CS(cs_port, cs_pin, 1);
//    if (HAL_SPI_TransmitReceive(hspi, tx, rx, 5, 10) != HAL_OK) { TMC5160_CS(cs_port, cs_pin, 0); return 0; }
//    TMC5160_CS(cs_port, cs_pin, 0);
//
//    *data_out = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16)
//              | ((uint32_t)rx[3] << 8)  | (uint32_t)rx[4];
//    return 1; // rx[0] is the SPI status byte (driver flags) if you want it later
//}
//
//// Converts a desired RMS run current (amps) into IRUN (0-31), assuming GLOBALSCALER
//// stays at its default full-scale value. NOTE: this is a simplified version of the
//// TMC5160's two-stage current scaling - fine for now, revisit for finer control at
//// low currents once you're tuning this joint for real.
//static uint8_t TMC5160_CurrentToCS(float irun_amps, float rsense, uint8_t vsense_bit){
//    float vfs = vsense_bit ? 0.180f : 0.325f;
//    float cs = (irun_amps * 32.0f * 1.41421356f * (rsense + 0.02f) / vfs) - 1.0f;
//    if (cs < 0) cs = 0;
//    if (cs > 31) cs = 31;
//    return (uint8_t)(cs + 0.5f);
//}
//
//// Configures the TMC5160 joint (joint 1): SpreadCycle, finest microstepping, run current.
//void TMC5160_ConfigJoint(SPI_HandleTypeDef *hspi, float rsense_ohm, float current_a){
//    GPIO_TypeDef *cs_port = TMC5160_SPI_CS_PORT;
//    uint16_t      cs_pin  = TMC5160_SPI_CS_PIN;
//    uint8_t vsense_bit = 0;
//
//    // GCONF: en_pwm_mode=0 -> SpreadCycle. NOTE: TMC5160's GCONF bit here is the
//    // OPPOSITE polarity convention from TMC2209's en_spreadcycle bit - on the 5160,
//    // 0 = SpreadCycle, 1 = StealthChop, so SpreadCycle needs no bit set at all.
//    TMC5160_WriteRegister(hspi, cs_port, cs_pin, TMC5160_REG_GCONF, 0x00000000);
//
//    // GLOBALSCALER: 0 = full scale (256/256) - current control handled entirely via IRUN below
//    TMC5160_WriteRegister(hspi, cs_port, cs_pin, TMC5160_REG_GLOBALSCALER, 0x00000000);
//
//    // CHOPCONF: same field layout as TMC2209 - MRES=0 (1/256 microstepping), INTPOL=1
//    uint32_t toff = 3, hstrt = 5, hend = 2, tbl = 2, mres = 0, intpol = 1;
//    uint32_t chopconf = (toff << 0) | (hstrt << 4) | (hend << 7) | (tbl << 15)
//                       | ((uint32_t)vsense_bit << 17) | (mres << 24) | (intpol << 28);
//    TMC5160_WriteRegister(hspi, cs_port, cs_pin, TMC5160_REG_CHOPCONF, chopconf);
//
//    // IHOLD_IRUN: run current from current_a, hold current = 50% of run
//    uint8_t irun_cs  = TMC5160_CurrentToCS(current_a, rsense_ohm, vsense_bit);
//    uint8_t ihold_cs = TMC5160_CurrentToCS(current_a * 0.5f, rsense_ohm, vsense_bit);
//    uint32_t ihold_irun = ((uint32_t)ihold_cs << 0) | ((uint32_t)irun_cs << 8) | (4UL << 16);
//    TMC5160_WriteRegister(hspi, cs_port, cs_pin, TMC5160_REG_IHOLD_IRUN, ihold_irun);
//}
//
//// Reads CHOPCONF/GCONF back and checks the write actually landed.
//uint8_t TMC5160_VerifyJoint(SPI_HandleTypeDef *hspi){
//    GPIO_TypeDef *cs_port = TMC5160_SPI_CS_PORT;
//    uint16_t      cs_pin  = TMC5160_SPI_CS_PIN;
//
//    uint32_t chopconf = 0, gconf = 0;
//    if (!TMC5160_ReadRegister(hspi, cs_port, cs_pin, TMC5160_REG_CHOPCONF, &chopconf)) return 0;
//    if (!TMC5160_ReadRegister(hspi, cs_port, cs_pin, TMC5160_REG_GCONF, &gconf)) return 0;
//
//    uint32_t mres = (chopconf >> 24) & 0x0F;
//    uint32_t en_pwm_mode = (gconf >> 2) & 0x01;
//    return (mres == 0) && (en_pwm_mode == 0); // SpreadCycle -> en_pwm_mode should read back 0
//}

// ===================== Simple debug helper (works for either driver) =====================
// Blinks PA5 fast forever if config verification fails, so a bad link is obvious
// at a glance without needing a debugger attached.
void Driver_ConfigFailBlink(void){
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}
// ===================== END =====================
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

  TMC2209_ConfigJoint(5);          // set 1/256 microstepping, SpreadCycle, run current for joint 5
//  HAL_Delay(10);                   // let the driver settle before reading back
//  if (!TMC2209_VerifyJoint(5)){
//	Driver_ConfigFailBlink();    // config read-back failed or didn't match - PA5 blinks fast forever
//  }

  Joint_Enable(1);                  // pulls shared EN low -> all drivers enabled
  Joint_SetDirection(5, 1);         // joint 5: pick a direction to start; flip to 0 to reverse
  Joint_Run(5, 1);                  // starts STEP pulses on joint 5's timer -> motor spins continuously
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
