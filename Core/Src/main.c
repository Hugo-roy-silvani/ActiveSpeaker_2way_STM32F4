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
#include <string.h>
#include <math.h>
/* USER CODE END Includes */
#include "dsp.h"
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

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static DspConfig g_dspDefaultCfg = {

    .sampleRate    = 48000.0f,   // cohérent avec ta config I2S
    .crossoverFreq = 2000.0f,    // Hz

	.inputTrim_dB  = 0.0f,

    .gain = {
        [DSP_CH_LOW]  = 1.0f,    // gain voie grave
        [DSP_CH_HIGH] = 1.0f     // gain voie aigu
    },

    .delaySamples = {
        [DSP_CH_LOW]  = 0,
        [DSP_CH_HIGH] = 0
    },
    .mute = {
        [DSP_CH_LOW]  = 0,
        [DSP_CH_HIGH] = 0
    },
    .invert = {
        [DSP_CH_LOW]  = 0,
        [DSP_CH_HIGH] = 0
    },
    .hpf = {
        .enabled = 1,
	    .stages  = 2,       // 2 x 12 dB/oct = 24 dB/oct
        .freq    = 80.0f,   // à ajuster selon le HP et la charge
        .Q       = 0.707f   // Butterworth
    },
	.eq = {
	    { // LOW
	      .numEq = 0,
	      .eqCoeffs = {
	          // biquad identité avec gain 0.5 (-6 dB)
	          { 0.0630957f, 0.0f, 0.0f, 0.0f, 0.0f }
	      }
	    },
	    { // HIGH
	      .numEq = 0
	    }
	},

    // Limiteurs par voie (désactivés pour l'instant)
    .limiter = {
        [DSP_CH_LOW] = {
            .enabled       = 0,
            .threshold_dB  = -1.0f,
            .ratio         = 20.0f,
            .attack_ms     = 1.0f,
            .release_ms    = 50.0f,
            .makeupGain_dB = 0.0f
        },
        [DSP_CH_HIGH] = {
            .enabled       = 0,
            .threshold_dB  = -3.0f,  // tweeter plus fragile
            .ratio         = 20.0f,
            .attack_ms     = 1.0f,
            .release_ms    = 50.0f,
            .makeupGain_dB = 0.0f
        }
    }
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CS43L22_I2C_ADDR   0x94  // Adresse 7 bits décalée (0x4A<<1) :contentReference[oaicite:0]{index=0}
#define CS43L22_REG_ID     0x01  // Registre ID :contentReference[oaicite:1]{index=1}
#define CS43L22_ID         0xE0
#define CS43L22_ID_MASK    0xF8

#define CS43L22_REG_POWER_CTL1   0x02
#define CS43L22_REG_POWER_CTL2   0x04
#define CS43L22_REG_CLOCKING     0x05
#define CS43L22_REG_IFACE_CTL1   0x06
#define CS43L22_REG_IFACE_CTL2   0x07
#define CS43L22_REG_PASSIVE_VOL_A 0x20
#define CS43L22_REG_PASSIVE_VOL_B 0x21
#define CS43L22_REG_PLAYBACK_CTL2 0x0F

// --- Paramètres audio ---
#define AUDIO_CHANNELS            2
#define AUDIO_FRAMES_PER_HALF     256  // nb d’images (L/R) par demi-buffer
#define AUDIO_TOTAL_FRAMES        (2 * AUDIO_FRAMES_PER_HALF)   // 2 demi-buffers
#define AUDIO_BUFFER_SAMPLES      (AUDIO_TOTAL_FRAMES * AUDIO_CHANNELS)
// nombre total de samples 16 bits (L et R)

// sinus test
static float g_phase = 0.0f;
static const float TWO_PI = 6.28318530718f;
static const float SAMPLE_RATE_HZ = 48000.0f;
static const float TEST_TONE_FREQ = 500.0f;

static int16_t audio_buffer[AUDIO_BUFFER_SAMPLES];




static HAL_StatusTypeDef CS43L22_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef CS43L22_ReadReg(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;

    // On envoie d'abord l'adresse du registre
    status = HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Puis on lit 1 octet
    return HAL_I2C_Master_Receive(&hi2c1, CS43L22_I2C_ADDR, value, 1, HAL_MAX_DELAY);
}

// Reset + init de base du codec pour I2S 16 bits stéréo
static HAL_StatusTypeDef CS43L22_Init(void)
{
    HAL_StatusTypeDef st;
    uint8_t id = 0;

    // Sortir le codec du reset (PD4 = Audio_RST)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(10);

    // Lire l’ID
    st = CS43L22_ReadReg(CS43L22_REG_ID, &id);
    if (st != HAL_OK || (id & CS43L22_ID_MASK) != CS43L22_ID)
    {
        return HAL_ERROR;
    }

    // Mettre le codec en power-down logiciel pendant la config
    CS43L22_WriteReg(0x00, 0x99);
    CS43L22_WriteReg(0x47, 0x80);
    CS43L22_WriteReg(0x32, 0x80);
    CS43L22_WriteReg(0x32, 0x00);
    CS43L22_WriteReg(0x00, 0x00);

    // Clock auto, tout activé (utilise MCLK fourni par I2S3)
    CS43L22_WriteReg(CS43L22_REG_CLOCKING, 0x81);

    // Interface audio : I2S, 16 bits, format standard
    CS43L22_WriteReg(CS43L22_REG_IFACE_CTL1, 0x07); // I2S, 16-bit, slave, etc.

    // Volume casque (0x00 = 0 dB, plus bas = moins fort)
    CS43L22_WriteReg(CS43L22_REG_PASSIVE_VOL_A, 0xCC); // canal A
    CS43L22_WriteReg(CS43L22_REG_PASSIVE_VOL_B, 0xCC); // canal B

    // Activer HP left/right et mixer
    CS43L22_WriteReg(CS43L22_REG_POWER_CTL2, 0xAF);  // HP left/right on

    return HAL_OK;
}

static void CS43L22_Start(void)
{
    // Play : run
    CS43L22_WriteReg(CS43L22_REG_POWER_CTL1, 0x9E); // Power up, codec ON
}

// Remplit un demi-buffer de frames stéréo avec un sinus
static void FillSineHalfBuffer(int16_t *buf, uint32_t frames)
{
    float phase_inc = TWO_PI * TEST_TONE_FREQ / SAMPLE_RATE_HZ;

    for (uint32_t n = 0; n < frames; n++)
    {
        float s = sinf(g_phase);
        int16_t sample = (int16_t)(s * 30000.0f);

        buf[2*n + 0] = sample; // L
        buf[2*n + 1] = sample; // R

        g_phase += phase_inc;
        if (g_phase > TWO_PI)
            g_phase -= TWO_PI;
    }
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_HCD_Init();
  /* USER CODE BEGIN 2 */

  DSP_Init(&g_dspDefaultCfg);

  // 1) Initialisation du codec
  if (CS43L22_Init() != HAL_OK)
  {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      while (1) {}
  }

  // 2) Pré-remplir les deux demi-buffers avec un sinus
  //    Première moitié :
  FillSineHalfBuffer(&audio_buffer[0], AUDIO_FRAMES_PER_HALF);
  //    Deuxième moitié :
  FillSineHalfBuffer(&audio_buffer[2 * AUDIO_FRAMES_PER_HALF], AUDIO_FRAMES_PER_HALF);

  // 3) Démarrer la DMA sur tout le buffer (deux moitiés stéréo)
  if (HAL_I2S_Transmit_DMA(&hi2s3,
                           (uint16_t*)audio_buffer,
                           AUDIO_BUFFER_SAMPLES) != HAL_OK)
  {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      while (1) {}
  }

  // 4) Démarrer le codec en lecture
  CS43L22_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  HAL_Delay(500);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)  // I2S3
    {
        int16_t *first_half = &audio_buffer[0];

        // Génère un sinus brut
        FillSineHalfBuffer(first_half, AUDIO_FRAMES_PER_HALF);

        // Puis applique le DSP (biquad CMSIS)
        DSP_ProcessHalfBuffer(first_half, AUDIO_FRAMES_PER_HALF);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    {
        int16_t *second_half = &audio_buffer[2 * AUDIO_FRAMES_PER_HALF];

        FillSineHalfBuffer(second_half, AUDIO_FRAMES_PER_HALF);
        DSP_ProcessHalfBuffer(second_half, AUDIO_FRAMES_PER_HALF);
    }
}
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
