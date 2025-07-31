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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "74HC4051.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_IDLE,          // ??????????
    MODE_CAL_1M_SFTP,   // 1m SFTP ??§µ?
    MODE_CAL_50M_UTP,   // 50m UTP ??§µ?
    MODE_DE,    // ?????
    MODE_SE     // ??????
  } SystemMode;

// ?????????
typedef enum {
    DE_DETECT_ORDER,      // ?§Ø???????
    DE_DETECT_TYPE,   // ?§Ø? UTP/SFTP
    DE_MEASURE_R   // ???????????
  } DE_State;

// ??????????
typedef enum {
    SE_DETECT_SHORT,        // ????¡¤
    SE_DETECT_TYPE,      // ?§Ø? UTP/SFTP
    SE_MEASURE_LENGTH,  // TDR ????
    SE_LOCATE_SHORT     // TDR ??¦Ë??¡¤??
  } SE_State;

// ??????
typedef struct {
    uint8_t is_crosed;  // 0=straight, 1=crossover
    uint8_t pair_order[8];
}PairOrder;

// ??????????
typedef struct {
    PairOrder pair_order;
    uint8_t type;       // 0=UTP, 1=SFTP
    float   R;    // ???????????
} DE_MeasCtx;

// ???????????
typedef struct {
    uint8_t is_shorted;          // 0=no short, 1=short detected
    uint8_t type;       // 0=UTP, 1=SFTP
    float   len;           // ???????????
    float   short_pos;        // ?????¡¤??¦Ë??
} SE_MeasCtx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile SystemMode    g_system_mode      = MODE_CAL_1M_SFTP;
volatile DE_State  g_de_substate  = DE_DETECT_ORDER;
volatile SE_State  g_se_substate  = SE_DETECT_SHORT;

volatile uint8_t cal1_done = 0;
volatile uint8_t cal2_done = 0;

// ?????while?§Ù???
DE_MeasCtx g_de_ctx;
SE_MeasCtx g_se_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void calibrate_1m_sftp(void);
void calibrate_50m_utp(void);

void DE_detect_order(DE_MeasCtx *ctx);
void DE_detect_type(DE_MeasCtx *ctx);
void DE_measure_R(DE_MeasCtx *ctx);

void SE_detect_short(SE_MeasCtx *ctx);
void SE_detect_type(SE_MeasCtx *ctx);   // ???????
void SE_measure_length(SE_MeasCtx *ctx);    //????????
void SE_measure_shortpos(SE_MeasCtx *ctx);

uint8_t usart1_receive(void);
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
  /* USER CODE BEGIN 2 */
    HC4051_init(S0_GPIO_Port, S0_Pin,
                S1_GPIO_Port, S1_Pin,
                S2_GPIO_Port, S2_Pin,
                E_GPIO_Port, E_Pin);
    HC4051_init(S0_GPIO_Port, S0_Pin,
                S1_GPIO_Port, S1_Pin,
                S2_GPIO_Port, S2_Pin,
                E_GPIO_Port, E_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     switch (g_system_mode) {
        case MODE_IDLE: {
            char cmd = usart1_receive();
            if (cmd == '1') {
                g_system_mode = MODE_CAL_1M_SFTP;
            } else if (cmd == '2') {
                g_system_mode = MODE_CAL_50M_UTP;
            } else if (cmd == '3') {
                g_system_mode  = MODE_DE;
                g_de_substate  = DE_DETECT_ORDER;
            } else if (cmd == '4') {
                g_system_mode  = MODE_SE;
                g_se_substate  = SE_DETECT_SHORT;
            }
            break;
        }

        // ???? §µ???1m SFTP ????
        case MODE_CAL_1M_SFTP:
            calibrate_1m_sftp();
            g_system_mode = MODE_IDLE;
            cal1_done = 1;
            break;

        // ???? §µ???50m UTP ????
        case MODE_CAL_50M_UTP:
            calibrate_50m_utp();
            g_system_mode = MODE_IDLE;
            cal2_done = 1;
            break;

        // ???? ????? ????
        case MODE_DE:
            switch (g_de_substate) {
                case DE_DETECT_ORDER:
                    DE_detect_order(&g_de_ctx);
                    g_de_substate = DE_DETECT_TYPE;
                    break;
                case DE_DETECT_TYPE:
                    DE_detect_type(&g_de_ctx);
                    g_de_substate = DE_MEASURE_R;
                    break;
                case DE_MEASURE_R:
                    DE_measure_R(&g_de_ctx);
                    g_system_mode = MODE_IDLE;
                    g_de_substate = DE_DETECT_ORDER;
                    break;
            }
            break;

        // ???? ?????? ????
        case MODE_SE:
            switch (g_se_substate) {
                case SE_DETECT_SHORT:
                    SE_detect_short(&g_se_ctx);
                    if (g_se_ctx.is_shorted == 1) {
                        g_system_mode = MODE_IDLE;
                        g_se_substate = SE_DETECT_SHORT;
                    }
                    else {
                        g_se_substate = SE_DETECT_SHORT;
                    }
                    break;
                case SE_DETECT_TYPE:
                    SE_detect_type(&g_se_ctx);
                    g_se_substate = SE_MEASURE_LENGTH;
                    break;
                case SE_MEASURE_LENGTH:
                    SE_measure_length(&g_se_ctx);
                    g_se_substate = SE_LOCATE_SHORT;
                    break;
                case SE_LOCATE_SHORT:
                    SE_measure_shortpos(&g_se_ctx);
                    g_system_mode = MODE_IDLE;
                    break;
            }
            break;

        default:
            g_system_mode = MODE_IDLE;
            break;
        } // end switch

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ?? main.c ???¦Â???????????? tests.c ????????? */
uint8_t usart1_receive(void) {
    // TODO: ????? USART1?????????? '\0'
    return '\0';
}

void calibrate_1m_sftp(void) {
    // TODO: ???§µ????
}
void calibrate_50m_utp(void) {
    // TODO: ???§µ????
}

void DE_detect_order(DE_MeasCtx *ctx) {
    // TODO: ?§Ø????/???—¨???? ctx->pair_order

}
void DE_detect_type(DE_MeasCtx *ctx) {
    // TODO: ?§Ø? UTP/SFTP?????? ctx->type
    ctx->type = 0;
}
void DE_measure_R(DE_MeasCtx *ctx) {
    // TODO: ??????????Ñk???? ctx->R
    ctx->R = 0.0f;
}

void SE_detect_short(SE_MeasCtx *ctx) {
    // TODO: ??¡¤??????? ctx->is_shorted
    ctx->is_shorted = 0;
}
void SE_detect_type(SE_MeasCtx *ctx) {
    // TODO: ?§Ø? UTP/SFTP?????? ctx->type
    ctx->type = 0;
}
void SE_measure_length(SE_MeasCtx *ctx) {
    // TODO: TDR ????????? ctx->len
    ctx->len = 0.0f;
}
void SE_measure_shortpos(SE_MeasCtx *ctx) {
    // TODO: TDR ??¦Ë??¡¤?????? ctx->short_pos
    ctx->short_pos = 0.0f;
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
