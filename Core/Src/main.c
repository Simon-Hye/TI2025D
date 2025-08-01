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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "74HC4051.h"
#include "fdc2214.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_IDLE,
    MODE_CAL_1M_SFTP,   // 1m SFTP
    MODE_CAL_50M_UTP,   // 50m UTP
    MODE_DE,
    MODE_SE,
    MODE_DEBUG
  } SystemMode;


typedef enum {
    DE_DETECT_ORDER,      //
    DE_DETECT_TYPE,   // UTP/SFTP
    DE_MEASURE_R
  } DE_State;


typedef enum {
    SE_DETECT_SHORT,
    SE_DETECT_TYPE,
    SE_MEASURE_LENGTH,
    SE_LOCATE_SHORT
  } SE_State;


typedef struct {
    uint8_t is_crossed;  // 0=straight, 1=crossover
    uint8_t order[8];
}PairOrder;


typedef struct {
    PairOrder pair_order;
    uint8_t type;       // 0=UTP, 1=SFTP
    float   R;
} DE_MeasCtx;


typedef struct {
    uint8_t is_shorted;          // 0=no short, 1=short detected
    uint8_t type;       // 0=UTP, 1=SFTP
    float   len;
    float   short_pos;
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


DE_MeasCtx g_de_ctx;
SE_MeasCtx g_se_ctx;

uint8_t usart1_rx_buf[RX_BUF_LEN];
volatile uint8_t usart1_idle_flag = 0;
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
void SE_detect_type(SE_MeasCtx *ctx);
void SE_measure_length(SE_MeasCtx *ctx);
void SE_measure_shortpos(SE_MeasCtx *ctx);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  g_system_mode = MODE_IDLE;

  HC4051_init(AS0_GPIO_Port, AS0_Pin,
                AS1_GPIO_Port, AS1_Pin,
                AS2_GPIO_Port, AS2_Pin,
                A_E_GPIO_Port, A_E_Pin);    //TX
  HC4051_init(BS0_GPIO_Port, BS0_Pin,
                BS1_GPIO_Port, BS1_Pin,
                BS2_GPIO_Port, BS2_Pin,
                B_E_GPIO_Port, B_E_Pin);    //RX

    FDC2214_Init(hi2c1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1,usart1_rx_buf,RX_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     switch (g_system_mode) {
         case MODE_DEBUG: {
             SE_measure_length(&g_se_ctx);
             printf("%4f m\r\n", g_se_ctx.len);

             break;
         }
        case MODE_IDLE: {
            if (usart1_idle_flag) {

                HAL_UART_Transmit(&huart2, usart1_rx_buf, RX_BUF_LEN, 1000);
                HAL_Delay(10);

                uint8_t cmd = usart1_rx_buf[0];
                if (cmd == 0x01) {
                    g_system_mode = MODE_CAL_1M_SFTP;
                } else if (cmd == 0x02) {
                    g_system_mode = MODE_CAL_50M_UTP;
                } else if (cmd == 0x03) {
                    g_system_mode  = MODE_DE;
                    g_de_substate  = DE_DETECT_ORDER;
                } else if (cmd == 0x04) {
                    g_system_mode  = MODE_SE;
                    g_se_substate  = SE_DETECT_SHORT;
                }
                usart1_idle_flag = 0;
                memset(usart1_rx_buf,0,RX_BUF_LEN);
            }
            break;
        }

        case MODE_CAL_1M_SFTP:
            calibrate_1m_sftp();
            g_system_mode = MODE_IDLE;
            cal1_done = 1;
             printf("Jiao.t0.txt=\" Ready1 \"\xff\xff\xff");
             printf("Jiao.t0.txt=\" Ready1 \"\xff\xff\xff\r\n");
            break;

        case MODE_CAL_50M_UTP:
            calibrate_50m_utp();
            g_system_mode = MODE_IDLE;
            cal2_done = 1;
             printf("Jiao.t1.txt=\" Ready2 \"\xff\xff\xff");
             printf("Jiao.t1.txt=\" Ready2 \"\xff\xff\xff\r\n");
            break;

        case MODE_DE:
            switch (g_de_substate) {
                case DE_DETECT_ORDER:
                    DE_detect_order(&g_de_ctx);
                    if (g_de_ctx.pair_order.is_crossed) {
                        printf("Shuang.t0.txt=\" 交叉 \"\xff\xff\xff");
                        printf("Shuang.t0.txt=\" 交叉 \"\xff\xff\xff\r\n");
                    }
                    else if (!g_de_ctx.pair_order.is_crossed) {
                        printf("Shuang.t0.txt=\" 直连 \"\xff\xff\xff");
                        printf("Shuang.t0.txt=\" 直连 \"\xff\xff\xff\r\n");
                    }
                    g_de_substate = DE_DETECT_TYPE;
                    break;
                case DE_DETECT_TYPE:
                    DE_detect_type(&g_de_ctx);
                    if (g_de_ctx.type == 0) {
                        printf("Shuang.t1.txt=\" UTP \"\xff\xff\xff");
                        printf("Shuang.t1.txt=\" UTP \"\xff\xff\xff\r\n");
                    }
                    else if (g_de_ctx.type == 1) {
                        printf("Shuang.t1.txt=\" SFTP \"\xff\xff\xff");
                        printf("Shuang.t1.txt=\" SFTP \"\xff\xff\xff\r\n");
                    }
                    g_de_substate = DE_MEASURE_R;
                    break;
                case DE_MEASURE_R:
                    DE_measure_R(&g_de_ctx);
                    printf("Shuang.t2.txt=\" %.5g Ω \"\xff\xff\xff",g_de_ctx.R);
                    printf("Shuang.t2.txt=\" %.5g Ω \"\xff\xff\xff\r\n",g_de_ctx.R);
                    g_system_mode = MODE_IDLE;
                    g_de_substate = DE_DETECT_ORDER;
                    break;
            }
            break;

        case MODE_SE:
            switch (g_se_substate) {
                case SE_DETECT_SHORT:
                    SE_detect_short(&g_se_ctx);
                    if (g_se_ctx.is_shorted == 1) {
                        printf("Dan.t0.txt=\" NaN \"\xff\xff\xff");
                        printf("Dan.t0.txt=\" NaN \"\xff\xff\xff\r\n");
                        printf("Dan.t1.txt=\" 短路 \"\xff\xff\xff");
                        printf("Dan.t1.txt=\" 短路 \"\xff\xff\xff\r\n");
                        g_se_substate = SE_LOCATE_SHORT;
                    }
                    else {
                        printf("Dan.t1.txt=\" 无短路 \"\xff\xff\xff");
                        printf("Dan.t1.txt=\" 无短路 \"\xff\xff\xff\r\n");
                        g_se_substate = SE_DETECT_TYPE;
                    }
                    break;
                case SE_DETECT_TYPE:
                    SE_detect_type(&g_se_ctx);
                    g_se_substate = SE_MEASURE_LENGTH;
                    break;
                case SE_MEASURE_LENGTH:
                    SE_measure_length(&g_se_ctx);
                    printf("Dan.t0.txt=\" %.5g m \"\xff\xff\xff",g_se_ctx.len);
                    printf("Dan.t0.txt=\" %.5g m \"\xff\xff\xff\r\n",g_se_ctx.len);
                    g_se_substate = SE_LOCATE_SHORT;
                    break;
                case SE_LOCATE_SHORT:
                    SE_measure_shortpos(&g_se_ctx);
                    printf("Dan.t2.txt=\" %.5g m \"\xff\xff\xff",g_se_ctx.short_pos);
                    printf("Dan.t2.txt=\" %.5g m \"\xff\xff\xff\r\n",g_se_ctx.short_pos);
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
void calibrate_1m_sftp(void) {
    // TODO: ?????????
}
void calibrate_50m_utp(void) {
    // TODO: ?????????
}

void DE_detect_order(DE_MeasCtx *ctx) {
    uint8_t is_crossed_flag = 0;
    HAL_GPIO_WritePin(Z_TX_GPIO_Port,Z_TX_Pin,GPIO_PIN_SET);
    for (uint8_t i = 0; i < 8; i++) {
        ctx->pair_order.order[i] = 0xFF;
        HC4051_open(0,i);
        HAL_Delay(10);
        for (uint8_t j = 0; j < 8; j++) {
            HC4051_open(1,j);
            HAL_Delay(10);
            if (HAL_GPIO_ReadPin(Z_RX_GPIO_Port,Z_RX_Pin) == GPIO_PIN_SET) {
                ctx->pair_order.order[i] = j;
                if (i != j) {
                    is_crossed_flag = 1; // crossed
                }
                break;
            }
        }
    }
    ctx->pair_order.is_crossed = is_crossed_flag;
    HC4051_close(0);
    HC4051_close(1);
    HAL_GPIO_WritePin(Z_TX_GPIO_Port, Z_TX_Pin, GPIO_PIN_RESET);
}
void DE_detect_type(DE_MeasCtx *ctx) {
    // TODO: ???? UTP/SFTP?????? ctx->type
    ctx->type = 0;
}
void DE_measure_R(DE_MeasCtx *ctx) {
    // TODO: ???????????k???? ctx->R
    ctx->R = 0.0f;
}

void SE_detect_short(SE_MeasCtx *ctx) {
    // TODO: ??????????? ctx->is_shorted
    ctx->is_shorted = 0;
}
void SE_detect_type(SE_MeasCtx *ctx) {
    // TODO: ???? UTP/SFTP?????? ctx->type
    ctx->type = 0;
}
void SE_measure_length(SE_MeasCtx *ctx) {
    float FDC2214_Value = 0;
    float len;
    FDC2214_Value = FDC2214_Cap_Calculate(0);
    len = FDC2214_Value * 0.025986 - 1.39567; //to be calibrated
    ctx->len = len;
}
void SE_measure_shortpos(SE_MeasCtx *ctx) {
    // TODO: TDR ?????????????? ctx->short_pos
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
