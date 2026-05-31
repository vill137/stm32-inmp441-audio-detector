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
#include "dma.h"
#include "i2s.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// 硬件驱动模块头文件，包含所有外设和FFT接口
#include "hardware.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== 全局变量定义 ====================
// printf调试标志（保留未使用）
volatile uint8_t printf_flag=0;
// I2S DMA传输完成计数器（调试用）
volatile unsigned dma_cnt;
// 拼接并符号扩展后的32位音频样本
volatile int val32;
// I2S DMA原始接收缓冲区（16位格式）
volatile uint32_t audio_buf[buffer_size];
// DMA传输计数器（保留未使用）
volatile uint32_t dma_count=0;
// 200-500Hz频段能量占比（0~100%）
float target_ratio=0.0f;
// 频谱峰值频率（保留未使用）
float freq=0;
// A加权声压级（dB SPL），变量名拼写错误但不影响功能
float sql_db=0.0f;

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
  MX_I2S2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	// 初始化1.69寸SPI LCD显示屏
	SPI_LCD_Init();
	// 设置LCD背景色为黑色
	LCD_SetBackColor(LCD_BLACK);
	// 设置LCD前景色为黄色
	LCD_SetColor(LCD_YELLOW);
	
	// 启动TIM4定时器（用于串口接收超时，调试用）
	HAL_TIM_Base_Start_IT(&htim4);
	// 启动TIM6定时器（保留未使用）
	HAL_TIM_Base_Start_IT(&htim6);
	
	// 启动I2S DMA接收，开始采集INMP441音频数据
	HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)audio_buf,buffer_size);
	
	// 初始化FFT处理模块
	fft_init();
	// FFT结果结构体指针（保留未使用）
	FFT_RESULT_T *fft_get_result;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// 当FFT缓冲区满时，处理一帧音频数据
		if(fft_ready){
			// 执行FFT变换和频谱分析
			fft_process();
			// 计算200-500Hz频段能量占比
			target_ratio=FFT_calculateTargetRatio();
			// 计算A加权声压级
			sql_db=FFT_CalculateSPL();
			// 清除FFT就绪标志，准备下一帧采集
			fft_ready=0;
		}
		
		// ==================== LCD显示逻辑 ====================
		char text[50];
		// 显示当前声压级
		sprintf(text,"NOISE:%.1fDB ",sql_db);
		LCD_DisplayString(26,100,text);
		
		// 显示200-500Hz频段能量占比
		sprintf(text,"NBR:%.1f %% ",target_ratio);
		LCD_DisplayString(26,150,text);
		
		// 噪声状态判断：分贝>65dB且占比>50%时判定为噪声超标
		if(sql_db>65&&target_ratio>50){
			sprintf(text,"Noisey  ");
		}
		else{
			sprintf(text,"Normal  ");
		}
		LCD_DisplayString(26,200,text);
		
		// 串口调试功能（已注释，生产环境关闭）
		//uart_proc();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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