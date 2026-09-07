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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "string.h"
#include "wrapper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct __attribute__((packed)) IMUPacket
{
  uint16_t header;
  uint64_t timestamp_us;      // IMU采样时间
  uint64_t frame_id;
  float acc[3];
  float gyro[3];
  uint8_t trigger_flag;       // STM32触发相机拍照的命令
  uint8_t strb_flag;          // 本帧是否包含STRB信号
  uint64_t strb_timestamp_us; // STRB信号到达的时刻
  uint16_t crc;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_GPIO_Port  GPIOA
#define TRIG_Pin        GPIO_PIN_11
#define STRB_Pin        GPIO_PIN_12

#define LED_GPIO_Port   GPIOC
#define LED_Pin         GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 双缓冲数据包
struct IMUPacket packet_buffer[2];
volatile uint8_t current_buffer = 0;  // 当前正在写入的缓冲区索引
volatile uint8_t data_ready = 0;      // 有新数据标志

// 触发相关
volatile uint32_t frame_counter = 0;  // 总帧数，一直累加
volatile uint8_t trigger_pending = 0;
volatile uint64_t trigger_timestamp_us = 0;

// IMU 数据缓存（从 wrapper 获取）
float accel_data[3];
float gyro_data[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 辅助函数：计算CRC16 (CCITT)
uint16_t crc16(uint8_t* data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// 通过UART发送数据包（在主循环中调用）
void send_imu_packet(struct IMUPacket* packet)
{
  uint8_t* byte_ptr = (uint8_t*)packet;
  uint16_t packet_size = sizeof(struct IMUPacket);

  // 计算CRC并填充
  packet->crc = crc16(byte_ptr, packet_size - 2);

  // 使用DMA发送（非阻塞）
  HAL_UART_Transmit_DMA(&huart1, byte_ptr, packet_size);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  packet_buffer[0].header = 0xA5A5;
  packet_buffer[1].header = 0xA5A5;
  packet_buffer[0].frame_id = 0;
  packet_buffer[1].frame_id = 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(72);  // 72MHz 系统时钟
  // TIM1_CH4: PA11 输出 30 Hz、500 us 的硬件触发脉冲
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // TIM2: 约 1 kHz 采集并打包 IMU 数据
  HAL_TIM_Base_Start_IT(&htim2);
  // 初始化 IMU（通过 wrapper）
  if (mpu6050_init_wrapper() != HAL_OK) {
    return 0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (data_ready)
    {
      __disable_irq();
      uint8_t buffer_to_send = current_buffer ^ 1;
      data_ready = 0;
      __enable_irq();

      send_imu_packet(&packet_buffer[buffer_to_send]);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
volatile uint64_t strb_timestamp_us = 0;
volatile uint8_t strb_triggered = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == STRB_Pin)
  {
    // 相机真正曝光的时刻！立即记录时间戳
    strb_timestamp_us = DWT_GetTimeline_us();
    strb_triggered = 1;

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    // TIM1 更新事件对应 PA11 PWM 周期起点
    trigger_timestamp_us = DWT_GetTimeline_us();
    trigger_pending = 1;
    frame_counter++;
    return;
  }

  if (htim->Instance == TIM2)
  {
    uint8_t write_idx = current_buffer;
    struct IMUPacket* pkt = &packet_buffer[write_idx];

    // 读取 IMU 数据
    mpu6050_get_data_wrapper(accel_data, gyro_data);

    // 填充数据包
    pkt->timestamp_us = DWT_GetTimeline_us();      // IMU采样时间
    pkt->frame_id = frame_counter;
    memcpy(pkt->acc, accel_data, sizeof(float) * 3);
    memcpy(pkt->gyro, gyro_data, sizeof(float) * 3);

    if (strb_triggered)
    {
      // 把 STRB 时间戳放入数据包（可以用一个新增字段）
      pkt->strb_timestamp_us = strb_timestamp_us;  // 需要加这个字段
      pkt->strb_flag = 1;                          // 标记本帧有 STRB

      strb_triggered = 0;  // 清除标记
    }
    else
    {
      pkt->strb_flag = 0;
    }

    if (trigger_pending)
    {
      pkt->trigger_flag = 1;
      trigger_pending = 0;
    }
    else
    {
      pkt->trigger_flag = 0;
    }

    current_buffer = write_idx ^ 1;
    data_ready = 1;
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
