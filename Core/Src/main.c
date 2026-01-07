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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "FOC.h"
#include <stdlib.h>  // for atof
#include <string.h>
#include "LowPassFilter.h"
#include "pid.h"
#include "InlineCurrent.h"
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

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

void VOFA_JustFloat_Send(float data_1, float data_2,float data_3)
{
    // 3个电压数据数组
    float data[3] = {data_1,data_2,data_3};
    
    // 发送浮点数组数据（小端格式）
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)data, sizeof(float)*2);
    
    // 发送帧尾
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit_DMA(&huart3, tail, 4);
}

float ctrl_target = 0.0f;
#define RX_BUFFER_SIZE 128
uint8_t rx_buffer[RX_BUFFER_SIZE];   

/**
 * @brief 定时器周期回调函数
 * @note  每1ms自动调用一次，在这里执行FOC控制
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim2)
    {
        // FOC 控制核心代码
        AS5600_Update(&AngleSensor);        // 更新角度传感器
        switch (ctrl_mode)
        {
            case FOC_MODE_CURRENT:
                set_Foc_current(ctrl_target);
                break;

            case FOC_MODE_VELOCITY:
                set_Foc_speed(ctrl_target);
                break;
            
            case FOC_MODE_ANGLE:
                set_Foc_angle(ctrl_target);
                break;

            case FOC_MODE_IDLE:
            default:
                setTorque(0, 0, _electricalAngle()); // 零力矩/空闲
                break;
				}
				
    }
}
		// 解析指令
    // 格式: "vel_3.5" --速度模式, 3.5 rad/s
    // 格式: "ang_1.57" --角度模式, 1.57 rad
    // 格式: "cur_0.5" --电流模式, 0.5 A
    // 格式: "stop" --停
void Process_Serial_Command(uint8_t *buf, uint16_t len)
{
    // 确保字符串结束符，防止越界
    if (len >= RX_BUFFER_SIZE) len = RX_BUFFER_SIZE - 1;
    buf[len] = '\0';
    
    char *cmd_ptr = (char *)buf;
    float val = 0.0f;

    if (strncmp(cmd_ptr, "vel_", 4) == 0) {
        val = atof(cmd_ptr + 4); 
        ctrl_mode = FOC_MODE_VELOCITY;
        ctrl_target = val;
      //printf("Mode: Velocity,Target: %.2f\r\n", val);
    }
    else if (strncmp(cmd_ptr, "ang_", 4) == 0) {
        val = atof(cmd_ptr + 4);
        ctrl_mode = FOC_MODE_ANGLE;
        ctrl_target = val;
			//printf("Mode: Angle,Target: %.2f\r\n", val);
    }
    else if (strncmp(cmd_ptr, "cur_", 4) == 0) {
        val = atof(cmd_ptr + 4);
        ctrl_mode = FOC_MODE_CURRENT;
        ctrl_target = val;
      //printf("Mode: Current,Target: %.2f\r\n", val);
    }
    else if (strncmp(cmd_ptr, "stop", 4) == 0) {
        ctrl_mode = FOC_MODE_IDLE;
        ctrl_target = 0.0f;
        setTorque(0, 0, _electricalAngle()); 
       // printf("Mode: Stopped\r\n");
    }
}

/**
  * @brief 扩展接收事件回调函数
  * @param huart 串口句柄
  * @param Size  接收到的数据字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        // 1. 处理接收到的数据
        Process_Serial_Command(rx_buffer, Size);

        // 2. 重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_SIZE);
				__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
    }
}


/* 修改ADC注入转换完成回调函数 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	
    if (hadc->Instance == ADC1)
    {
        // 读取ADC原始值并转换为电压
        uint32_t adc1_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t adc2_raw = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        
        float u_a = InlineCurrent_ADCToVoltage(adc1_raw);
        float u_b = InlineCurrent_ADCToVoltage(adc2_raw);
			
        InlineCurrent_GetPhaseCurrents(&CurrentSensor, u_a, u_b);
    }
}


/* USER CODE END 0 */

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);
	
	//启动串口DMA空闲接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
	
	//初始化电流传感器
  InlineCurrent_Init(&CurrentSensor, 0.02f, 50.0f);  
	
  //校准电流传感器
  printf("Start calibrating the current sensor...\r\n");
  InlineCurrent_CalibrateOffsets(&CurrentSensor);
  printf("Calibration complete: OffsetA=%.3fV, OffsetB=%.3fV\r\n", 
         CurrentSensor.offset_a, CurrentSensor.offset_b);		 
	HAL_Delay(500);	 
	
	//初始化FOC	
	FOC_Init_Simple(12.0f,7,1);
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			static uint32_t last_print_time = 0;
      
      if (HAL_GetTick() - last_print_time >= 10)
      {
          float actual_val = 0.0f;
					switch (ctrl_mode)
          {
              case FOC_MODE_VELOCITY:
                  // 获取当前速度 (rad/s)
                  actual_val = AS5600_GetVelocity(&AngleSensor) * DIR;
                  break;
                  
              case FOC_MODE_ANGLE:
                  // 获取当前累计角度 (rad)
                  actual_val = AS5600_GetAccumulateAngle(&AngleSensor) * DIR;
                  break;
                  
              case FOC_MODE_CURRENT:
                  // 获取当前 Q 轴电流 (Iq)
                  actual_val = I_q; 
                  break;
                  
              default:
                  actual_val = 0.0f;
                  break;
          }
				  if (ctrl_mode != FOC_MODE_IDLE)
          {
              printf("%.3f,%.3f\r\n", ctrl_target, actual_val);
          }
          
          last_print_time = HAL_GetTick();
			}
			
    // 每10ms发送一次数据到VOFA+（100Hz刷新率）
//    static uint32_t last_vofa_send = 0;
//    if(HAL_GetTick() - last_vofa_send >= 10) {
//        VOFA_JustFloat_Send_DMA(Ua, Ub, Uc);
//        last_vofa_send = HAL_GetTick();
//    }
			   
//		   // 在主循环中定期打印电流值
//        static uint32_t last_print = 0;
//        if(HAL_GetTick() - last_print >= 50) {  // 每50ms打印一次
//            printf("Ia=%.3fA, Ib=%.3fA, Ic=%.3fA\r\n", 
//                   CurrentSensor.current_a, 
//                   CurrentSensor.current_b, 
//                   CurrentSensor.current_c);
//            last_print = HAL_GetTick();
//        }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
