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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "FOC.h"
#include <stdlib.h>  // for atof
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
//        setTorque(3, _electricalAngle());   // 设置力矩
				//set_Foc_angle(10);
				//set_Foc_speed(10);//高级定时器mode3跟普通的up有啥区别？
//		
    }
}

// 全局变量定义
#define UART_RX_BUFFER_SIZE 32
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_data;
volatile uint8_t uart_data_ready = 0;

/**
  * @brief 串口中断回调函数
  * @param 调用回调函数的串口
  * @note  串口每次收到数据以后都会关闭中断，如需重复使用，必须再次开启
  * @retval None
  */  

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        static uint8_t rx_index = 0;
        
        // 接收字符
        if (uart_rx_data == '\n' || uart_rx_data == '\r')
        {
            // 遇到换行符，解析数据
            if (rx_index > 0)
            {
                uart_rx_buffer[rx_index] = '\0';
                serial_motor_target = atof((char*)uart_rx_buffer);
								printf("m0_angle:%.3f\r\n",serial_motor_target);
                rx_index = 0;
            }
        }
        else if (rx_index < UART_RX_BUFFER_SIZE - 1)
        {
            // 存储字符
            uart_rx_buffer[rx_index++] = uart_rx_data;
        }
        else
        {
            // 缓冲区溢出，重置
            rx_index = 0;
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart3, &uart_rx_data, 1);
    }
}

void VOFA_JustFloat_Send(float ua, float ub, float uc)
{
    // 3个电压数据数组
    float data[3] = {ua, ub, uc};
    
    // 发送浮点数组数据（小端格式）
    HAL_UART_Transmit(&huart3, (uint8_t *)data, sizeof(float) * 3, 1000);
    
    // 发送帧尾
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart3, tail, 4, 1000);
}

// 全局变量定义
InlineCurrent_T CurrentSensor;

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
	
	//初始化电流传感器
  InlineCurrent_Init(&CurrentSensor, 0.02f, 10.0f);  
  // 校准电流传感器
  printf("Start calibrating the current sensor...\r\n");
  InlineCurrent_CalibrateOffsets(&CurrentSensor);
  printf("Calibration complete: OffsetA=%.3fV, OffsetB=%.3fV\r\n", 
         CurrentSensor.offset_a, CurrentSensor.offset_b);		 
	HAL_Delay(1000);	 
	
	//初始化FOC	
	FOC_Init_Simple(12.0f,7,1);
    
	// 启动串口接收中断
  HAL_UART_Receive_IT(&huart3, &uart_rx_data, 1);

  setPwm(6, 0, 0);
  HAL_Delay(300);
  printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));

  setPwm(0, 6, 0);
  HAL_Delay(300);
   printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));

  setPwm(0, 0, 6);
  HAL_Delay(300);
  printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));
  setPwm(0, 0, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		float vel_m1=AS5600_GetVelocity(&AngleSensor);
//		float vel=10;
//		printf("%.2f,%.2f\r\n", vel, vel_m1);
		
//			float angle=serial_motor_target;
//      float angle_m1=AS5600_GetAccumulateAngle(&AngleSensor);
//			printf("%.2f,%.2f\r\n", 3.14, angle_m1);
//		
//		    float current = AS5600_GetAccumulateAngle(&AngleSensor);
//        float velocity = AS5600_GetVelocity(&AngleSensor);
//        printf("angle:%.2f, vel:%.2f\r\n", current, velocity);
//				HAL_Delay(10);
//				printf("integral: %.2f\r\n", pid_angle.integral);
		
		
    // 每10ms发送一次数据到VOFA+（100Hz刷新率）
//    static uint32_t last_vofa_send = 0;
//    if(HAL_GetTick() - last_vofa_send >= 10) {
//        VOFA_JustFloat_Send(Ua, Ub, Uc);
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


//// --- 原始值相位映射测试代码 ---
//// 确保主输出使能 (MOE)
////__HAL_TIM_MOE_ENABLE(&htim1); 

//static int test_step = 0;
//static uint32_t last_step_time = 0;

//if (HAL_GetTick() - last_step_time > 1000) // 每1秒切换一个状态
//{
//    test_step++;
//    if (test_step > 3) test_step = 0;
//    last_step_time = HAL_GetTick();

//    // 状态切换
//    switch (test_step)
//    {
//        case 0: // 全停 (All OFF)
//            setPwm(0, 0, 0);
//            printf("\r\n--- STOP (0,0,0) ---\r\n");
//            break;
//        case 1: // A相拉高 (A+, B-, C-)
//            setPwm(3.0f, 0, 0); // 给3V电压（先小一点，防烧）
//            printf("\r\n--- Test Phase A (3V,0,0) ---\r\n");
//            break;
//        case 2: // B相拉高 (A-, B+, C-)
//            setPwm(0, 3.0f, 0);
//            printf("\r\n--- Test Phase B (0,3V,0) ---\r\n");
//            break;
//        case 3: // C相拉高 (A-, B-, C+)
//            setPwm(0, 0, 3.0f);
//            printf("\r\n--- Test Phase C (0,0,3V) ---\r\n");
//            break;
//    }
//}

//// 每100ms 打印一次 ADC 原始值
//static uint32_t last_print = 0;
//if (HAL_GetTick() - last_print > 100)
//{
//    // 直接读取注入组的寄存器值（最真实的数据）
//    // 注意：这里假设你ADC1转换的是rank1，ADC2转换的也是rank1
//    uint32_t raw_adc1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//    uint32_t raw_adc2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//    
//    // 打印格式：状态 | ADC1原始值 | ADC2原始值
//    // 正常静止时，值应该在 2048 附近 (如果是3.3V参考电压，中点1.65V)
//    printf("State %d | Raw1: %lu | Raw2: %lu\r\n", test_step, raw_adc1, raw_adc2);
//    
//    last_print = HAL_GetTick();
//}

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
