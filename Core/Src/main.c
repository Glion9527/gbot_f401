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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 头文件声明“有哪些函数可以调用”，具体函数实现在同名 .c 中。
 * ROBOT_APP_MODE 是编译期选择：模式 7 编入遥控入口，其余模式保留本地测试。
 * #if 与运行时 if 不同；修改模式后需要重新编译、下载才能改变固件行为。 */
#include "motor4.h"
#include "mecanum.h"
#include "robot_app.h"
#if ROBOT_APP_MODE == 7
#include "remote_control.h"
#endif
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
/* 按键、行驶路线和串口状态由 robot_app.c 管理，电机状态由 motor4.c 管理。
 * main.c 只把这些模块连接起来，因此不再另外保存一套电机控制变量。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 阅读本工程可按以下顺序理解：
 * 1. main.c：上电初始化、启动定时器、反复调用应用程序。
 * 2. robot_app.h/.c：按键、路线阶段、启动延迟、运行时间和串口日志。
 * 3. mecanum.h/.c：将前进/横移/转弯要求分解为四个轮子的目标 RPM。
 * 4. motor4.h/.c：读编码器、计算转速、PI 调速、输出 PWM 和方向。
 * 5. tim.c / gpio.c / usart.c：CubeMX 生成的外设配置。
 *
 * 主循环决定“什么时候、往哪里走”；TIM9 每 10ms 中断一次，负责“怎样调速”。
 * PWM 波形和编码器计数由定时器硬件完成，不需要 CPU 对每个脉冲逐个操作。
 * 自写内容尽量放在 USER CODE 区域；CubeMX 开启保留用户代码后可保留这些区域。
 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 上电流程：HAL 基础初始化 -> 系统时钟 -> GPIO/各定时器/串口配置
   * -> 电机模块初始化并启动 PWM、编码器 -> 启动 10ms 控制中断 -> 应用初始化。
   * 配置外设不等于让小车运动；遥控模式收到握手、使能和有效速度指令后才会运动。 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* HAL_Init 建立 HAL 基础环境和 SysTick 毫秒计时；它不启动电机。
   * 后面的 MX_xxx_Init 配置外设，Motor4_Init 和 Start_IT 才负责启动对应硬件。
   * HAL_OK 表示初始化成功；其他返回值交给 Error_Handler 统一撤销驱动。 */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 当前时钟使用芯片内部 HSI 16MHz，不依赖 ST-LINK 提供外部时钟：
   * PLL 输入 = 16MHz / M(16) = 1MHz；VCO = 1MHz * N(336) = 336MHz；
   * SYSCLK = 336MHz / P(4) = 84MHz，AHB 不分频，所以 CPU/HCLK 为 84MHz。
   * APB1 分频 2 后 PCLK1=42MHz，APB2 不分频所以 PCLK2=84MHz。
   * 本配置下 APB1 定时器时钟自动乘 2，APB2 定时器不再乘 2，均为 84MHz。
   * 时钟改变后，PWM 频率、控制周期和串口波特率都要重新核对。 */
  /* 术语对应：HSI=高速内部振荡源，PLL=锁相环，VCO=PLL 内部振荡器；
   * M/N/P 分别是输入分频、内部倍频、输出分频，336MHz 不是 CPU 频率。
   * SYSCLK 是系统选中的主时钟；AHB/APB 是内部总线，HCLK/PCLK 是对应时钟。
   * APB1 上的 USART2 使用 PCLK1=42MHz；定时器分支才按当前规则得到 84MHz。
   * 这里的说明属于注释，真正设置寄存器的是 SystemClock_Config 函数。 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  /* motor4 持有四路电机状态，先完成初始化，避免定时中断读到未就绪的数据。 */
  /* 首先保持 STBY 关闭，启动 PWM 和编码器读取能力，等待应用下发非零目标。
   * 初始化不等于使能遥控；连接、使能和速度目标由 remote_control 分开管理。 */
  if (Motor4_Init() != HAL_OK) {
      Error_Handler();
  }
  /* TIM9 作为全车共用的 10ms 节拍。启动前清零计数器和旧更新标志，
   * 避免把初始化时留下的更新标志误当作一次新的控制周期。 */
  __HAL_TIM_SET_COUNTER(&htim9, 0);
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim9) != HAL_OK) {
      Error_Handler();
  }
  /* 应用从停止状态开始；模式7建立遥控接收状态，其余模式初始化按键/路线。
   * 遥控的结构化T遥测和本地测试的READY文本由各自模式的主循环流程输出。 */
  RobotApp_Init();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 每次轮询只处理当前该做的事，不用长延时等待 5秒/12秒路线结束。
     * 这样运动期间仍能检查按键、故障和超时。HAL_Delay(1) 只是约 1ms 的让步；
     * 它不产生电机的 10ms 控制周期，控制周期由 TIM9 中断独立提供。 */
    RobotApp_Poll();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 中断调用链：TIM9 更新事件 -> TIM1_BRK_TIM9_IRQHandler（stm32f4xx_it.c）
 * -> HAL_TIM_IRQHandler -> 本回调 -> 四路编码器测速、PI 调速和故障检查。
 * TIM1 和 TIM9 共用中断入口，故通过 Instance 判断，只让 TIM9 驱动控制节拍。
 * 此处在中断环境中执行，应短小，不能加串口打印、长延时或路线等待。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9) {
#if ROBOT_APP_MODE == 7
        /* 指令超时和本地停止先于本周期PI输出，不依赖主循环是否阻塞。 */
        /* 主循环负责接收和解析“下一条命令”，本中断独立检查“已有命令是否过期”。
         * 若许可失效，先停止，再让 motor4 处理停止后的状态，避免继续输出旧目标。 */
        RemoteControl_Tick10ms();
#endif
        Motor4_ControlTick10ms();
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
  /* 初始化等关键 HAL 操作失败会走到这里：先拉低共享 STBY，撤销四轮驱动，
   * 再关闭中断并停留，等待排查或复位；普通轮询故障由 motor4/robot_app 处理。 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  __disable_irq();
  while (1) {
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
