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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <stdbool.h>
#include "board_map.h"
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
uint32_t Encoder_i = 0;  // 编码器计数
uint32_t PWM_F = 0;  // PWM频率
bool PWM_E = 1;  // PWM使能 默认为1，不要修改
bool motor_direction = 0;  // 电机方向：0=正转，1=反转
volatile uint8_t direction_toggle_flag = 0;  // 方向切换标志位

volatile bool X_limit_left__flag = 0;  // X轴左限位标志位
volatile bool X_limit_right__flag = 0;  // X轴右限位标志位
volatile bool Y_limit_left__flag = 0;  // Y轴左限位标志位
volatile bool Y_limit_right__flag = 0;  // Y轴右限位标志位

volatile uint32_t X_tim2_tick_count = 0;  // X轴步数值(TIM2中断计数)
volatile uint32_t Y_tim2_tick_count = 0;  // Y轴步数值(TIM2中断计数)
volatile uint32_t X_target_tick_count = 0;  // 目标X轴步数值
volatile uint32_t Y_target_tick_count = 0;  // 目标Y轴步数值
volatile bool X_tim2_count_enable = 0;  // X轴计数使能
volatile bool Y_tim2_count_enable = 0;  // Y轴计数使能

volatile uint32_t X_tim2_tick_count_old = 0;  // X轴步数值最大的旧值
volatile uint32_t Y_tim2_tick_count_old = 0;  // Y轴步数值最大的旧值

bool Motor_action_flag = 0;  // 电机行动标志位
bool Motor_STEP_flag_X = 0;  // X轴电机步进标志位
bool Motor_STEP_flag_Y = 0;  // Y轴电机步进标志位

volatile uint32_t X_left_last_irq_ms = 0;   // X左限位上次中断时间
volatile uint32_t X_right_last_irq_ms = 0;  // X右限位上次中断时间
volatile uint32_t Y_left_last_irq_ms = 0;   // Y左限位上次中断时间
volatile uint32_t Y_right_last_irq_ms = 0;  // Y右限位上次中断时间

#define LIMIT_DEBOUNCE_MS 20U  // 限位开关软件消抖时间

#define X_tim2_tick_count_max 6452  // X轴计数最大值
#define Y_tim2_tick_count_max 9931  // Y轴计数最大值


#define SERVO_PULSE_MIN_US 500U    // 舵机最小控制脉宽(0.5ms, 对应最小角度端)
#define SERVO_PULSE_MAX_US 2500U   // 舵机最大控制脉宽(2.5ms, 对应最大角度端)
#define SERVO_PULSE_INIT_US 1500U  // 舵机上电初始脉宽(1.5ms, 通常对应中位角度)
#define SERVO_PICK_PLACE_ANGLE_DEG 172U  // 自动下棋时取放棋子的下探角度
volatile int16_t Servo_angle_deg = 90;  // 舵机角度(0~180度)，用于显示

#define K230_CENTER_X 134  // K230坐标系中心棋格X坐标
#define K230_CENTER_Y 76   // K230坐标系中心棋格Y坐标
#define K230_GRID_DELTA 25 // K230坐标系相邻棋格坐标差
#define STEP_GRID_DELTA 1130 // 我方步进坐标系相邻棋格步数差
#define TASK3_X_MIN_STEP 1362 // 题目三允许的最小X步进坐标
#define TASK3_X_MAX_STEP 6122 // 题目三允许的最大X步进坐标
#define TASK3_Y_MIN_STEP 2751 // 题目三允许的最小Y步进坐标
#define TASK3_Y_MAX_STEP 9299 // 题目三允许的最大Y步进坐标

bool X_Motor_direction = 0;  // X轴电机方向：0=正转，1=反转
bool Y_Motor_direction = 0;  // Y轴电机方向：0=正转，1=反转

volatile uint8_t MODS = 0;  // 模式 0调整模式 1X轴设置 2Y轴设置 3舵机调节 4自动下棋
volatile uint8_t MODS_old = 0;  // 模式旧值
#define MODS_MAX 4 // 模式最大值，循环使用

uint8_t rx_buf[1];
char uart_rx_buffer[64];
uint8_t uart_rx_idx = 0;
volatile uint8_t uart_task = 0;
volatile uint8_t uart_coord1 = 0;
volatile uint8_t uart_coord2 = 0;
volatile int16_t uart_cam_x = 0;
volatile int16_t uart_cam_y = 0;
volatile bool uart_data_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void X_Motor_test_while(void);// X轴, Y轴电机测试循环
void Motor_Direction(void);  // 根据限位标志位切换电机方向
void Servo_Control(void);// 舵机控制函数(编码器 -> TIM4_CH3 PWM占空比)
void Motor_MoveToStep(int motor_id,uint32_t step);//电机步数控制
void Mode1_X_ByEncoder(void);// 模式1: 编码器控制X轴步进
void Mode2_Y_ByEncoder(void);// 模式2: 编码器控制Y轴步进
void Mode3_ServoToggleAndBack(void);// 模式3: 编码器直接控制舵机
void Mode4_AutoChess(void); // 模式4: 串口接收任务自动下棋
void CameraCoord_ToStep(int16_t cam_x, int16_t cam_y, int32_t *x_step, int32_t *y_step); // K230坐标转步进坐标
//void ToggleMotorDirection(void);// 切换电机方向
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//OLED显示
void oled_while(void)
{
  //Encoder_i = __HAL_TIM_GET_COUNTER(&htim1);  // 读取编码器计数

  char buffer[20];
  char buffer_2[20];
  char buffer_3[20];
  char buffer_4[20];
  char buffer_5[20];
  //char buffer_6[20];


  if(MODS == 0) //调整模式
  {
    Servo_Control();  // 编码器控制舵机
    snprintf(buffer, sizeof(buffer), "Xcount:%lu", X_tim2_tick_count);
    snprintf(buffer_2, sizeof(buffer_2), "Ycount:%lu", Y_tim2_tick_count);
    snprintf(buffer_3, sizeof(buffer_3), "servo:%d deg", (int)Servo_angle_deg);
    snprintf(buffer_4, sizeof(buffer_4), "X_MAX:%lu",X_tim2_tick_count_old);
    snprintf(buffer_5, sizeof(buffer_5), "Y_MAX:%lu",Y_tim2_tick_count_old);


    OLED_NewFrame();
    OLED_PrintASCIIString(1,1,buffer,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,11,buffer_2,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,21,buffer_3,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,31,buffer_4,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,41,buffer_5,&afont8x6, OLED_COLOR_NORMAL);
  }
  else if(MODS == 1 || MODS == 2 || MODS == 4)
  {
    snprintf(buffer, sizeof(buffer), "Xcount:%lu", X_tim2_tick_count);
    snprintf(buffer_2, sizeof(buffer_2), "Ycount:%lu",Y_tim2_tick_count);

    OLED_NewFrame();
    OLED_PrintASCIIString(1,1,buffer,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,11,buffer_2,&afont8x6, OLED_COLOR_NORMAL);
  }
  else if(MODS == 3)
  {
    snprintf(buffer, sizeof(buffer), "servo:%d deg", (int)Servo_angle_deg);

    OLED_NewFrame();
    OLED_PrintASCIIString(1,1,buffer,&afont8x6, OLED_COLOR_NORMAL);
  }
  snprintf(buffer_4, sizeof(buffer_4), "MODS:%d", MODS);
  OLED_PrintASCIIString(1,55,buffer_4,&afont8x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();

  //HAL_Delay(25);
}

// GPIO外部部中断  
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now_ms = HAL_GetTick();

	if(GPIO_Pin == MX_KEY_A_Pin)  // X轴左限位: PB10
  {
    if ((now_ms - X_left_last_irq_ms) < LIMIT_DEBOUNCE_MS) return;
    X_left_last_irq_ms = now_ms;

    X_limit_left__flag = 1;// 设置X轴左限位触碰标志位

    if(MODS == 0) X_tim2_tick_count = 0;  //调整模式 X左限位触发后清零计数
    if(MODS == 1) X_tim2_tick_count = 0;  // X左限位触发后清零计数
  }
  else if(GPIO_Pin == MX_KEY_B_Pin)  // X轴右限位: PB1
  {
    if ((now_ms - X_right_last_irq_ms) < LIMIT_DEBOUNCE_MS) return;
    X_right_last_irq_ms = now_ms;

    X_limit_right__flag = 1;// 设置X轴右限位触碰标志位
    
    if(MODS == 0)
    {
      X_tim2_tick_count_old = X_tim2_tick_count;// 保存旧值
      //X_tim2_count_enable = 1;  // X右限位触发后开始计数
    }
    //if(MODS == 1) X_tim2_tick_count = X_tim2_tick_count_max;  // X右限位触发后最大计数
  }
  
  else if(GPIO_Pin == MY_KEY_A_Pin)  // Y轴左限位: PB0
  {
    if ((now_ms - Y_left_last_irq_ms) < LIMIT_DEBOUNCE_MS) return;
    Y_left_last_irq_ms = now_ms;

    Y_limit_left__flag = 1;// 设置Y轴左限位触碰标志位

    if(MODS == 0) Y_tim2_tick_count = 0;  // Y左限位触发后清零计数
    if(MODS == 1) Y_tim2_tick_count = 0;  // Y左限位触发后清零计数
  }
  else if(GPIO_Pin == MY_KEY_B_Pin)  // Y轴右限位: PA7
  {
    if ((now_ms - Y_right_last_irq_ms) < LIMIT_DEBOUNCE_MS) return;
    Y_right_last_irq_ms = now_ms;

    Y_limit_right__flag = 1;// 设置Y轴右限位触碰标志位

    if(MODS == 0)
    {
      Y_tim2_tick_count_old = Y_tim2_tick_count;// 保存旧值
      //Y_tim2_count_enable = 1;  // Y右限位触发后开始计数
    }
    //if(MODS == 1) Y_tim2_tick_count = Y_tim2_tick_count_max;  // Y右限位触发后最大计数
  }
  if(MODS == 0)
  {
    X_Motor_test_while();// X轴, Y轴电机测试循环
    Motor_Direction();  // 根据限位标志位切换电机方向
  }
  else if(MODS == 1)
  {
    
  }
}

// 根据全局变量切换电机方向
void Motor_Direction(void)
{
  if(X_Motor_direction == 0)
  {
    HAL_GPIO_WritePin(MX_DIR_GPIO_Port, MX_DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MX_DIR_GPIO_Port, MX_DIR_Pin, GPIO_PIN_RESET);
  }

  if(Y_Motor_direction == 0)
  {
    HAL_GPIO_WritePin(MY_DIR_GPIO_Port, MY_DIR_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MY_DIR_GPIO_Port, MY_DIR_Pin, GPIO_PIN_SET);
  }
}

// 电机初始化  
void Motor_init(void)
{
  HAL_GPIO_WritePin(MX_EN_GPIO_Port, MX_EN_Pin, GPIO_PIN_RESET);  // X轴电机
  HAL_GPIO_WritePin(MY_EN_GPIO_Port, MY_EN_Pin, GPIO_PIN_RESET);  // Y轴电机

  X_Motor_direction = 1;// X轴电机方向：0=正转，1=反转
  Y_Motor_direction = 1;// Y轴电机方向：0=正转，1=反转
}

//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(MODS == 0)//调整模式
  {
      if (htim->Instance == TIM2)
    {
      if (X_Motor_direction == 0)// 正方向计数增加，反方向计数减少
      {
        X_tim2_tick_count++;
      }
      else
      {
        if (X_tim2_tick_count > 0) X_tim2_tick_count--;
      }
      if (Y_Motor_direction == 0)// 正方向计数增加，反方向计数减少
      {
        Y_tim2_tick_count++;
      }
      else
      {
        if (Y_tim2_tick_count > 0) Y_tim2_tick_count--;
      }
      HAL_GPIO_WritePin(GPIOA, MX_STEP_Pin | MY_STEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      HAL_TIM_Base_Start_IT(&htim3);
    }
    else if (htim->Instance == TIM3)
    {
      HAL_GPIO_WritePin(GPIOA, MX_STEP_Pin | MY_STEP_Pin, GPIO_PIN_RESET);
      HAL_TIM_Base_Stop_IT(&htim3);
    }
  }
  else if(MODS == 1)//X轴设置
  {
     if (htim->Instance == TIM2)
    {
      if(X_target_tick_count == X_tim2_tick_count) Motor_STEP_flag_X = 0;
      else Motor_STEP_flag_X = 1;

      Motor_STEP_flag_Y = 0;  // 模式1仅控制X轴

      if(Motor_STEP_flag_X == 1)
      {
        Motor_Direction();
        HAL_GPIO_WritePin(GPIOA, MX_STEP_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
      }
    }
    else if (htim->Instance == TIM3)
    {
      if(Motor_STEP_flag_X == 1) 
      {
        HAL_GPIO_WritePin(GPIOA, MX_STEP_Pin, GPIO_PIN_RESET);
        Motor_STEP_flag_X = 0;
        if(X_Motor_direction == 0) X_tim2_tick_count++;
        else if (X_tim2_tick_count > 0U) X_tim2_tick_count--;
      }
      HAL_TIM_Base_Stop_IT(&htim3);
    }
  }
  else if(MODS == 2)//Y轴设置
  {
     if (htim->Instance == TIM2)
    {
      Motor_STEP_flag_X = 0;  // 模式2仅控制Y轴

      if(Y_target_tick_count == Y_tim2_tick_count) Motor_STEP_flag_Y = 0;
      else Motor_STEP_flag_Y = 1;

      if(Motor_STEP_flag_Y == 1)
      {
        Motor_Direction();
        HAL_GPIO_WritePin(GPIOA, MY_STEP_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
      }
    }
    else if (htim->Instance == TIM3)
    {
      if(Motor_STEP_flag_Y == 1)
      {
        HAL_GPIO_WritePin(GPIOA, MY_STEP_Pin, GPIO_PIN_RESET);
        Motor_STEP_flag_Y = 0;
        if(Y_Motor_direction == 0) Y_tim2_tick_count++;
        else if (Y_tim2_tick_count > 0U) Y_tim2_tick_count--;
      }
      HAL_TIM_Base_Stop_IT(&htim3);
    }
  }
  else if(MODS == 4)//自动下棋模式
  {
     if (htim->Instance == TIM2)
    {
      if(X_target_tick_count == X_tim2_tick_count) Motor_STEP_flag_X = 0;
      else Motor_STEP_flag_X = 1;

      if(Y_target_tick_count == Y_tim2_tick_count) Motor_STEP_flag_Y = 0;
      else Motor_STEP_flag_Y = 1;

      if(Motor_STEP_flag_X == 1 || Motor_STEP_flag_Y == 1)
      {
        Motor_Direction();
        uint16_t pin_set = 0;
        if(Motor_STEP_flag_X == 1) pin_set |= MX_STEP_Pin;
        if(Motor_STEP_flag_Y == 1) pin_set |= MY_STEP_Pin;
        HAL_GPIO_WritePin(GPIOA, pin_set, GPIO_PIN_SET);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
      }
    }
    else if (htim->Instance == TIM3)
    {
      uint16_t pin_reset = 0;
      if(Motor_STEP_flag_X == 1) 
      {
        pin_reset |= MX_STEP_Pin;
        Motor_STEP_flag_X = 0;
        if(X_Motor_direction == 0) X_tim2_tick_count++;
        else if (X_tim2_tick_count > 0U) X_tim2_tick_count--;
      }
      if(Motor_STEP_flag_Y == 1)
      {
        pin_reset |= MY_STEP_Pin;
        Motor_STEP_flag_Y = 0;
        if(Y_Motor_direction == 0) Y_tim2_tick_count++;
        else if (Y_tim2_tick_count > 0U) Y_tim2_tick_count--;
      }
      if(pin_reset != 0) {
        HAL_GPIO_WritePin(GPIOA, pin_reset, GPIO_PIN_RESET);
      }
      HAL_TIM_Base_Stop_IT(&htim3);
    }
  }
}

// X轴, Y轴电机测试循环
void X_Motor_test_while(void)
{
  if(X_limit_right__flag)
  {
    X_Motor_direction = 1;  // 反转
    X_limit_right__flag = 0;  // 清除标志位
  }
  else if(X_limit_left__flag)
  {
    X_Motor_direction = 0;  // 正转
    X_limit_left__flag = 0;  // 清除标志位
  }
  if(Y_limit_right__flag)
  {
    Y_Motor_direction = 1;  // 反转
    Y_limit_right__flag = 0;  // 清除标志位
  }
  else if(Y_limit_left__flag)
  {
    Y_Motor_direction = 0;  // 正转
    Y_limit_left__flag = 0;  // 清除标志位
  }
  

}

// 舵机控制函数(编码器 -> TIM4_CH3 PWM占空比)
void Servo_Control(void)
{
  // 1) 读取 TIM1 编码器计数器当前值
  // 2) 计算相对上次采样的增量 delta，按增量调节舵机脉宽
  // 3) 将脉宽限制在 SERVO_PULSE_MIN_US ~ SERVO_PULSE_MAX_US，避免越界卡死
  // 4) 通过 TIM4_CH3 的 CCR 写入新的脉宽(单位: us，基于1MHz计数时钟)
  // 说明:
  // - 首次调用仅完成初始化并输出 SERVO_PULSE_INIT_US，不做增量运算
  // - 当前比例为每个编码器计数步进 5us，可按手感调整灵敏度
  static uint8_t inited = 0;  // 首次调用初始化标志
  static int16_t last_raw = 0;  // 上次编码器原始计数值
  static int32_t servo_pulse_us = SERVO_PULSE_INIT_US;  // 当前舵机目标脉宽(us)
  int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);  // 当前编码器原始计数值

  if (!inited)
  {
    inited = 1;
    last_raw = raw;
    // 首次进入时将舵机打到初始中位
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)servo_pulse_us);
    Servo_angle_deg = (int16_t)(((servo_pulse_us - (int32_t)SERVO_PULSE_MIN_US) * 180) /
                                ((int32_t)SERVO_PULSE_MAX_US - (int32_t)SERVO_PULSE_MIN_US));
    return;
  }

  int16_t delta = (int16_t)(raw - last_raw);  // 本次相对上次的编码器增量
  if (delta != 0)
  {
    // 每个编码器计数步进 5us，平滑控制舵机角度
    servo_pulse_us += (int32_t)delta * 5;
    if (servo_pulse_us < (int32_t)SERVO_PULSE_MIN_US) servo_pulse_us = SERVO_PULSE_MIN_US;
    if (servo_pulse_us > (int32_t)SERVO_PULSE_MAX_US) servo_pulse_us = SERVO_PULSE_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)servo_pulse_us);
    Servo_angle_deg = (int16_t)(((servo_pulse_us - (int32_t)SERVO_PULSE_MIN_US) * 180) /
                                ((int32_t)SERVO_PULSE_MAX_US - (int32_t)SERVO_PULSE_MIN_US));
  }
  last_raw = raw;
}

// 按键控制
void Key_while(void)
{
  if(HAL_GPIO_ReadPin(Encoder_KEY_GPIO_Port, Encoder_KEY_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(100);  // 简单消抖
    if(HAL_GPIO_ReadPin(Encoder_KEY_GPIO_Port, Encoder_KEY_Pin) == GPIO_PIN_RESET)
    {
      MODS_old = MODS;  // 记录旧值
      MODS++;  // 切换到调整模式
      if(MODS > MODS_MAX) MODS = 1;  // 模式循环
    }
  }
}

//电机步数控制
void Motor_MoveToStep(int motor_id,uint32_t step)
{
  if(motor_id == 1)
  {
    X_target_tick_count = step;
    if(step > X_tim2_tick_count) X_Motor_direction = 0;
    else X_Motor_direction = 1;
  }
  else if(motor_id == 2)
  {
    Y_target_tick_count = step;
    if(step > Y_tim2_tick_count) Y_Motor_direction = 0;
    else Y_Motor_direction = 1;
  }
}

// 模式1: 读取编码器增量并映射到X轴目标步数(左端0, 右端X_tim2_tick_count_max)
void Mode1_X_ByEncoder(void)
{
  static uint8_t inited = 0;
  static int16_t last_raw = 0;
  int16_t raw;
  int16_t delta;
  int32_t target;

  if (MODS != 1U)
  {
    inited = 0;
    return;
  }

  raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
  if (!inited)
  {
    inited = 1;
    last_raw = raw;
    X_target_tick_count = X_tim2_tick_count;
    return;
  }

  delta = (int16_t)(raw - last_raw);
  if (delta != 0)
  {
    // 灵敏度放大30倍，并且基于"当前位置"计算目标，避免在0点附近出现反向失效
    target = (int32_t)X_tim2_tick_count + (int32_t)delta * 30;
    if (target < 0) target = 0;
    if (target > (int32_t)X_tim2_tick_count_max) target = X_tim2_tick_count_max;
    X_target_tick_count = (uint32_t)target;
    Motor_MoveToStep(1, X_target_tick_count);
  }
  last_raw = raw;
}

// 模式2: 读取编码器增量并映射到Y轴目标步数(左端0, 右端Y_tim2_tick_count_max)
void Mode2_Y_ByEncoder(void)
{
  static uint8_t inited = 0;
  static int16_t last_raw = 0;
  int16_t raw;
  int16_t delta;
  int32_t target;

  if (MODS != 2U)
  {
    inited = 0;
    return;
  }

  raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
  if (!inited)
  {
    inited = 1;
    last_raw = raw;
    Y_target_tick_count = Y_tim2_tick_count;
    return;
  }

  delta = (int16_t)(raw - last_raw);
  if (delta != 0)
  {
    // 与模式1一致：灵敏度放大30倍，并基于当前位置计算目标
    target = (int32_t)Y_tim2_tick_count + (int32_t)delta * 30;
    if (target < 0) target = 0;
    if (target > (int32_t)Y_tim2_tick_count_max) target = Y_tim2_tick_count_max;
    Y_target_tick_count = (uint32_t)target;
    Motor_MoveToStep(2, Y_target_tick_count);
  }
  last_raw = raw;
}

// 模式3: 编码器直接控制舵机，两个步进电机不运动，也不自动切换模式
void Mode3_ServoToggleAndBack(void)
{
  static uint8_t mode3_inited = 0;

  if (MODS != 3U)
  {
    mode3_inited = 0;
    return;
  }

  if (!mode3_inited)
  {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    mode3_inited = 1;
  }

  Servo_Control();
}

// K230坐标转步进坐标:
// - K230坐标(135,63)对应中间棋盘中心格，即 board_x_step[2], board_y_step[2]
// - K230坐标系假定: 向上为Y正方向，向左为X正方向
// - 我方步进坐标系: 向上为X正方向，向右为Y正方向
void CameraCoord_ToStep(int16_t cam_x, int16_t cam_y, int32_t *x_step, int32_t *y_step)
{
  int32_t delta_cam_x = (int32_t)cam_x - K230_CENTER_X;
  int32_t delta_cam_y = (int32_t)cam_y - K230_CENTER_Y;

  if (x_step != NULL)
  {
    *x_step = board_x_step[2] + (delta_cam_y * STEP_GRID_DELTA) / K230_GRID_DELTA;
  }

  if (y_step != NULL)
  {
    *y_step = board_y_step[2] - (delta_cam_x * STEP_GRID_DELTA) / K230_GRID_DELTA;
  }
}

// 模式4: 自动下棋模式
void Mode4_AutoChess(void)
{
  static uint8_t state = 0;
  static uint32_t wait_ms = 0;
  static int32_t place_target_x = 0;
  static int32_t place_target_y = 0;
  uint32_t pulse_us;

  if(MODS != 4U) {
    state = 0;
    return;
  }

  switch(state) {
    case 0: // Wait for valid task
      if(uart_data_ready) {
        uart_data_ready = 0;
        int32_t target_x = 0, target_y = 0;
        uint8_t command_valid = 0;
        place_target_x = 0;
        place_target_y = 0;

        // 题目1/2/4/5: 从1~19的棋格拿起，再放到1~19的棋格
        if((uart_task == 1U || uart_task == 2U || uart_task == 4U || uart_task == 5U) &&
           uart_coord1 >= 1U && uart_coord1 <= 19U) {
          BoardMap_GetCellStep(uart_coord1 - 1, &target_x, &target_y);
          if(uart_coord2 >= 1U && uart_coord2 <= 19U) {
            BoardMap_GetCellStep(uart_coord2 - 1, &place_target_x, &place_target_y);
            command_valid = 1;
          }
        }
        // 题目3: 从待命区1~10拿子，放到K230给出的坐标
        else if(uart_task == 3U && uart_coord1 >= 1U && uart_coord1 <= 10U) {
          BoardMap_GetCellStep(uart_coord1 - 1, &target_x, &target_y);
          CameraCoord_ToStep(uart_cam_x, uart_cam_y, &place_target_x, &place_target_y);
          if(place_target_x >= TASK3_X_MIN_STEP && place_target_x <= TASK3_X_MAX_STEP &&
             place_target_y >= TASK3_Y_MIN_STEP && place_target_y <= TASK3_Y_MAX_STEP) {
            command_valid = 1;
          }
        }
        if(command_valid) {
          Motor_MoveToStep(1, (uint32_t)target_x);
          Motor_MoveToStep(2, (uint32_t)target_y);
          state = 1;
        }
      }
      break;

    case 1: // Wait for motors to reach coord1
      if(X_tim2_tick_count == X_target_tick_count && Y_tim2_tick_count == Y_target_tick_count) {
        // 到达取子位置: 舵机下压到取放棋角度
        pulse_us = (uint32_t)SERVO_PULSE_MIN_US +
                   (SERVO_PICK_PLACE_ANGLE_DEG * ((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US)) / 180U;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
        Servo_angle_deg = SERVO_PICK_PLACE_ANGLE_DEG;
        wait_ms = HAL_GetTick();
        state = 2;
      }
      break;

    case 2: // Wait for servo down at coord1
      if((HAL_GetTick() - wait_ms) >= 1200U) {
        // 取子: 电磁铁吸附，然后舵机抬回90度
        HAL_GPIO_WritePin(YA_S_GPIO_Port, YA_S_Pin, GPIO_PIN_SET);
        pulse_us = (uint32_t)SERVO_PULSE_MIN_US +
                   (90U * ((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US)) / 180U;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
        Servo_angle_deg = 90;
        wait_ms = HAL_GetTick();
        state = 3;
      }
      break;

    case 3: // Wait for servo up after pickup, then move to coord2
      if((HAL_GetTick() - wait_ms) >= 1200U) {
        Motor_MoveToStep(1, (uint32_t)place_target_x);
        Motor_MoveToStep(2, (uint32_t)place_target_y);
        state = 4;
      }
      break;

    case 4: // Wait for motors to reach coord2
      if(X_tim2_tick_count == X_target_tick_count && Y_tim2_tick_count == Y_target_tick_count) {
        // 到达放子位置: 舵机下压到取放棋角度
        pulse_us = (uint32_t)SERVO_PULSE_MIN_US +
                   (SERVO_PICK_PLACE_ANGLE_DEG * ((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US)) / 180U;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
        Servo_angle_deg = SERVO_PICK_PLACE_ANGLE_DEG;
        wait_ms = HAL_GetTick();
        state = 5;
      }
      break;

    case 5: // Wait for servo down at coord2
      if((HAL_GetTick() - wait_ms) >= 1200U) {
        // 放子: 电磁铁断电，然后舵机抬回90度
        HAL_GPIO_WritePin(YA_S_GPIO_Port, YA_S_Pin, GPIO_PIN_RESET);
        pulse_us = (uint32_t)SERVO_PULSE_MIN_US +
                   (90U * ((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US)) / 180U;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
        Servo_angle_deg = 90;
        wait_ms = HAL_GetTick();
        state = 6;
      }
      break;

    case 6: // Wait for servo up, then move motors away from camera
      if((HAL_GetTick() - wait_ms) >= 1200U) {
        if(uart_task == 1U || uart_task == 2U || uart_task == 3U || uart_task == 4U || uart_task == 5U) {
          Motor_MoveToStep(1, 300U);
          Motor_MoveToStep(2, 300U);
          state = 7;
        }
        else {
          state = 0;
          MODS = 1U; // Return to mode 1
        }
      }
      break;

    case 7: // Wait for motors to move to camera-clear position
      if(X_tim2_tick_count == X_target_tick_count && Y_tim2_tick_count == Y_target_tick_count) {
        state = 0;
        MODS = 1U; // Return to mode 1
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)"OK", sizeof("OK"));
      }
      break;
  }
}

// 串口接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    if(rx_buf[0] == '(') {
      uart_rx_idx = 0;
      uart_rx_buffer[uart_rx_idx++] = rx_buf[0];
    }
    else if(uart_rx_idx > 0 && uart_rx_idx < sizeof(uart_rx_buffer) - 1) {
      uart_rx_buffer[uart_rx_idx++] = rx_buf[0];
      if(rx_buf[0] == ')') {
        uart_rx_buffer[uart_rx_idx] = '\0';
        int t, c1, c2, c3;
        if(sscanf(uart_rx_buffer, "(%d,%d,%d,%d)", &t, &c1, &c2, &c3) == 4) {
          uart_task = (uint8_t)t;
          uart_coord1 = (uint8_t)c1;
          uart_cam_x = (int16_t)c2;
          uart_cam_y = (int16_t)c3;
          uart_data_ready = 1;
          if(t == 3) {
            MODS = 4U; // 收到题目3指令后切入模式4
          }
        }
        else if(sscanf(uart_rx_buffer, "(%d,%d,%d)", &t, &c1, &c2) == 3) {
          uart_task = (uint8_t)t;
          uart_coord1 = (uint8_t)c1;
          uart_coord2 = (uint8_t)c2;
          uart_data_ready = 1;
          if(t == 1 || t == 2 || t == 4 || t == 5) {
            MODS = 4U; // 收到指令后切入模式4
          }
        }
        uart_rx_idx = 0;
      }
    }
    HAL_UART_Receive_IT(&huart2, rx_buf, 1);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* 覆写TIM2/TIM3/TIM4参数，避免CubeMX重生成后时序丢失 */
  __HAL_TIM_SET_PRESCALER(&htim2, 71);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 599);
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  __HAL_TIM_SET_PRESCALER(&htim3, 71);
  __HAL_TIM_SET_AUTORELOAD(&htim3, 19);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  /* TIM4 设为 50Hz (1MHz计数时钟下 ARR=20000-1) */
  __HAL_TIM_SET_PRESCALER(&htim4, 71);
  __HAL_TIM_SET_AUTORELOAD(&htim4, 19999);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);  // 启动编码器
  HAL_TIM_Base_Start_IT(&htim2);  // TIM2: 步进调度时基中断
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  // 启动PWM输出
  HAL_UART_Receive_IT(&huart2, rx_buf, 1);  // 启动UART接收中断

  Motor_init();  // 电机初始化
  Motor_Direction(); // 电机方向初始化
  OLED_Init();  // OLED初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    Mode1_X_ByEncoder();  // 模式1: 编码器控制X轴步进
    Mode2_Y_ByEncoder();  // 模式2: 编码器控制Y轴步进
    Mode3_ServoToggleAndBack();  // 模式3: 编码器直接控制舵机
    Mode4_AutoChess();  // 模式4: 串口接收任务自动下棋
    oled_while();  // OLED显示
    Key_while();   // 按键控制
    //Motor_Direction();// 根据全局变量切换电机方向
    //X_Motor_test_while();// X轴, Y轴电机测试循环
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
