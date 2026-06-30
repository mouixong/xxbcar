/*
* 本文件是 hoverboard-firmware-hack 项目的一部分。
*
* 版权所有 (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* 版权所有 (C) 2017-2018 Nico Stute <crinq@crinq.de>
* 版权所有 (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* 版权所有 (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* 本程序是自由软件：您可以根据自由软件基金会发布的
* GNU 通用公共许可证（第3版或任何后续版本）的条款，
* 重新分发和/或修改本程序。
*
* 本程序的发布是希望它能有所帮助，
* 但没有任何担保；甚至没有对适销性或特定用途适用性的暗示担保。
* 详情请参见 GNU 通用公共许可证。
*
* 您应该已经收到一份 GNU 通用公共许可证的副本。
* 如果没有，请参见 <http://www.gnu.org/licenses/>。
*/








/*
*********************************************************************************************************
*
*	项目名称 : DIY卡丁车固件基于GIT开源代码适配大小象源码
*	版    本 : V1.0
*	说    明 : GITHUB开源搜:hoverboard-firmware-hack-foc-bbcar 
*              市面上平衡车主板改卡丁车，基本都是用此开源代码
*			   本着开源精神，适配过的源码无偿分享，摆脱高价固件
*              只为让更多人以更低价格享受卡丁车的快乐      基于开源精神分享
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2026-06-20 开源社区  正式发布
*
*********************************************************************************************************
*/







#include <stdio.h>
#include <stdlib.h> // 用于 abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"      /* BLDC 控制器头文件 */
#include "rtwtypes.h"
#include "comms.h"
#include "xxbcar_features.h"    // DIY卡丁车扩展功能模块头文件

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

//------------------------------------------------------------------------
// 外部设置的全局变量
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  extern LCD_PCF8574_HandleTypeDef lcd;
  extern uint8_t LCDerrorFlag;
#endif

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

volatile uint8_t uart_buf[200];

// Matlab 定义 - 来自自动代码生成
//---------------
extern P    rtP_Left;                   /* 模块参数（自动存储） */
extern P    rtP_Right;                  /* 模块参数（自动存储） */
extern ExtY rtY_Left;                   /* 外部输出 */
extern ExtY rtY_Right;                  /* 外部输出 */
extern ExtU rtU_Left;                   /* 外部输入 */
extern ExtU rtU_Right;                  /* 外部输入 */
//---------------

extern uint8_t     inIdx;               // 双输入使用的输入索引
extern uint8_t     inIdx_prev;
extern InputStruct input1[];            // 输入结构体
extern InputStruct input2[];            // 输入结构体

extern int16_t speedAvg;                // 平均测量速度
extern int16_t speedAvgAbs;             // 平均测量速度的绝对值
extern volatile uint32_t timeoutCntGen; // 通用超时计数器（PPM、PWM、Nunchuk）
extern volatile uint8_t  timeoutFlgGen; // 通用超时标志（PPM、PWM、Nunchuk）
extern uint8_t timeoutFlgADC;           // ADC 保护超时标志：0 = 正常，1 = 检测到问题（线路断开或 ADC 数据错误）
extern uint8_t timeoutFlgSerial;        // 串口接收命令超时标志：0 = 正常，1 = 检测到问题（线路断开或接收数据错误）

extern volatile int pwml;               // 左轮 PWM 全局变量，范围 -1000 到 1000
extern volatile int pwmr;               // 右轮 PWM 全局变量，范围 -1000 到 1000

extern uint8_t enable;                  // 电机使能全局变量

extern int16_t batVoltage;              // 电池电压全局变量

#if defined(SIDEBOARD_SERIAL_USART2)
extern SerialSideboard Sideboard_L;
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
extern SerialSideboard Sideboard_R;
#endif
#if (defined(CONTROL_PPM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PPM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif
#if (defined(CONTROL_PWM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PWM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t pwm_captured_ch1_value;
extern volatile uint16_t pwm_captured_ch2_value;
#endif


//------------------------------------------------------------------------
// 在 main.c 中设置的全局变量
//------------------------------------------------------------------------
uint8_t backwardDrive;
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // 校准后的电池电压全局变量
int16_t board_temp_deg_c;        // 校准后的板载温度全局变量，单位：摄氏度
int16_t left_dc_curr;            // 左侧直流母线电流全局变量
int16_t right_dc_curr;           // 右侧直流母线电流全局变量
int16_t dc_curr;                 // 总直流母线电流全局变量
int16_t cmdL;                    // 左侧命令全局变量
int16_t cmdR;                    // 右侧命令全局变量 

//------------------------------------------------------------------------
// 局部变量
//------------------------------------------------------------------------
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

#ifdef VARIANT_TRANSPOTTER
  uint8_t  nunchuk_connected;
  extern float    setDistance;  

  static uint8_t  checkRemote = 0;
  static uint16_t distance;
  static float    steering;
  static int      distanceErr;  
  static int      lastDistance = 0;
  static uint16_t transpotter_counter = 0;
#endif

#ifdef VARIANT_BBCAR
  int16_t    speed;                // 速度局部变量，范围 -1000 到 1000
  int16_t speedL     = 0, speedR     = 0;
  int16_t lastSpeedL = 0, lastSpeedR = 0;
#else
  static int16_t    speed;                // 速度局部变量，范围 -1000 到 1000
#endif

#ifndef VARIANT_TRANSPOTTER
  static int16_t  steer;                // 转向局部变量，范围 -1000 到 1000
  static int16_t  steerRateFixdt;       // 转向速率限制器的局部定点变量
  static int16_t  speedRateFixdt;       // 速度速率限制器的局部定点变量
  static int32_t  steerFixdt;           // 转向低通滤波器的局部定点变量
  static int32_t  speedFixdt;           // 速度低通滤波器的局部定点变量
#endif

static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // 刹车踏板的多击检测功能定义

static uint16_t rate = RATE; // 可调速率，用于在启动时支持多种驾驶模式

#ifdef MULTI_MODE_DRIVE
  static uint8_t drive_mode;
  static uint16_t max_speed;
#endif


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* 系统中断初始化 */
  /* MemoryManagement_IRQn 中断配置 */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn 中断配置 */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn 中断配置 */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn 中断配置 */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn 中断配置 */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn 中断配置 */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn 中断配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();        

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   
  Input_Lim_Init();  
  Input_Init();       

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  #ifdef VARIANT_BBCAR
    poweroffPressCheck();  
    bbcarDetectDrivingMode();
  #endif

  // 初始化DIY卡丁车扩展功能模块（蓝牙通信、一线通仪表、大灯等）
  XXBCAR_Init();

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  #ifdef MULTI_MODE_DRIVE
    if (adc_buffer.l_tx2 > input1[0].min + 50 && adc_buffer.l_rx2 > input2[0].min + 50) {
      drive_mode = 2;
      max_speed = MULTI_MODE_DRIVE_M3_MAX;
      rate = MULTI_MODE_DRIVE_M3_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M3_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M3_I_MOT_MAX * A2BIT_CONV) << 4;
    } else if (adc_buffer.l_tx2 > input1[0].min + 50) {
      drive_mode = 1;
      max_speed = MULTI_MODE_DRIVE_M2_MAX;
      rate = MULTI_MODE_DRIVE_M2_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M2_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M2_I_MOT_MAX * A2BIT_CONV) << 4;
    } else {
      drive_mode = 0;
      max_speed = MULTI_MODE_DRIVE_M1_MAX;
      rate = MULTI_MODE_DRIVE_M1_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M1_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M1_I_MOT_MAX * A2BIT_CONV) << 4;
    }

    printf("Drive mode %i selected: max_speed:%i acc_rate:%i \r\n", drive_mode, max_speed, rate);
  #endif

  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  #ifdef MULTI_MODE_DRIVE
    // 等待触发器释放。如果超时则退出（以便在输入未校准时解除阻塞）
    int iTimeout = 0;
    while((adc_buffer.l_rx2 + adc_buffer.l_tx2) >= (input1[0].min + input2[0].min) && iTimeout++ < 300) {
      HAL_Delay(10);
    }
  #endif

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {   

    readCommand();                        
    calcAvgSpeed();                       

    // 调用DIY卡丁车扩展功能主循环（蓝牙通信、定速巡航、智能空挡、电子手刹等）
    XXBCAR_MainLoop();

    #ifndef VARIANT_TRANSPOTTER
      if (enable == 0 && !rtY_Left.z_errCode && !rtY_Right.z_errCode && 
          ABS(input1[inIdx].cmd) < 50 && ABS(input2[inIdx].cmd) < 50){
        beepShort(6);                     
        beepShort(4); HAL_Delay(100);
        steerFixdt = speedFixdt = 0;      
        enable = 1;                       
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }

      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;                                        
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50); 
      #endif

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold();                                           
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   
        if (speedAvgAbs < 60) {                                     
          multipleTapDet(input1[inIdx].cmd, HAL_GetTick(), &MultipleTapBrake); 
        }

        if (input1[inIdx].cmd > 30) {                              
          input2[inIdx].cmd = (int16_t)((input2[inIdx].cmd * speedBlend) >> 15);
          cruiseControl((uint8_t)rtP_Left.b_cruiseCtrlEna);        
        }
      }
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap); 
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   
        if (speedAvg > 0) {                                         
          input1[inIdx].cmd = (int16_t)((-input1[inIdx].cmd * speedBlend) >> 15);
        } else {
          input1[inIdx].cmd = (int16_t)(( input1[inIdx].cmd * speedBlend) >> 15);
        }
      }
      #endif

      #ifdef VARIANT_SKATEBOARD
        if (input2[inIdx].cmd < 0) {                                 
          if (speedAvg > 0) {                                       
            input2[inIdx].cmd  = (int16_t)(( input2[inIdx].cmd * speedBlend) >> 15);
          } else {
            input2[inIdx].cmd  = (int16_t)((-input2[inIdx].cmd * speedBlend) >> 15);
          }
        }
      #endif

      rateLimiter16(input1[inIdx].cmd, rate, &steerRateFixdt);
      rateLimiter16(input2[inIdx].cmd, rate, &speedRateFixdt);
      filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      steer = (int16_t)(steerFixdt >> 16);  
      speed = (int16_t)(speedFixdt >> 16);  

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {               /* 仅在使用ADC踏板输入时启用以下实现 */

        #ifdef MULTI_MODE_DRIVE
        if (speed >= max_speed) {
          speed = max_speed;
        }
        #endif

        if (!MultipleTapBrake.b_multipleTap) {  /* 检测行驶方向：前进 */
          speed = steer + speed;                /* 前进驾驶：此时steer代表刹车，speed代表油门 */
        } else {
          speed = steer - speed;                /* 后退驾驶：此时steer代表刹车，speed代表油门 */
        }
        steer = 0;                              /* 不应用转向值，避免STEER_COEFFICIENT非零时产生副作用 */
      }
      #endif

      #if defined(TANK_STEERING) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_SKATEBOARD) 
        /* 坦克转向模式：左右轮分别由steer和speed直接控制，不进行混合 */
        cmdL = steer; 
        cmdR = speed;
      #else 
        mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);   /* 速度和转向混合算法：根据speed和steer计算左右轮命令 */
      #endif
      
      #ifdef VARIANT_BBCAR
        cmdR = cmdL = bbcarLoop();

        // DIY卡丁车扩展功能：电子手刹开启时强制电机抱死（踩踏板临时解除）
        if (handBrakeEnabled) {
            extern InputStruct input1[];
            extern InputStruct input2[];
            uint8_t pedalPressed = (input1[inIdx].cmd > 50) || (input2[inIdx].cmd > 50);
            if (!pedalPressed) {
                cmdL = cmdR = 0;    // 手刹抱死：强制电机输出为0
            }
        }

        // DIY卡丁车扩展功能：智能空挡时解除电机驱动，车轮可自由推动
        if (neutralGearActive) {
            cmdL = cmdR = 0;        // 空挡：电机无输出
        }

        // DIY卡丁车扩展功能：遥控模式差速转向
        if (currentDriveMode == MODE_REMOTE) {
            cmdL += leftMotorOffset;
            cmdR += rightMotorOffset;
            cmdL = CLAMP(cmdL, -1000, 1000);
            cmdR = CLAMP(cmdR, -1000, 1000);
        }
      #endif


      #ifdef INVERT_R_DIRECTION
        pwmr = cmdR;
      #else
        pwmr = -cmdR;
      #endif
      #ifdef INVERT_L_DIRECTION
        pwml = -cmdL;
      #else
        pwml = cmdL;
      #endif
    #endif

    #ifdef VARIANT_TRANSPOTTER
      distance    = CLAMP(input1[inIdx].cmd - 180, 0, 4095);
      steering    = (input2[inIdx].cmd - 2048) / 2048.0;
      distanceErr = distance - (int)(setDistance * 1345);

      if (nunchuk_connected == 0) {
        cmdL = cmdL * 0.8f + (CLAMP(distanceErr + (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        cmdR = cmdR * 0.8f + (CLAMP(distanceErr - (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        if (distanceErr > 0) {
          enable = 1;
        }
        if (distanceErr > -300) {
          #ifdef INVERT_R_DIRECTION
            pwmr = cmdR;
          #else
            pwmr = -cmdR;
          #endif
          #ifdef INVERT_L_DIRECTION
            pwml = -cmdL;
          #else
            pwml = cmdL;
          #endif

          if (checkRemote) {
            if (!HAL_GPIO_ReadPin(LED_PORT, LED_PIN)) {
              //enable = 1;
            } else {
              enable = 0;
            }
          }
        } else {
          enable = 0;
        }
        timeoutCntGen = 0;
        timeoutFlgGen = 0;
      }

      if (timeoutFlgGen) {
        pwml = 0;
        pwmr = 0;
        enable = 0;
        #ifdef SUPPORT_LCD
          LCD_SetLocation(&lcd,  0, 0); LCD_WriteString(&lcd, "Len:");
          LCD_SetLocation(&lcd,  8, 0); LCD_WriteString(&lcd, "m(");
          LCD_SetLocation(&lcd, 14, 0); LCD_WriteString(&lcd, "m)");
        #endif
        HAL_Delay(1000);
        nunchuk_connected = 0;
      }

      if ((distance / 1345.0) - setDistance > 0.5 && (lastDistance / 1345.0) - setDistance > 0.5) { // 错误，机器人距离太远！
        enable = 0;
        beepLong(5);
        #ifdef SUPPORT_LCD
          LCD_ClearDisplay(&lcd);
          HAL_Delay(5);
          LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Emergency Off!");
          LCD_SetLocation(&lcd, 0, 1); LCD_WriteString(&lcd, "Keeper too fast.");
        #endif
        poweroff();
      }

      #ifdef SUPPORT_NUNCHUK
        if (transpotter_counter % 500 == 0) {
          if (nunchuk_connected == 0 && enable == 0) {
              if(Nunchuk_Read() == NUNCHUK_CONNECTED) {
                #ifdef SUPPORT_LCD
                  LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Nunchuk Control");
                #endif
                nunchuk_connected = 1;
	      }
	    } else {
              nunchuk_connected = 0;
	    }
          }
        }   
      #endif

      #ifdef SUPPORT_LCD
        if (transpotter_counter % 100 == 0) {
          if (LCDerrorFlag == 1 && enable == 0) {

          } else {
            if (nunchuk_connected == 0) {
              LCD_SetLocation(&lcd,  4, 0); LCD_WriteFloat(&lcd,distance/1345.0,2);
              LCD_SetLocation(&lcd, 10, 0); LCD_WriteFloat(&lcd,setDistance,2);
            }
            LCD_SetLocation(&lcd,  4, 1); LCD_WriteFloat(&lcd,batVoltage, 1);
            // LCD_SetLocation(&lcd, 11, 1); LCD_WriteFloat(&lcd,MAX(ABS(currentR), ABS(currentL)),2);
          }
        }
      #endif
      transpotter_counter++;
    #endif

    // ####### 副板处理 #######
    #if defined(SIDEBOARD_SERIAL_USART2)
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
    #endif
    

    // ####### 计算板载温度 #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // 将定点数转换为整数
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### 计算校准后的电池电压 #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### 计算直流母线电流 #######
    left_dc_curr  = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;   // 左侧直流母线电流 * 100
    right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;  // 右侧直流母线电流 * 100
    dc_curr       = left_dc_curr + right_dc_curr;            // 总直流母线电流 * 100

    // ####### 调试串口输出 #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (main_loop_counter % 50 == 0) {    // 每 125 毫秒定期发送数据      
        #if defined(DEBUG_SERIAL_PROTOCOL)
          process_debug();
        #else
          // printf("in1:%i in2:%i cmdL:%i cmdR:%i BatADC:%i BatV:%i TempADC:%i Temp:%i velL:%i velR:%i curL:%i curR:%i\r\n",
          printf("in1:%i in2:%i cmdL:%i cmdR:%i BatADC:%i BatV:%i TempADC:%i Temp:%i velL:%i velR:%i\r\n",
            input1[inIdx].raw,        // 1: 输入1
            input2[inIdx].raw,        // 2: 输入2
            cmdL,                     // 3: 输出命令：[-1000, 1000]
            cmdR,                     // 4: 输出命令：[-1000, 1000]
            adc_buffer.batt1,         // 5: 用于电池电压校准
            batVoltageCalib,          // 6: 用于验证电池电压校准
            board_temp_adcFilt,       // 7: 用于板载温度校准
            board_temp_deg_c,         // 8: 用于验证板载温度校准
            rtY_Left.n_mot,           // 9: 电机转速
            rtY_Right.n_mot           //10: 电机转速
            // rtY_Left.iq,              //11: 电机 q 轴电流
            // rtY_Right.iq              //12: 电机 q 轴电流
          );
        #endif
      }
    #endif

    // ####### 反馈串口输出 #######
    #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      if (main_loop_counter % 2 == 0) {    // 每 10 毫秒定期发送数据
        Feedback.start	        = (uint16_t)SERIAL_START_FRAME;
        Feedback.cmd1           = (int16_t)input1[inIdx].cmd;
        Feedback.cmd2           = (int16_t)input2[inIdx].cmd;
        Feedback.speedR_meas	  = (int16_t)rtY_Right.n_mot;
        Feedback.speedL_meas	  = (int16_t)rtY_Left.n_mot;
        Feedback.batVoltage	    = (int16_t)batVoltageCalib;
        Feedback.boardTemp	    = (int16_t)board_temp_deg_c;

        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_L;
            Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_R;
            Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
      }
    #endif

    poweroffPressCheck();

    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20){  
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, temperature is too high\r\n");
      #endif
      poweroff();
    } else if ( BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20){
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, battery voltage is too low\r\n");
      #endif
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {                                           
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                                                       
       beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                                                                    
      beepCount(3, 24, 1);
    } else if (timeoutFlgGen) {                                                                       
      beepCount(4, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && (((cmdR < -50 || cmdL < -50) && speedAvg < 0) || MultipleTapBrake.b_multipleTap)) { 
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {  
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }


    inactivity_timeout_counter++;

    // ####### 不活动超时 #######
    if (abs(cmdL) > 50 || abs(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    }

    #if defined(CRUISE_CONTROL_SUPPORT) || defined(STANDSTILL_HOLD_ENABLE)
      if ((abs(rtP_Left.n_cruiseMotTgt)  > 50 && rtP_Left.b_cruiseCtrlEna) || 
          (abs(rtP_Right.n_cruiseMotTgt) > 50 && rtP_Right.b_cruiseCtrlEna)) {
        inactivity_timeout_counter = 0;
      }
    #endif

    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // 主循环其余部分大约需要 1 毫秒
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, wheels were inactive for too long\r\n");
      #endif
      poweroff();
    }


    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // 用于通过连接示波器到 LED_PIN 测量 main() 循环持续时间
    // 更新状态
    inIdx_prev = inIdx;
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
    }
  }
}


// ===========================================================
/** 系统时钟配置
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**初始化 CPU、AHB 和 APB 总线时钟
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**初始化 CPU、AHB 和 APB 总线时钟
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 兆赫兹
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 兆赫兹
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**配置 Systick 中断时间
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**配置 Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn 中断配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
