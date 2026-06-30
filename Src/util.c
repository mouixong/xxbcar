/**
  * 本文件是 hoverboard-firmware-hack 项目的一部分。
  *
  * 版权所有 (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * 本程序是自由软件：您可以根据自由软件基金会发布的 GNU 通用公共许可证的条款
  * 重新分发和/或修改它，无论是许可证的第 3 版，还是（由您选择）任何更高版本。
  *
  * 分发本程序是希望它有用，但没有任何担保；甚至没有对适销性或特定用途适用性的默示担保。
  * 更多细节请参阅 GNU 通用公共许可证。
  *
  * 您应该已经收到一份 GNU 通用公共许可证的副本。
  * 如果没有，请参见 <http://www.gnu.org/licenses/>。
*/

// 头文件包含
#include <stdio.h>
#include <stdlib.h> // 用于 abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

/* =========================== 变量定义 =========================== */

//------------------------------------------------------------------------
// 外部设置的全局变量
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerCount;             // 蜂鸣器计数全局变量，可为 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // 蜂鸣器音调全局变量，可为 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // 蜂鸣器模式全局变量，可为 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // 电机使能全局变量

extern uint8_t nunchuk_data[6];
extern volatile uint32_t timeoutCntGen; // 通用超时计数器
extern volatile uint8_t  timeoutFlgGen; // 通用超时标志
extern volatile uint32_t main_loop_counter;

#if defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

#if defined(CONTROL_PWM_LEFT) || defined(CONTROL_PWM_RIGHT)
extern volatile uint16_t pwm_captured_ch1_value;
extern volatile uint16_t pwm_captured_ch2_value;
#endif

#ifdef VARIANT_BBCAR
  int8_t drive_mode = 0;
  int32_t adc_error_break_and_poweroff = 0;
#endif


//------------------------------------------------------------------------
// 在 util.c 中设置的全局变量
//------------------------------------------------------------------------
// Matlab 定义 - 来自自动生成代码
//---------------
RT_MODEL rtM_Left_;                     /* 实时模型 */
RT_MODEL rtM_Right_;                    /* 实时模型 */
RT_MODEL *const rtM_Left  = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

extern P rtP_Left;                      /* 模块参数（自动存储） */
DW       rtDW_Left;                     /* 可观测状态 */
ExtU     rtU_Left;                      /* 外部输入 */
ExtY     rtY_Left;                      /* 外部输出 */

P        rtP_Right;                     /* 模块参数（自动存储） */
DW       rtDW_Right;                    /* 可观测状态 */
ExtU     rtU_Right;                     /* 外部输入 */
ExtY     rtY_Right;                     /* 外部输出 */
//---------------

uint8_t  inIdx      = 0;
uint8_t  inIdx_prev = 0;
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
InputStruct input1[INPUTS_NR] = { {0, 0, 0, PRI_INPUT1}, {0, 0, 0, AUX_INPUT1} };
InputStruct input2[INPUTS_NR] = { {0, 0, 0, PRI_INPUT2}, {0, 0, 0, AUX_INPUT2} };
#else
InputStruct input1[INPUTS_NR] = { {0, 0, 0, PRI_INPUT1} };
InputStruct input2[INPUTS_NR] = { {0, 0, 0, PRI_INPUT2} };
#endif

int16_t  speedAvg;                      // 平均测量速度
int16_t  speedAvgAbs;                   // 平均测量速度绝对值
uint8_t  timeoutFlgADC    = 0;          // ADC 保护超时标志：0 = 正常，1 = 检测到问题（线路断开或 ADC 数据错误）
uint8_t  timeoutFlgSerial = 0;          // 串口接收命令超时标志：0 = 正常，1 = 检测到问题（线路断开或接收数据错误）

uint8_t  ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t  ctrlModReq    = CTRL_MOD_REQ;  // 最终控制模式请求 

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
LCD_PCF8574_HandleTypeDef lcd;
#endif

#ifdef VARIANT_TRANSPOTTER
float    setDistance;
uint16_t VirtAddVarTab[NB_OF_VAR] = {1337};       // 用户定义的虚拟地址：禁止使用 0xFFFF 值
static   uint16_t saveValue       = 0;
static   uint8_t  saveValue_valid = 0;
#elif !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
uint16_t VirtAddVarTab[NB_OF_VAR] = {1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009,
                                     1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018};
#else
uint16_t VirtAddVarTab[NB_OF_VAR] = {1000};       // 虚拟地址占位，用于避免编译警告
#endif


//------------------------------------------------------------------------
// 局部变量
//------------------------------------------------------------------------
static int16_t INPUT_MAX;             // [-] 输入目标最大值限制
static int16_t INPUT_MIN;             // [-] 输入目标最小值限制


#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
  static uint8_t  cur_spd_valid  = 0;
  static uint8_t  inp_cal_valid  = 0;
#endif

#if defined(CONTROL_ADC)
static uint16_t timeoutCntADC = ADC_PROTECT_TIMEOUT;  // ADC 保护超时计数器
#endif

#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
static uint8_t  rx_buffer_L[SERIAL_BUFFER_SIZE];      // USART 接收 DMA 循环缓冲区
static uint32_t rx_buffer_L_len = ARRAY_LEN(rx_buffer_L);
#endif
#if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
static uint16_t timeoutCntSerial_L = SERIAL_TIMEOUT;  // 串口接收命令超时计数器
static uint8_t  timeoutFlgSerial_L = 0;               // 串口接收命令超时标志：0 = 正常，1 = 检测到问题（线路断开或接收数据错误）
#endif
#if defined(SIDEBOARD_SERIAL_USART2)
SerialSideboard Sideboard_L;
SerialSideboard Sideboard_L_raw;
static uint32_t Sideboard_L_len = sizeof(Sideboard_L);
#endif

#if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
static uint8_t  rx_buffer_R[SERIAL_BUFFER_SIZE];      // USART 接收 DMA 循环缓冲区
static uint32_t rx_buffer_R_len = ARRAY_LEN(rx_buffer_R);
#endif
#if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
static uint16_t timeoutCntSerial_R = SERIAL_TIMEOUT;  // 串口接收命令超时计数器
static uint8_t  timeoutFlgSerial_R = 0;               // 串口接收命令超时标志：0 = 正常，1 = 检测到问题（线路断开或接收数据错误）
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
SerialSideboard Sideboard_R;
SerialSideboard Sideboard_R_raw;
static uint32_t Sideboard_R_len = sizeof(Sideboard_R);
#endif

#if defined(CONTROL_SERIAL_USART2)
static SerialCommand commandL;
static SerialCommand commandL_raw;
static uint32_t commandL_len = sizeof(commandL);
  #ifdef CONTROL_IBUS
  static uint16_t ibusL_captured_value[IBUS_NUM_CHANNELS];
  #endif
#endif

#if defined(CONTROL_SERIAL_USART3)
static SerialCommand commandR;
static SerialCommand commandR_raw;
static uint32_t commandR_len = sizeof(commandR);
  #ifdef CONTROL_IBUS
  static uint16_t ibusR_captured_value[IBUS_NUM_CHANNELS];
  #endif
#endif

#if defined(SUPPORT_BUTTONS) || defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT)
static uint8_t button1;                 // 蓝色按钮
static uint8_t button2;                 // 绿色按钮
#endif

#ifdef VARIANT_HOVERCAR
static uint8_t brakePressed;
#endif

#if defined(CRUISE_CONTROL_SUPPORT) || (defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE))
static uint8_t cruiseCtrlAcv = 0;
static uint8_t standstillAcv = 0;
#endif

/* =========================== printf 重定向 =========================== */
/* 将 C 库 printf 函数重定向到 USART */
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  #ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #endif
  PUTCHAR_PROTOTYPE {
    #if defined(DEBUG_SERIAL_USART2)
      HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    #elif defined(DEBUG_SERIAL_USART3)
      HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);
    #endif
    return ch;
  }
  
  #ifdef __GNUC__
    int _write(int file, char *data, int len) {
      int i;
      for (i = 0; i < len; i++) { __io_putchar( *data++ );}
      return len;
    }
  #endif
#endif

 
/* =========================== 初始化函数 =========================== */

void BLDC_Init(void) {
  /* 设置 BLDC 控制器参数 */ 
  rtP_Left.b_angleMeasEna       = 0;            // 电机角度输入：0 = 估算角度，1 = 测量角度（例如有编码器时）
  rtP_Left.z_selPhaCurMeasABC   = 0;            // 左侧电机测量电流相位 {Green, Blue} = {iA, iB} -> 请勿更改
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA;
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)

  rtP_Right                     = rtP_Left;     // 将左侧电机参数复制到右侧电机参数
  rtP_Right.z_selPhaCurMeasABC  = 1;            // 右侧电机测量电流相位 {Blue, Yellow} = {iB, iC} -> 请勿更改

  /* 将左侧电机数据打包到 RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* 将右侧电机数据打包到 RTM */
  rtM_Right->defaultParam       = &rtP_Right;
  rtM_Right->dwork              = &rtDW_Right;
  rtM_Right->inputs             = &rtU_Right;
  rtM_Right->outputs            = &rtY_Right;

  /* 初始化 BLDC 控制器 */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
}

void Input_Lim_Init(void) {     // 输入限制 - 请勿修改！
  if (rtP_Left.b_fieldWeakEna || rtP_Right.b_fieldWeakEna) {
    INPUT_MAX = MAX( 1000, FIELD_WEAK_HI);
    INPUT_MIN = MIN(-1000,-FIELD_WEAK_HI);
  } else {
    INPUT_MAX =  1000;
    INPUT_MIN = -1000;
  }
}

void Input_Init(void) {
  #if defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)
    PPM_Init();
  #endif

 #if defined(CONTROL_PWM_LEFT) || defined(CONTROL_PWM_RIGHT)
    PWM_Init();
  #endif

  #if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
    UART2_Init();
  #endif
  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    UART3_Init();
  #endif
  #if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer_L, sizeof(rx_buffer_L));
    UART_DisableRxErrors(&huart2);
  #endif
  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_buffer_R, sizeof(rx_buffer_R));
    UART_DisableRxErrors(&huart3);
  #endif

  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    uint16_t writeCheck, readVal;
    HAL_FLASH_Unlock();
    EE_Init();            /* EEPROM 初始化 */
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    if (writeCheck == FLASH_WRITE_KEY) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Using the configuration from EEprom\r\n");
      #endif

      EE_ReadVariable(VirtAddVarTab[1] , &readVal); rtP_Left.i_max = rtP_Right.i_max = (int16_t)readVal;
      EE_ReadVariable(VirtAddVarTab[2] , &readVal); rtP_Left.n_max = rtP_Right.n_max = (int16_t)readVal;
      for (uint8_t i=0; i<INPUTS_NR; i++) {
        EE_ReadVariable(VirtAddVarTab[ 3+8*i] , &readVal); input1[i].typ = (uint8_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 4+8*i] , &readVal); input1[i].min = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 5+8*i] , &readVal); input1[i].mid = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 6+8*i] , &readVal); input1[i].max = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 7+8*i] , &readVal); input2[i].typ = (uint8_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 8+8*i] , &readVal); input2[i].min = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 9+8*i] , &readVal); input2[i].mid = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[10+8*i] , &readVal); input2[i].max = (int16_t)readVal;
      
        printf("Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\nLimits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
          input1[i].typ, input1[i].min, input1[i].mid, input1[i].max,
          input2[i].typ, input2[i].min, input2[i].mid, input2[i].max);
      }
    } else {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Using the configuration from config.h\r\n");
      #endif

      for (uint8_t i=0; i<INPUTS_NR; i++) {
        if (input1[i].typDef == 3) {  // 如果输入类型定义为 3（自动），根据 config.h 中的值识别输入类型
          input1[i].typ = checkInputType(input1[i].min, input1[i].mid, input1[i].max);
        } else {
          input1[i].typ = input1[i].typDef;
        }
        if (input2[i].typDef == 3) {
          input2[i].typ = checkInputType(input2[i].min, input2[i].mid, input2[i].max);
        } else {
          input2[i].typ = input2[i].typDef;
        }
        printf("Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\nLimits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
          input1[i].typ, input1[i].min, input1[i].mid, input1[i].max,
          input2[i].typ, input2[i].min, input2[i].mid, input2[i].max);
      }
    }
    HAL_FLASH_Lock();
  #endif

  #ifdef VARIANT_TRANSPOTTER
    enable = 1;

    HAL_FLASH_Unlock();
    EE_Init();            /* EEPROM 初始化 */
    EE_ReadVariable(VirtAddVarTab[0], &saveValue);
    HAL_FLASH_Lock();

    setDistance = saveValue / 1000.0;
    if (setDistance < 0.2) {
      setDistance = 1.0;
    }
  #endif

  #if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
    I2C_Init();
    HAL_Delay(50);
    lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
    lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
    lcd.pcf8574.i2c             = hi2c2;
    lcd.NUMBER_OF_LINES         = NUMBER_OF_LINES_2;
    lcd.type                    = TYPE0;

    if(LCD_Init(&lcd)!=LCD_OK) {
        // 发生错误
        //TODO while(1);
    }

    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd, 0, 0);
    #ifdef VARIANT_TRANSPOTTER
      LCD_WriteString(&lcd, "TranspOtter V2.1");
    #else
      LCD_WriteString(&lcd, "Hover V2.0");
    #endif
    LCD_SetLocation(&lcd,  0, 1); LCD_WriteString(&lcd, "Initializing...");
  #endif

  #if defined(VARIANT_TRANSPOTTER) && defined(SUPPORT_LCD)
    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd,  0, 1); LCD_WriteString(&lcd, "Bat:");
    LCD_SetLocation(&lcd,  8, 1); LCD_WriteString(&lcd, "V");
    LCD_SetLocation(&lcd, 15, 1); LCD_WriteString(&lcd, "A");
    LCD_SetLocation(&lcd,  0, 0); LCD_WriteString(&lcd, "Len:");
    LCD_SetLocation(&lcd,  8, 0); LCD_WriteString(&lcd, "m(");
    LCD_SetLocation(&lcd, 14, 0); LCD_WriteString(&lcd, "m)");
  #endif
}

/**
  * @brief  禁用 UART 外设上的接收错误检测中断（因为我们不希望 DMA 被停止）
  *         错误数据将基于起始帧和校验和进行过滤。
  * @param  huart: UART 句柄。
  * @retval None
  */
#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
void UART_DisableRxErrors(UART_HandleTypeDef *huart)
{  
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);    /* 禁用 PE（奇偶校验错误）中断 */  
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);     /* 禁用 EIE（帧错误、噪声错误、溢出错误）中断 */
}
#endif


/* =========================== 通用函数 =========================== */

void poweronMelody(void) {
    buzzerCount = 0;  // 防止与蜂鸣计数器相互干扰
    for (int i = 8; i >= 0; i--) {
      buzzerFreq = (uint8_t)i;
      HAL_Delay(100);
    }
    buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
    buzzerCount   = cnt;
    buzzerFreq    = freq;
    buzzerPattern = pattern;
}

void beepLong(uint8_t freq) {
    buzzerCount = 0;  // 防止与蜂鸣计数器相互干扰
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

void beepShort(uint8_t freq) {
    buzzerCount = 0;  // 防止与蜂鸣计数器相互干扰
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

void beepShortMany(uint8_t cnt, int8_t dir) {
    if (dir >= 0) {   // 升调
      for(uint8_t i = 2*cnt; i >= 2; i=i-2) {
        beepShort(i + 3);
      }
    } else {          // 降调
      for(uint8_t i = 2; i <= 2*cnt; i=i+2) {
        beepShort(i + 3);
      }
    }
}

#ifdef VARIANT_BBCAR
  void beepShortMany2(uint8_t cnt) {
      for(uint8_t i = 0; i < cnt; i++) {
      beepShort(2);
      HAL_Delay(200);
      }
  }
#endif

void calcAvgSpeed(void) {
    // 计算平均测量速度。负号（-）是因为电机以相反方向旋转
    speedAvg = 0;
    #if defined(MOTOR_LEFT_ENA)
      #if defined(INVERT_L_DIRECTION)
        speedAvg -= rtY_Left.n_mot;
      #else
        speedAvg += rtY_Left.n_mot;
      #endif
    #endif
    #if defined(MOTOR_RIGHT_ENA)
      #if defined(INVERT_R_DIRECTION)
        speedAvg += rtY_Right.n_mot;
      #else
        speedAvg -= rtY_Right.n_mot;
      #endif

      // 仅当两个电机都使能时才取平均
      #if defined(MOTOR_LEFT_ENA)
        speedAvg /= 2;
      #endif  
    #endif

    // 处理 SPEED_COEFFICIENT 符号为负的情况（即最高有效位为 1 时）
    if (SPEED_COEFFICIENT & (1 << 16)) {
      speedAvg    = -speedAvg;
    } 
    speedAvgAbs   = abs(speedAvg);
}

 /*
 * ADC 限值自动校准
 * 该函数用于查找 ADC 输入的最小值、最大值和中位值
 * 步骤：
 * - 按住电源按钮超过 5 秒，并在蜂鸣声后松开
 * - 反复将电位器自由移动到最小和最大限值
 * - 将电位器释放到静止位置
 * - 按下电源按钮确认，或等待 20 秒超时
 * 数值将保存到 Flash 中。如果使用 platformio 刷写，数值会持久保存。要擦除它们，请执行整片擦除。
 */
void adcCalibLim(void) {
#ifdef AUTO_CALIBRATION_ENA
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // 如果电机正在旋转，请勿进入此模式
    return;
  }

#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  printf("# Input calibration started...\r\n");
  printf("# move the potentiometers freely to the min and max limits repeatedly\r\n");
  printf("# release potentiometers to the resting postion\r\n");
  printf("# press the power button to confirm or wait for the 20 sec timeout\r\n");
  #endif

  readInputRaw();
  // 初始化：MIN = 高值，MAX = 低值
  int32_t  input1_fixdt = input1[inIdx].raw << 16;
  int32_t  input2_fixdt = input2[inIdx].raw << 16;
  int16_t  INPUT1_MIN_temp = MAX_int16_T;
  int16_t  INPUT1_MID_temp = 0;
  int16_t  INPUT1_MAX_temp = MIN_int16_T;
  int16_t  INPUT2_MIN_temp = MAX_int16_T;
  int16_t  INPUT2_MID_temp = 0;
  int16_t  INPUT2_MAX_temp = MIN_int16_T;
  int16_t  input_margin    = 0;
  uint16_t input_cal_timeout = 0;
  
  #ifdef CONTROL_ADC
  if (inIdx == CONTROL_ADC) {
    input_margin = ADC_MARGIN;
  }
  #endif

  // 在电源按钮未按下时，从 ADC 提取 MIN、MAX 和 MID
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && input_cal_timeout++ < 4000) {   // 20 秒超时
    readInputRaw();
    filtLowPass32(input1[inIdx].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[inIdx].raw, FILTER, &input2_fixdt);
    
    INPUT1_MID_temp = (int16_t)(input1_fixdt >> 16);// CLAMP(input1_fixdt >> 16, INPUT1_MIN, INPUT1_MAX);   // 将定点数转换为整数
    INPUT2_MID_temp = (int16_t)(input2_fixdt >> 16);// CLAMP(input2_fixdt >> 16, INPUT2_MIN, INPUT2_MAX);
    INPUT1_MIN_temp = MIN(INPUT1_MIN_temp, INPUT1_MID_temp);
    INPUT1_MAX_temp = MAX(INPUT1_MAX_temp, INPUT1_MID_temp);
    INPUT2_MIN_temp = MIN(INPUT2_MIN_temp, INPUT2_MID_temp);
    INPUT2_MAX_temp = MAX(INPUT2_MAX_temp, INPUT2_MID_temp);
    HAL_Delay(5);
  }

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  printf("Input1 is ");
  #endif
  uint8_t input1TypTemp = checkInputType(INPUT1_MIN_temp, INPUT1_MID_temp, INPUT1_MAX_temp);
  if (input1TypTemp == input1[inIdx].typDef || input1[inIdx].typDef == 3) {  // 仅在类型正确或类型设置为 3（自动）时接受校准
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("..OK\r\n");
    #endif
  } else {
    input1TypTemp = 0; // 禁用输入
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("..NOK\r\n");
    #endif
  }

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  printf("Input2 is ");
  #endif
  uint8_t input2TypTemp = checkInputType(INPUT2_MIN_temp, INPUT2_MID_temp, INPUT2_MAX_temp);
  if (input2TypTemp == input2[inIdx].typDef || input2[inIdx].typDef == 3) {  // 仅在类型正确或类型设置为 3（自动）时接受校准
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("..OK\r\n");
    #endif
  } else {
    input2TypTemp = 0; // 禁用输入
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("..NOK\r\n");
    #endif
  }


  // 至少有一个输入未被忽略
  if (input1TypTemp != 0 || input2TypTemp != 0){
    input1[inIdx].typ = input1TypTemp;
    input1[inIdx].min = INPUT1_MIN_temp + input_margin;
    input1[inIdx].mid = INPUT1_MID_temp;
    input1[inIdx].max = INPUT1_MAX_temp - input_margin;

    input2[inIdx].typ = input2TypTemp;
    input2[inIdx].min = INPUT2_MIN_temp + input_margin;
    input2[inIdx].mid = INPUT2_MID_temp;
    input2[inIdx].max = INPUT2_MAX_temp - input_margin;

    inp_cal_valid = 1;    // 标记校准值，在关机时保存到 Flash
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\nLimits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
            input1[inIdx].typ, input1[inIdx].min, input1[inIdx].mid, input1[inIdx].max,
            input2[inIdx].typ, input2[inIdx].min, input2[inIdx].mid, input2[inIdx].max);
    #endif
  }else{
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("Both inputs cannot be ignored, calibration rejected.\r\n");
    #endif
  }

#endif
#endif  // AUTO_CALIBRATION_ENA
}
 /*
 * 更新最大电机电流限制（通过 ADC1）和最大速度限制（通过 ADC2）
 * 步骤：
 * - 按住电源按钮超过 5 秒，在蜂鸣声后立即再短按一次
 * - 移动并保持电位器到所需的电流和速度限值位置
 * - 按下电源按钮确认，或等待 10 秒超时
 */
void updateCurSpdLim(void) {
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // 如果电机正在旋转，请勿进入此模式
    return;
  }

#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  printf("# Torque and Speed limits update started...\r\n");
  printf("# move and hold the pots to a desired limit position for Current and Speed\r\n");
  printf("# press the power button to confirm or wait for the 10 sec timeout\r\n");
  #endif

  int32_t  input1_fixdt = input1[inIdx].raw << 16;
  int32_t  input2_fixdt = input2[inIdx].raw << 16;
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // 等待电源按钮按下
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 秒超时
    readInputRaw();
    filtLowPass32(input1[inIdx].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[inIdx].raw, FILTER, &input2_fixdt);
    HAL_Delay(5);
  }
  // 计算缩放因子
  cur_factor = CLAMP((input1_fixdt - (input1[inIdx].min << 16)) / (input1[inIdx].max - input1[inIdx].min), 6553, 65535);    // ADC1, 最小电流(10%) = 1.5 A 
  spd_factor = CLAMP((input2_fixdt - (input2[inIdx].min << 16)) / (input2[inIdx].max - input2[inIdx].min), 3276, 65535);    // ADC2, 最小速度(5%) = 50 rpm
      
  if (input1[inIdx].typ != 0){
    // 更新电流限制
    rtP_Left.i_max = rtP_Right.i_max  = (int16_t)((I_MOT_MAX * A2BIT_CONV * cur_factor) >> 12);    // fixdt(0,16,16) 转 fixdt(1,16,4)
    cur_spd_valid   = 1;  // 标记更新值，在关机时保存到 Flash
  }

  if (input2[inIdx].typ != 0){
    // 更新速度限制
    rtP_Left.n_max = rtP_Right.n_max  = (int16_t)((N_MOT_MAX * spd_factor) >> 12);                 // fixdt(0,16,16) 转 fixdt(1,16,4)
    cur_spd_valid  += 2;  // 标记更新值，在关机时保存到 Flash
  }

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  // cur_spd_valid: 0 = 未更改限制, 1 = 电流限制已更改, 2 = 速度限制已更改, 3 = 两项限制均已更改
  printf("Limits (%i)\r\nCurrent: fixdt:%li factor%i i_max:%i \r\nSpeed: fixdt:%li factor:%i n_max:%i\r\n",
          cur_spd_valid, input1_fixdt, cur_factor, rtP_Left.i_max, input2_fixdt, spd_factor, rtP_Left.n_max);
  #endif

#endif
}

 /*
 * 静止保持功能
 * 该功能使用巡航控制在静止时提供防溜车功能。
 * 仅在 FOC 电压模式或 FOC 转矩模式下可用且有意义。
 * 
 * 输入：无
 * 输出：standstillAcv
 */
void standstillHold(void) {
  #if defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE)
    if (!rtP_Left.b_cruiseCtrlEna) {                                  // 如果静止保持未激活 -> 尝试激活
      if (((input1[inIdx].cmd > 50 || input2[inIdx].cmd < -50) && speedAvgAbs < 30) // 检查刹车是否按下且测量速度很小
          || (input2[inIdx].cmd < 20 && speedAvgAbs < 5)) {           // 或油门很小且测量速度非常小
        rtP_Left.n_cruiseMotTgt   = 0;
        rtP_Right.n_cruiseMotTgt  = 0;
        rtP_Left.b_cruiseCtrlEna  = 1;
        rtP_Right.b_cruiseCtrlEna = 1;
        standstillAcv = 1;
      } 
    }
    else {                                                            // 如果静止保持已激活 -> 尝试解除
      if (input1[inIdx].cmd < 20 && input2[inIdx].cmd > 50 && !cruiseCtrlAcv) { // 检查刹车是否释放且油门是否按下且未启用巡航控制
        rtP_Left.b_cruiseCtrlEna  = 0;
        rtP_Right.b_cruiseCtrlEna = 0;
        standstillAcv = 0;
      }
    }
  #endif
}

 /*
 * 电动制动功能
 * 在转矩模式下，当输入转矩请求为 0 时，该功能用恒定制动代替电机的“自由滑行”。
 * 这在希望有少量电机制动而不是“自由滑行”时很有用。
 * 
 * 输入：speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * 输出：input2.cmd（油门），包含制动分量
 */
void electricBrake(uint16_t speedBlend, uint8_t reverseDir) {
  #if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
    int16_t brakeVal;

    // 确保制动踏板与运动方向相反，并且在接近静止时归零（以避免倒车） 
    if (speedAvg > 0) {
      brakeVal = (int16_t)((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    } else {
      brakeVal = (int16_t)(( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    }

    // 检查方向是否反转
    if (reverseDir) {
      brakeVal = -brakeVal;
    }

    // 计算包含制动分量的新 input2.cmd
    if (input2[inIdx].cmd >= 0 && input2[inIdx].cmd < ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((ELECTRIC_BRAKE_THRES - input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= -ELECTRIC_BRAKE_THRES && input2[inIdx].cmd < 0) {
      input2[inIdx].cmd = MIN(brakeVal, ((ELECTRIC_BRAKE_THRES + input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((input2[inIdx].cmd - ELECTRIC_BRAKE_THRES) * INPUT_MAX) / (INPUT_MAX - ELECTRIC_BRAKE_THRES));
    } else {  // 当 (input2.cmd < -ELECTRIC_BRAKE_THRES) 时
      input2[inIdx].cmd = MIN(brakeVal, ((input2[inIdx].cmd + ELECTRIC_BRAKE_THRES) * INPUT_MIN) / (INPUT_MIN + ELECTRIC_BRAKE_THRES));
    }
  #endif
}

 /*
 * 巡航控制功能
 * 该功能用于激活/关闭巡航控制。
 * 
 * 输入：button（作为脉冲）
 * 输出：cruiseCtrlAcv
 */
void cruiseControl(uint8_t button) {
  #ifdef CRUISE_CONTROL_SUPPORT
    if (button && !rtP_Left.b_cruiseCtrlEna) {                          // 巡航控制已激活
      rtP_Left.n_cruiseMotTgt   = rtY_Left.n_mot;
      rtP_Right.n_cruiseMotTgt  = rtY_Right.n_mot;
      rtP_Left.b_cruiseCtrlEna  = 1;
      rtP_Right.b_cruiseCtrlEna = 1;
      cruiseCtrlAcv = 1;
      beepShortMany(2, 1);                                              // 200 毫秒蜂鸣延时，同时用作去抖。
    } else if (button && rtP_Left.b_cruiseCtrlEna && !standstillAcv) {  // 如果未激活静止保持，则关闭巡航控制
      rtP_Left.b_cruiseCtrlEna  = 0;
      rtP_Right.b_cruiseCtrlEna = 0;
      cruiseCtrlAcv = 0;
      beepShortMany(2, -1);
    }
  #endif
}
  #ifdef VARIANT_BBCAR
    static float speedRL = 0.0;  // [-1000.0 到 1000.0] 用于高精度内部速度计算
    static float weak = 0.0;  // [-0.0 到 500.0]
    
    extern int16_t speedL, speedR, speed;

    static float acc_cmd = 0.0;
    static float brk_cmd = 0.0;
    
    static float input1_filtered;
    static float input2_filtered;

    extern int8_t drive_mode;

    void adc_error_melody(){
      for(uint8_t i = 0; i < 6; i++) {
        buzzerFreq = 6;
        HAL_Delay(50);
        buzzerFreq = 0;
        HAL_Delay(50);
        buzzerFreq = 8;
        HAL_Delay(50);
        buzzerFreq = 0;
        HAL_Delay(50);
      }
    }

    int32_t isAroundMin(int32_t value, int32_t minn, int32_t maxx){
      // 最小和最大校准值在保存时已减去或加上 ADC_MARGIN。这对计算油门范围有用，但对检测驾驶模式不适用。因此这里使用真实的 ADC 值：
      minn -= ADC_MARGIN;
      maxx += ADC_MARGIN;

      int32_t delta = ((maxx - minn) * 15) / 100;  // *0.15
      // printf("isAroundMin: minn %i, maxx %i, minn_low %i, value %i, minn_high %i, delta: %i, ABS(value - minn): %i\r\n", minn, maxx, minn-delta, value, minn+delta, delta, ABS(value - minn));
      return ABS(value - minn) < delta;
    }

    int32_t isAroundMax(int32_t value, int32_t minn, int32_t maxx){
      minn -= ADC_MARGIN;
      maxx += ADC_MARGIN;

      int32_t delta = ((maxx - minn) * 15) / 100;  // *0.15
      // printf("isAroundMax: minn %i, maxx %i, maxx_low %i, value %i, maxx_high %i, delta: %i, ABS(value - maxx): %i\r\n", minn, maxx, maxx-delta, value, maxx+delta, delta, ABS(value - maxx));
      return ABS(value - maxx) < delta;
    }

     /*
     * 启动时的驾驶模式检测
     * 启动时按住前进或后退电位器的不同组合来选择驾驶模式。
     * 
     * 输入：电位器：完全按下电位器、上电、释放电位器
     * 输出：drive_mode
     */
    void bbcarDetectDrivingMode() {
      readInputRaw();
      input1_filtered = input1[inIdx].min;  // 设置起始值
      input2_filtered = input2[inIdx].min;
      
      printf("\r\n");
      printf("# hoverboard-firmware-hack-FOC larsm's Bobby Car Edition\r\n");
//      printf("# GCC Version: %s\r\n",__VERSION__);
      printf("# Build Date: %s\r\n",__DATE__);
      printf("\r\n");
      printf("# Mode 1: MAX_SPEED_FORWARDS_M1:%i ACC_FORWARDS_M1:%4.2f MAX_SPEED_BACKWARDS_M1:%i ACC_BACKWARDS_M1:%4.2f (no turbo)\r\n", MAX_SPEED_FORWARDS_M1, ACC_FORWARDS_M1, MAX_SPEED_BACKWARDS_M1, ACC_BACKWARDS_M1);
      printf("# Mode 2: MAX_SPEED_FORWARDS_M2:%i ACC_FORWARDS_M2:%4.2f MAX_SPEED_BACKWARDS_M2:%i ACC_BACKWARDS_M2:%4.2f (no turbo)\r\n", MAX_SPEED_FORWARDS_M2, ACC_FORWARDS_M2, MAX_SPEED_BACKWARDS_M2, ACC_BACKWARDS_M2);
      printf("# Mode 3: MAX_SPEED_FORWARDS_M3:%i ACC_FORWARDS_M3:%4.2f MAX_SPEED_BACKWARDS_M3:%i ACC_BACKWARDS_M3:%4.2f (no turbo)\r\n", MAX_SPEED_FORWARDS_M3, ACC_FORWARDS_M3, MAX_SPEED_BACKWARDS_M3, ACC_BACKWARDS_M3);
      printf("# Mode 4: MAX_SPEED_FORWARDS_M4:%i ACC_FORWARDS_M4:%4.2f MAX_SPEED_BACKWARDS_M4:%i ACC_BACKWARDS_M4:%4.2f (turbo)\r\n", MAX_SPEED_FORWARDS_M4, ACC_FORWARDS_M4, MAX_SPEED_BACKWARDS_M4, ACC_BACKWARDS_M4);
      printf("# ADC_MARGIN: %i\r\n", ADC_MARGIN);
      printf("\r\n");

      int16_t start_left  = input2[inIdx].raw;  // ADC2，左侧，后退，绿色
      int16_t start_right = input1[inIdx].raw;  // ADC1，右侧，前进，蓝色
      HAL_Delay(300);
      if(isAroundMax(start_left, input2[inIdx].min, input2[inIdx].max) && isAroundMax(start_right, input1[inIdx].min, input1[inIdx].max)){  // 模式 4
        drive_mode = 4;
        beepShortMany2(4);
      } else if(isAroundMin(start_left, input2[inIdx].min, input2[inIdx].max) && isAroundMax(start_right, input1[inIdx].min, input1[inIdx].max)){  // 模式 3
        drive_mode = 3;
        beepShortMany2(3);
      } else if(isAroundMax(start_left, input2[inIdx].min, input2[inIdx].max) && isAroundMin(start_right, input1[inIdx].min, input1[inIdx].max)){  // 模式 1
        drive_mode = 1;
        beepShortMany2(1);
      } else if(isAroundMin(start_left, input2[inIdx].min, input2[inIdx].max) && isAroundMin(start_right, input1[inIdx].min, input1[inIdx].max)) {  // 模式 2
        drive_mode = 2;
        beepShortMany2(2);
      }
      printf("# Input1 (right): %i, Input2 (left): %i\r\n", start_right, start_left);
      if(drive_mode == 0){
        printf("# Driving mode detection failed. Potis have to be either near min or near max position.\r\n");
        printf("# Are your potis calibrated? To calibrate: Poweroff, then poweron and hold power button until beep (10s). After a second you will hear a lower beep. Then move both potis to min and max. With potis in min position (which equals min position in this case), short press power button (or wait a few seconds) to save values.\r\n");
        poweroff();
      }
      printf("# Mode: %i\r\n", drive_mode);
      printf("# waiting for poti release...\r\n");
      int counter = 0;
      while(!isAroundMin(input2[inIdx].raw, input2[inIdx].min, input2[inIdx].max) || !isAroundMin(input1[inIdx].raw, input1[inIdx].min, input1[inIdx].max)){
        readInputRaw();
        if(counter++ > 50){
          printf("# potis are not around min position for 5 sec. powering off...\r\n");
          printf("# Input1 (right): %i, Input2 (left): %i\r\n", start_right, start_left);
          poweroff();
        }
        HAL_Delay(100); 
      }
      printf("# potis released\r\n");
      printf("# driving mode detection done\r\n");
    }

    int16_t bbcarLoop() {
      #define INPUT_MAX 1000  
      #define INPUT_MIN -1000 

      input1_filtered = input1_filtered * 0.9 + (float)input1[inIdx].raw * 0.1;  
      input2_filtered = input2_filtered * 0.9 + (float)input2[inIdx].raw * 0.1;  

      acc_cmd = CLAMP((input1_filtered - input1[inIdx].min) / (input1[inIdx].max - input1[inIdx].min), 0, 1.0);
      brk_cmd = CLAMP((input2_filtered - input2[inIdx].min) / (input2[inIdx].max - input2[inIdx].min), 0, 1.0);

      if (timeoutFlgADC || timeoutFlgSerial || timeoutFlgGen || adc_error_break_and_poweroff) {  

           adc_error_break_and_poweroff = 1;

           if (ABS((int)speedRL) < 5) { 
          printf("# Poti significantly out of range:\r\n");
          printf("# Input1: %i, Input2: %i\r\n", input1[inIdx].raw, input2[inIdx].raw);
          printf("# Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\n# Limits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
          input1[inIdx].typ, input1[inIdx].min, input1[inIdx].mid, input1[inIdx].max,
          input2[inIdx].typ, input2[inIdx].min, input2[inIdx].mid, input2[inIdx].max);
          printf("# power off\r\n");
          adc_error_melody();
          poweroff();
        
        }
        acc_cmd = brk_cmd = 0.0;
        
      }

      if (drive_mode == 1) {  
        speedRL = speedRL * (1.0 - (speedRL > 0 ? ACC_FORWARDS_M1/MAX_SPEED_FORWARDS_M1*5.0 : ACC_BACKWARDS_M1/MAX_SPEED_BACKWARDS_M1*5.0))  // 电位器未按下时减速
                + acc_cmd * ACC_FORWARDS_M1*5.0  
                - brk_cmd * ACC_BACKWARDS_M1*5.0;  

      } else if (drive_mode == 2) {  
        speedRL = speedRL * (1.0 - (speedRL > 0 ? ACC_FORWARDS_M2/MAX_SPEED_FORWARDS_M2*5.0 : ACC_BACKWARDS_M2/MAX_SPEED_BACKWARDS_M2*5.0))  
                + acc_cmd * ACC_FORWARDS_M2*5.0  
                - brk_cmd * ACC_BACKWARDS_M2*5.0;  

      } else if (drive_mode == 3) {  
        speedRL = speedRL * (1.0 - (speedRL > 0 ? ACC_FORWARDS_M3/MAX_SPEED_FORWARDS_M3*5.0 : ACC_BACKWARDS_M3/MAX_SPEED_BACKWARDS_M3*5.0)) 
                + acc_cmd * ACC_FORWARDS_M3*5.0  
                - brk_cmd * ACC_BACKWARDS_M3*5.0;  

      } else if (drive_mode == 4) {  
        if (adc_error_break_and_poweroff) {  
          float acc_forwards_m4 = 0.5;  
          speedRL = speedRL * (1.0 - (speedRL > 0 ? acc_forwards_m4/MAX_SPEED_FORWARDS_M4*5.0 : ACC_BACKWARDS_M4/MAX_SPEED_BACKWARDS_M4*5.0));  
          weak = weak * 0.985;  
        } else if (acc_cmd > 0.8 & brk_cmd > 0.8 & speedRL > 0.7 * (float)INPUT_MAX){ 
          speedRL = speedRL * (1.0 - (speedRL > 0 ? ACC_FORWARDS_M4/MAX_SPEED_FORWARDS_M4*5.0 : ACC_BACKWARDS_M4/MAX_SPEED_BACKWARDS_M4*5.0)) 
                  + acc_cmd * ACC_FORWARDS_M4*5.0; 
          weak = weak * 0.95 + 500.0 * 0.05;  
        } else {  
          speedRL = speedRL * (1.0 - (speedRL > 0 ? ACC_FORWARDS_M4/MAX_SPEED_FORWARDS_M4*5.0 : ACC_BACKWARDS_M4/MAX_SPEED_BACKWARDS_M4*5.0))  // 电位器未按下时减速
                  + acc_cmd * ACC_FORWARDS_M4*5.0  
                  - brk_cmd * ACC_BACKWARDS_M4*5.0;  
          weak = weak * 0.95;  
        }
      }

      return CLAMP((int16_t)(speedRL + weak), INPUT_MIN, FIELD_WEAK_HI);  
    }
  #endif




 /*
 * 检查输入类型
 * 该函数识别输入类型：0：禁用，1：普通电位器，2：中位回弹电位器
 */
int checkInputType(int16_t min, int16_t mid, int16_t max){

  int type = 0;  
  #ifdef CONTROL_ADC
  int16_t threshold = 400;      // 定义数值是否过于接近的阈值
  #else
  int16_t threshold = 200;
  #endif

  if ((min / threshold) == (max / threshold) || (mid / threshold) == (max / threshold) || min > max || mid > max) {
    type = 0;
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    printf("ignored");                // （MIN 和 MAX）或（MID 和 MAX）过于接近，禁用输入
    #endif
  } else {
    if ((min / threshold) == (mid / threshold)){
      type = 1;
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      printf("a normal pot");        // MIN 和 MID 接近，是普通电位器
      #endif
    } else {
      type = 2;
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      printf("a mid-resting pot");   // 是中位回弹电位器
      #endif
    }

    #ifdef CONTROL_ADC
    if ((min + ADC_MARGIN - ADC_PROTECT_THRESH) > 0 && (max - ADC_MARGIN + ADC_PROTECT_THRESH) < 4095) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      printf(" AND protected");
      #endif
      beepLong(2); // 通过蜂鸣声指示保护已启用
    }
    #endif
  }

  return type;
}



/* =========================== 输入函数 =========================== */

 /*
 * 计算输入命令
 * 该函数在 0 附近实现死区，并将输入缩放到 [out_min, out_max] 之间
 */
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max) {
  switch (in->typ){
    case 1: // 输入是普通电位器
      in->cmd = CLAMP(MAP(in->raw, in->min, in->max, 0, out_max), 0, out_max);
      break;
    case 2: // 输入是中位回弹电位器
      if( in->raw > in->mid - in->dband && in->raw < in->mid + in->dband ) {
        in->cmd = 0;
      } else if(in->raw > in->mid) {
        in->cmd = CLAMP(MAP(in->raw, in->mid + in->dband, in->max, 0, out_max), 0, out_max);
      } else {
        in->cmd = CLAMP(MAP(in->raw, in->mid - in->dband, in->min, 0, out_min), out_min, 0);
      }
      break;
    default: // 输入被忽略
      in->cmd = 0;
      break;
  }
}

 /*
 * 从各种输入设备读取原始输入值的函数
 */
void readInputRaw(void) {
    #ifdef CONTROL_ADC
    if (inIdx == CONTROL_ADC) {
      #ifdef ADC_ALTERNATE_CONNECT
        input1[inIdx].raw = adc_buffer.l_rx2;
        input2[inIdx].raw = adc_buffer.l_tx2;
      #else
        input1[inIdx].raw = adc_buffer.l_tx2;
        input2[inIdx].raw = adc_buffer.l_rx2;
      #endif
    }
    #endif

    #if defined(CONTROL_NUNCHUK) || defined(SUPPORT_NUNCHUK)
    if (Nunchuk_Read() == NUNCHUK_CONNECTED) {
      if (inIdx == CONTROL_NUNCHUK) {
        input1[inIdx].raw = (nunchuk_data[0] - 127) * 8; // X 轴 0-255
        input2[inIdx].raw = (nunchuk_data[1] - 128) * 8; // Y 轴 0-255
      }
      #ifdef SUPPORT_BUTTONS
        button1 = (uint8_t)nunchuk_data[5] & 1;
        button2 = (uint8_t)(nunchuk_data[5] >> 1) & 1;
      #endif
    }
    #endif

    #if defined(CONTROL_SERIAL_USART2)
    if (inIdx == CONTROL_SERIAL_USART2) {
      #ifdef CONTROL_IBUS
        for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i+=2) {
          ibusL_captured_value[(i/2)] = CLAMP(commandL.channels[i] + (commandL.channels[i+1] << 8) - 1000, 0, INPUT_MAX); // 1000-2000 -> 0-1000
        }
        input1[inIdx].raw = (ibusL_captured_value[0] - 500) * 2;
        input2[inIdx].raw = (ibusL_captured_value[1] - 500) * 2; 
      #else
        input1[inIdx].raw = commandL.steer;
        input2[inIdx].raw = commandL.speed;
      #endif
    }
    #endif
    #if defined(CONTROL_SERIAL_USART3)
    if (inIdx == CONTROL_SERIAL_USART3) {
      #ifdef CONTROL_IBUS
        for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i+=2) {
          ibusR_captured_value[(i/2)] = CLAMP(commandR.channels[i] + (commandR.channels[i+1] << 8) - 1000, 0, INPUT_MAX); // 1000-2000 -> 0-1000
        }
        input1[inIdx].raw = (ibusR_captured_value[0] - 500) * 2;
        input2[inIdx].raw = (ibusR_captured_value[1] - 500) * 2; 
      #else
        input1[inIdx].raw = commandR.steer;
        input2[inIdx].raw = commandR.speed;
      #endif
    }
    #endif

    #if defined(SIDEBOARD_SERIAL_USART2)
    if (inIdx == SIDEBOARD_SERIAL_USART2) {
      input1[inIdx].raw = Sideboard_L.cmd1;
      input2[inIdx].raw = Sideboard_L.cmd2;
    }
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
    if (inIdx == SIDEBOARD_SERIAL_USART3) {
      input1[inIdx].raw = Sideboard_R.cmd1;
      input2[inIdx].raw = Sideboard_R.cmd2;
    }
    #endif

    #if defined(CONTROL_PPM_LEFT)
    if (inIdx == CONTROL_PPM_LEFT) {
      input1[inIdx].raw = (ppm_captured_value[0] - 500) * 2;
      input2[inIdx].raw = (ppm_captured_value[1] - 500) * 2;
    }
    #endif
    #if defined(CONTROL_PPM_RIGHT)
    if (inIdx == CONTROL_PPM_RIGHT) {
      input1[inIdx].raw = (ppm_captured_value[0] - 500) * 2;
      input2[inIdx].raw = (ppm_captured_value[1] - 500) * 2;
    }
    #endif
    #if (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)) && defined(SUPPORT_BUTTONS)
      button1 = ppm_captured_value[5] > 500;
      button2 = 0;
    #endif

    #if defined(CONTROL_PWM_LEFT)
    if (inIdx == CONTROL_PWM_LEFT) {
      input1[inIdx].raw = (pwm_captured_ch1_value - 500) * 2;
      input2[inIdx].raw = (pwm_captured_ch2_value - 500) * 2;
    }
    #endif
    #if defined(CONTROL_PWM_RIGHT)
    if (inIdx == CONTROL_PWM_RIGHT) {
      input1[inIdx].raw = (pwm_captured_ch1_value - 500) * 2;
      input2[inIdx].raw = (pwm_captured_ch2_value - 500) * 2;
    }
    #endif

    #ifdef VARIANT_TRANSPOTTER
      #ifdef GAMETRAK_CONNECTION_NORMAL
        input1[inIdx].cmd = adc_buffer.l_rx2;
        input2[inIdx].cmd = adc_buffer.l_tx2;
      #endif
      #ifdef GAMETRAK_CONNECTION_ALTERNATE
        input1[inIdx].cmd = adc_buffer.l_tx2;
        input2[inIdx].cmd = adc_buffer.l_rx2;
      #endif
    #endif
}

 /*
 * 处理 ADC、UART 和通用超时（Nunchuk、PPM、PWM）的函数
 */
void handleTimeout(void) {
    #ifdef CONTROL_ADC
    if (inIdx == CONTROL_ADC) {
      // 如果 input1 或 Input2 低于 MIN - 阈值或高于 MAX + 阈值，则触发 ADC 保护超时
      if (IN_RANGE(input1[inIdx].raw, input1[inIdx].min - ADC_PROTECT_THRESH, input1[inIdx].max + ADC_PROTECT_THRESH) &&
          IN_RANGE(input2[inIdx].raw, input2[inIdx].min - ADC_PROTECT_THRESH, input2[inIdx].max + ADC_PROTECT_THRESH)) {
          timeoutFlgADC = 0;                            // 重置超时标志
          timeoutCntADC = 0;                            // 重置超时计数器
      } else {
        if (timeoutCntADC++ >= ADC_PROTECT_TIMEOUT) {   // 超时判定
          timeoutFlgADC = 1;                            // 检测到超时
          timeoutCntADC = ADC_PROTECT_TIMEOUT;          // 限制超时计数器数值
        }
      }
    }
    #endif

    #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
      if (timeoutCntSerial_L++ >= SERIAL_TIMEOUT) {     // 超时判定
        timeoutFlgSerial_L = 1;                         // 检测到超时
        timeoutCntSerial_L = SERIAL_TIMEOUT;            // 限制超时计数器数值
        #if defined(DUAL_INPUTS) && ((defined(CONTROL_SERIAL_USART2) && CONTROL_SERIAL_USART2 == 1) || (defined(SIDEBOARD_SERIAL_USART2) && SIDEBOARD_SERIAL_USART2 == 1))
          inIdx = 0;                                    // 辅助输入超时时切换到主输入
        #endif
      } else {                                          // 未超时
        #if defined(DUAL_INPUTS) && defined(SIDEBOARD_SERIAL_USART2)
          if (Sideboard_L.sensors & SWA_SET) {          // 如果 SWA 已设置，切换到侧板控制
            inIdx = SIDEBOARD_SERIAL_USART2;
          } else {
            inIdx = !SIDEBOARD_SERIAL_USART2;
          }
        #elif defined(DUAL_INPUTS) && (defined(CONTROL_SERIAL_USART2) && CONTROL_SERIAL_USART2 == 1)
          inIdx = 1;                                    // 辅助输入未超时时切换到辅助输入
        #endif
      }
      #if (defined(CONTROL_SERIAL_USART2) && CONTROL_SERIAL_USART2 == 0) || (defined(SIDEBOARD_SERIAL_USART2) && SIDEBOARD_SERIAL_USART2 == 0 && !defined(VARIANT_HOVERBOARD))
        timeoutFlgSerial = timeoutFlgSerial_L;          // 仅在主输入上报告超时
      #endif
    #endif

    #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
      if (timeoutCntSerial_R++ >= SERIAL_TIMEOUT) {     // 超时判定
        timeoutFlgSerial_R = 1;                         // 检测到超时
        timeoutCntSerial_R = SERIAL_TIMEOUT;            // 限制超时计数器数值
        #if defined(DUAL_INPUTS) && ((defined(CONTROL_SERIAL_USART3) && CONTROL_SERIAL_USART3 == 1) || (defined(SIDEBOARD_SERIAL_USART3) && SIDEBOARD_SERIAL_USART3 == 1))
          inIdx = 0;                                    // 辅助输入超时时切换到主输入
        #endif
      } else {                                          // 未超时
        #if defined(DUAL_INPUTS) && defined(SIDEBOARD_SERIAL_USART3)
          if (Sideboard_R.sensors & SWA_SET) {          // 如果 SWA 已设置，切换到侧板控制
            inIdx = SIDEBOARD_SERIAL_USART3;
          } else {
            inIdx = !SIDEBOARD_SERIAL_USART3;
          }
        #elif defined(DUAL_INPUTS) && (defined(CONTROL_SERIAL_USART3) && CONTROL_SERIAL_USART3 == 1)
          inIdx = 1;                                    // 辅助输入未超时时切换到辅助输入
        #endif
      }
      #if (defined(CONTROL_SERIAL_USART3) && CONTROL_SERIAL_USART3 == 0) || (defined(SIDEBOARD_SERIAL_USART3) && SIDEBOARD_SERIAL_USART3 == 0 && !defined(VARIANT_HOVERBOARD))
        timeoutFlgSerial = timeoutFlgSerial_R;          // 仅在主输入上报告超时
      #endif
    #endif

    #if defined(SIDEBOARD_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART3)
      timeoutFlgSerial = timeoutFlgSerial_L || timeoutFlgSerial_R;
    #endif

    #if defined(CONTROL_NUNCHUK) || defined(SUPPORT_NUNCHUK) || defined(VARIANT_TRANSPOTTER) || \
        defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_LEFT) || defined(CONTROL_PWM_RIGHT)
      if (timeoutCntGen++ >= TIMEOUT) {                 // 超时判定
        #if defined(CONTROL_NUNCHUK) || defined(SUPPORT_NUNCHUK) || defined(VARIANT_TRANSPOTTER) || \
            (defined(CONTROL_PPM_LEFT) && CONTROL_PPM_LEFT == 0) || (defined(CONTROL_PPM_RIGHT) && CONTROL_PPM_RIGHT == 0) || \
            (defined(CONTROL_PWM_LEFT) && CONTROL_PWM_LEFT == 0) || (defined(CONTROL_PWM_RIGHT) && CONTROL_PWM_RIGHT == 0)
          timeoutFlgGen = 1;                            // 仅在主输入上报告超时
          timeoutCntGen = TIMEOUT;
        #endif
        #if defined(DUAL_INPUTS) && ((defined(CONTROL_PPM_LEFT)  && CONTROL_PPM_LEFT == 1) || (defined(CONTROL_PPM_RIGHT) && CONTROL_PPM_RIGHT == 1) || \
                                     (defined(CONTROL_PWM_LEFT)  && CONTROL_PWM_LEFT == 1) || (defined(CONTROL_PWM_RIGHT) && CONTROL_PWM_RIGHT == 1))
          inIdx = 0;                                    // 辅助输入超时时切换到主输入
        #endif
      } else {
        #if defined(DUAL_INPUTS) && ((defined(CONTROL_PPM_LEFT)  && CONTROL_PPM_LEFT == 1) || (defined(CONTROL_PPM_RIGHT) && CONTROL_PPM_RIGHT == 1) || \
                                     (defined(CONTROL_PWM_LEFT)  && CONTROL_PWM_LEFT == 1) || (defined(CONTROL_PWM_RIGHT) && CONTROL_PWM_RIGHT == 1))
          inIdx = 1;                                    // 辅助输入未超时时切换到辅助输入
        #endif
      }
    #endif

    #ifndef VARIANT_BBCAR
      // 发生超时时将系统带入安全状态
      if (timeoutFlgADC || timeoutFlgSerial || timeoutFlgGen) {
        ctrlModReq  = OPEN_MODE;                                          // 请求 OPEN_MODE，这将使电机功率以受控方式降为 0
        input1[inIdx].cmd  = 0;
        input2[inIdx].cmd  = 0;
      } else {
        ctrlModReq  = ctrlModReqRaw;                                      // 跟随模式请求
      }
    #endif

    // 输入索引变化时发出蜂鸣
    if (inIdx && !inIdx_prev) {                                         // 上升沿
      beepShort(8);
    } else if (!inIdx && inIdx_prev) {                                  // 下降沿
      beepShort(18);
    }
}

 /*
 * 计算电机命令的函数。该函数还管理：
 * - 超时检测
 * - 最小/最大限制和死区
 */
void readCommand(void) {
    readInputRaw();

    #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
      calcInputCmd(&input1[inIdx], INPUT_MIN, INPUT_MAX);
      #if !defined(VARIANT_SKATEBOARD)
        calcInputCmd(&input2[inIdx], INPUT_MIN, INPUT_MAX);
      #else
        calcInputCmd(&input2[inIdx], INPUT_BRK, INPUT_MAX);
      #endif
    #endif

    handleTimeout();

    #ifdef VARIANT_HOVERCAR
    if (inIdx == CONTROL_ADC) {
      brakePressed = (uint8_t)(input1[inIdx].cmd > 50);
    }
    else {
      brakePressed = (uint8_t)(input2[inIdx].cmd < -50);
    }
    #endif

    #if defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT)
      button1 = !HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN);
      button2 = !HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN);
    #endif

    #if defined(CRUISE_CONTROL_SUPPORT) && (defined(SUPPORT_BUTTONS) || defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT))
        cruiseControl(button1);                                           // 巡航控制激活/停用
    #endif
}


/*
 * 检查 USART2 DMA 接收的新数据：来自 https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx 的重构函数
 * - 该函数在每次 USART 空闲线检测时，在 USART 中断处理程序中被调用
 */
void usart2_rx_check(void)
{
  #if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)  
  static uint32_t old_pos;
  uint32_t pos;
  pos = rx_buffer_L_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx);         // 计算缓冲区中的当前位置
  #endif

  #if defined(DEBUG_SERIAL_USART2)
  uint8_t ptr_debug[SERIAL_BUFFER_SIZE];
  if (pos != old_pos) {                                                 // 检查接收数据的变化
    if (pos > old_pos) {                                                // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后
      usart_process_debug(&rx_buffer_L[old_pos], pos - old_pos);        // 处理数据
    } else {                                                            // "溢出" 缓冲区模式
      memcpy(&ptr_debug[0], &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        memcpy(&ptr_debug[rx_buffer_L_len - old_pos], &rx_buffer_L[0], pos);                              // 复制剩余数据
      }
      usart_process_debug(ptr_debug, rx_buffer_L_len - old_pos + pos);        // 处理数据
    }
  }
  #endif // DEBUG_SERIAL_USART2

  #ifdef CONTROL_SERIAL_USART2
  uint8_t *ptr;	
  if (pos != old_pos) {                                                 // 检查接收数据的变化
    ptr = (uint8_t *)&commandL_raw;                                     // 用 command_raw 地址初始化指针
    if (pos > old_pos && (pos - old_pos) == commandL_len) {             // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后且数据长度等于预期长度
      memcpy(ptr, &rx_buffer_L[old_pos], commandL_len);                 // 复制数据。仅在 command_raw 连续时才可能！（即所有结构成员大小相同）
      usart_process_command(&commandL_raw, &commandL, 2);               // 处理数据
    } else if ((rx_buffer_L_len - old_pos + pos) == commandL_len) {     // "溢出" 缓冲区模式：检查数据长度是否等于预期长度
      memcpy(ptr, &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        ptr += rx_buffer_L_len - old_pos;                               // 移动到 command_raw 中的正确位置
        memcpy(ptr, &rx_buffer_L[0], pos);                              // 复制剩余数据
      }
      usart_process_command(&commandL_raw, &commandL, 2);               // 处理数据
    }
  }
  #endif // CONTROL_SERIAL_USART2

  #ifdef SIDEBOARD_SERIAL_USART2
  uint8_t *ptr;	
  if (pos != old_pos) {                                                 // 检查接收数据的变化
    ptr = (uint8_t *)&Sideboard_L_raw;                                  // 用 Sideboard_raw 地址初始化指针
    if (pos > old_pos && (pos - old_pos) == Sideboard_L_len) {          // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后且数据长度等于预期长度
      memcpy(ptr, &rx_buffer_L[old_pos], Sideboard_L_len);              // 复制数据。仅在 Sideboard_raw 连续时才可能！（即所有结构成员大小相同）
      usart_process_sideboard(&Sideboard_L_raw, &Sideboard_L, 2);       // 处理数据
    } else if ((rx_buffer_L_len - old_pos + pos) == Sideboard_L_len) {  // "溢出" 缓冲区模式：检查数据长度是否等于预期长度
      memcpy(ptr, &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        ptr += rx_buffer_L_len - old_pos;                               // 移动到 Sideboard_raw 中的正确位置
        memcpy(ptr, &rx_buffer_L[0], pos);                              // 复制剩余数据
      }
      usart_process_sideboard(&Sideboard_L_raw, &Sideboard_L, 2);       // 处理数据
    }
  }
  #endif // SIDEBOARD_SERIAL_USART2

  #if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  old_pos = pos;                                                        // 更新旧位置
  if (old_pos == rx_buffer_L_len) {                                     // 检查是否到达缓冲区末尾并手动更新
    old_pos = 0;
  }
	#endif
}


/*
 * 检查 USART3 DMA 接收的新数据：来自 https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx 的重构函数
 * - 该函数在每次 USART 空闲线检测时，在 USART 中断处理程序中被调用
 */
void usart3_rx_check(void)
{
  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  static uint32_t old_pos;
  uint32_t pos;  
  pos = rx_buffer_R_len - __HAL_DMA_GET_COUNTER(huart3.hdmarx);         // 计算缓冲区中的当前位置
  #endif

  #if defined(DEBUG_SERIAL_USART3)
  uint8_t ptr_debug[SERIAL_BUFFER_SIZE];

  if (pos != old_pos) {                                                 // 检查接收数据的变化
    if (pos > old_pos) {                                                // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后
      usart_process_debug(&rx_buffer_R[old_pos], pos - old_pos);        // 处理数据
    } else {                                                            // "溢出" 缓冲区模式
      memcpy(&ptr_debug[0], &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        memcpy(&ptr_debug[rx_buffer_R_len - old_pos], &rx_buffer_R[0], pos);                              // 复制剩余数据
      }
      usart_process_debug(ptr_debug, rx_buffer_R_len - old_pos + pos);        // 处理数据
    }
  }
  #endif // DEBUG_SERIAL_USART3

  #ifdef CONTROL_SERIAL_USART3
  uint8_t *ptr;
  if (pos != old_pos) {                                                 // 检查接收数据的变化
    ptr = (uint8_t *)&commandR_raw;                                     // 用 command_raw 地址初始化指针
    if (pos > old_pos && (pos - old_pos) == commandR_len) {             // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后且数据长度等于预期长度
      memcpy(ptr, &rx_buffer_R[old_pos], commandR_len);                 // 复制数据。仅在 command_raw 连续时才可能！（即所有结构成员大小相同）
      usart_process_command(&commandR_raw, &commandR, 3);               // 处理数据
    } else if ((rx_buffer_R_len - old_pos + pos) == commandR_len) {     // "溢出" 缓冲区模式：检查数据长度是否等于预期长度
      memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        ptr += rx_buffer_R_len - old_pos;                               // 移动到 command_raw 中的正确位置
        memcpy(ptr, &rx_buffer_R[0], pos);                              // 复制剩余数据
      }
      usart_process_command(&commandR_raw, &commandR, 3);               // 处理数据
    }
  }
  #endif // CONTROL_SERIAL_USART3

  #ifdef SIDEBOARD_SERIAL_USART3
  uint8_t *ptr;
  if (pos != old_pos) {                                                 // 检查接收数据的变化
    ptr = (uint8_t *)&Sideboard_R_raw;                                  // 用 Sideboard_raw 地址初始化指针
    if (pos > old_pos && (pos - old_pos) == Sideboard_R_len) {          // "线性" 缓冲区模式：检查当前位置是否在前一个位置之后且数据长度等于预期长度
      memcpy(ptr, &rx_buffer_R[old_pos], Sideboard_R_len);              // 复制数据。仅在 Sideboard_raw 连续时才可能！（即所有结构成员大小相同）
      usart_process_sideboard(&Sideboard_R_raw, &Sideboard_R, 3);       // 处理数据
    } else if ((rx_buffer_R_len - old_pos + pos) == Sideboard_R_len) {  // "溢出" 缓冲区模式：检查数据长度是否等于预期长度
      memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // 首先从缓冲区末尾复制数据
      if (pos > 0) {                                                    // 检查并继续从缓冲区开头复制
        ptr += rx_buffer_R_len - old_pos;                               // 移动到 Sideboard_raw 中的正确位置
        memcpy(ptr, &rx_buffer_R[0], pos);                              // 复制剩余数据
      }
      usart_process_sideboard(&Sideboard_R_raw, &Sideboard_R, 3);       // 处理数据
    }
  }
  #endif // SIDEBOARD_SERIAL_USART3

  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  old_pos = pos;                                                        // 更新旧位置
  if (old_pos == rx_buffer_R_len) {                                     // 检查是否到达缓冲区末尾并手动更新
    old_pos = 0;
  }
  #endif
}

/*
 * 处理接收到的调试用户命令输入
 */
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
void usart_process_debug(uint8_t *userCommand, uint32_t len)
{
  #ifdef DEBUG_SERIAL_PROTOCOL
    handle_input(userCommand, len);
  #endif
}

#endif // SERIAL_DEBUG

/*
 * 处理命令接收数据
 * - 如果 command_in 数据有效（正确的起始帧和校验和），则将 command_in 复制到 command_out
 */
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx)
{
  #ifdef CONTROL_IBUS
    uint16_t ibus_chksum;
    if (command_in->start == IBUS_LENGTH && command_in->type == IBUS_COMMAND) {
      ibus_chksum = 0xFFFF - IBUS_LENGTH - IBUS_COMMAND;
      for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i++) {
        ibus_chksum -= command_in->channels[i];
      }
      if (ibus_chksum == (uint16_t)((command_in->checksumh << 8) + command_in->checksuml)) {
        *command_out = *command_in;
        if (usart_idx == 2) {             // 侧板 USART2
          #ifdef CONTROL_SERIAL_USART2
          timeoutFlgSerial_L = 0;         // 清除超时标志
          timeoutCntSerial_L = 0;         // 重置超时计数器
          #endif
        } else if (usart_idx == 3) {      // 侧板 USART3
          #ifdef CONTROL_SERIAL_USART3
          timeoutFlgSerial_R = 0;         // 清除超时标志
          timeoutCntSerial_R = 0;         // 重置超时计数器
          #endif
        }
      }
    }
  #else
  uint16_t checksum;
  if (command_in->start == SERIAL_START_FRAME) {
    checksum = (uint16_t)(command_in->start ^ command_in->steer ^ command_in->speed);
    if (command_in->checksum == checksum) {
      *command_out = *command_in;
      if (usart_idx == 2) {             // 侧板 USART2
        #ifdef CONTROL_SERIAL_USART2
        timeoutFlgSerial_L = 0;         // 清除超时标志
        timeoutCntSerial_L = 0;         // 重置超时计数器
        #endif
      } else if (usart_idx == 3) {      // 侧板 USART3
        #ifdef CONTROL_SERIAL_USART3
        timeoutFlgSerial_R = 0;         // 清除超时标志
        timeoutCntSerial_R = 0;         // 重置超时计数器
        #endif
      }
    }
  }
  #endif
}
#endif

/*
 * 处理侧板接收数据
 * - 如果 Sideboard_in 数据有效（正确的起始帧和校验和），则将 Sideboard_in 复制到 Sideboard_out
 */
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
void usart_process_sideboard(SerialSideboard *Sideboard_in, SerialSideboard *Sideboard_out, uint8_t usart_idx)
{
  uint16_t checksum;
  if (Sideboard_in->start == SERIAL_START_FRAME) {
    checksum = (uint16_t)(Sideboard_in->start ^ Sideboard_in->pitch ^ Sideboard_in->dPitch ^ Sideboard_in->cmd1 ^ Sideboard_in->cmd2 ^ Sideboard_in->sensors);
    if (Sideboard_in->checksum == checksum) {
      *Sideboard_out = *Sideboard_in;
      if (usart_idx == 2) {             // 侧板 USART2
        #ifdef SIDEBOARD_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // 重置超时计数器
        timeoutFlgSerial_L = 0;         // 清除超时标志
        #endif
      } else if (usart_idx == 3) {      // 侧板 USART3
        #ifdef SIDEBOARD_SERIAL_USART3
        timeoutCntSerial_R = 0;         // 重置超时计数器
        timeoutFlgSerial_R = 0;         // 清除超时标志
        #endif
      }
    }
  }
}
#endif


/* =========================== 侧板函数 =========================== */

/*
 * 侧板 LED 处理
 * 该函数管理连接到侧板的 LED 行为
 */
void sideboardLeds(uint8_t *leds) {
  #if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
    // 使能标志：使用 LED4（底部蓝色）
    // enable == 1，点亮 LED
    // enable == 0，LED 闪烁
    if (enable) {
      *leds |= LED4_SET;
    } else if (!enable && (main_loop_counter % 20 == 0)) {
      *leds ^= LED4_SET;
    }

    // 倒车：使用 LED5（上部蓝色）
    // backwardDrive == 1，LED 闪烁
    // backwardDrive == 0，关闭 LED
    if (backwardDrive && (main_loop_counter % 50 == 0)) {
      *leds ^= LED5_SET;
    }

    // 刹车：使用 LED5（上部蓝色）
    // brakePressed == 1，点亮 LED
    // brakePressed == 0，关闭 LED
    #ifdef VARIANT_HOVERCAR
      if (brakePressed) {
        *leds |= LED5_SET;
      } else if (!brakePressed && !backwardDrive) {
        *leds &= ~LED5_SET;
      }
    #endif

    // 电池电量指示：使用 LED1、LED2、LED3
    if (main_loop_counter % BAT_BLINK_INTERVAL == 0) {              //  | 红色 (LED1) | 黄色 (LED3) | 绿色 (LED2) |
      if (batVoltage < BAT_DEAD) {                                  //  |     0      |       0       |      0       |
        *leds &= ~LED1_SET & ~LED3_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL1) {                           //  |     B      |       0       |      0       |
        *leds ^= LED1_SET;
        *leds &= ~LED3_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL2) {                           //  |     1      |       0       |      0       |
        *leds |= LED1_SET;
        *leds &= ~LED3_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL3) {                           //  |     0      |       B       |      0       |
        *leds ^= LED3_SET;
        *leds &= ~LED1_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL4) {                           //  |     0      |       1       |      0       |
        *leds |= LED3_SET;
        *leds &= ~LED1_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL5) {                           //  |     0      |       0       |      B       |
        *leds ^= LED2_SET;
        *leds &= ~LED1_SET & ~LED3_SET;
      } else {                                                      //  |     0      |       0       |      1       |
        *leds |= LED2_SET;
        *leds &= ~LED1_SET & ~LED3_SET;
      }
    }

    // 错误处理
    // 严重错误：LED1 亮（红色）+ 高音蜂鸣（在主程序中处理）
    // 轻微错误：LED3 亮（黄色）+ 低音蜂鸣（在主程序中处理）
    if (rtY_Left.z_errCode || rtY_Right.z_errCode) {
      *leds |= LED1_SET;
      *leds &= ~LED3_SET & ~LED2_SET;
    }
    if (timeoutFlgADC || timeoutFlgSerial) {
      *leds |= LED3_SET;
      *leds &= ~LED1_SET & ~LED2_SET;
    }
  #endif
}

/*
 * 侧板传感器处理
 * 该函数管理侧板的光电传感器。
 * 在非平衡车变体中，传感器用作按键。
 */
void sideboardSensors(uint8_t sensors) {
  #if !defined(VARIANT_HOVERBOARD) && (defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3))
    static uint8_t sensor1_index;                                 // 当用作按钮时，保存 sensor1 的按下索引号
    static uint8_t sensor1_prev,  sensor2_prev;
    uint8_t sensor1_trig = 0, sensor2_trig = 0;
    #if defined(SIDEBOARD_SERIAL_USART2)
    uint8_t  sideboardIdx = SIDEBOARD_SERIAL_USART2;
    uint16_t sideboardSns = Sideboard_L.sensors;
    #else
    uint8_t  sideboardIdx = SIDEBOARD_SERIAL_USART3;
    uint16_t sideboardSns = Sideboard_R.sensors;
    #endif

    if (inIdx == sideboardIdx) {                                  // 使用侧板数据
      sensor1_index = 2 + ((sideboardSns & SWB_SET) >> 9);        // RC 发射器上的 SWB 用于更改控制类型
      if (sensor1_index == 2) {                                   // FOC 控制类型
        sensor1_index = (sideboardSns & SWC_SET) >> 11;           // RC 发射器上的 SWC 用于更改控制模式
      }
      sensor1_trig  = sensor1_index != sensor1_prev;              // 上升沿或下降沿变化检测
      if (inIdx != inIdx_prev) {                                  // 在输入索引变化时强制更新一次
        sensor1_trig  = 1;
      }
      sensor1_prev  = sensor1_index;
    } else {                                                      // 使用光电开关
      sensor1_trig  = (sensors & SENSOR1_SET) && !sensor1_prev;   // 上升沿检测
      sensor1_prev  =  sensors & SENSOR1_SET;
    }

    // 控制模式和控制类型处理
    if (sensor1_trig) {
      switch (sensor1_index) {
        case 0:     // FOC 电压模式
          rtP_Left.z_ctrlTypSel = rtP_Right.z_ctrlTypSel = FOC_CTRL;
          ctrlModReqRaw         = VLT_MODE;
          break;
        case 1:     // FOC 速度模式
          rtP_Left.z_ctrlTypSel = rtP_Right.z_ctrlTypSel = FOC_CTRL;
          ctrlModReqRaw         = SPD_MODE;
          break;
        case 2:     // FOC 转矩模式
          rtP_Left.z_ctrlTypSel = rtP_Right.z_ctrlTypSel = FOC_CTRL;
          ctrlModReqRaw         = TRQ_MODE;
          break;
        case 3:     // 正弦波模式
          rtP_Left.z_ctrlTypSel = rtP_Right.z_ctrlTypSel = SIN_CTRL;
          break;
        case 4:     // 换向模式
          rtP_Left.z_ctrlTypSel = rtP_Right.z_ctrlTypSel = COM_CTRL;
          break;
      }
      if (inIdx == inIdx_prev) { beepShortMany(sensor1_index + 1, 1); }
      if (++sensor1_index > 4) { sensor1_index = 0; }
    }

                                                             // 弱磁激活/停用
      static uint8_t  sensor2_index = 1;                          // 当用作按钮时，保存 sensor2 的按下索引号

      // 如果侧板控制处于活动状态，则覆盖
      if (inIdx == sideboardIdx) {                                // 使用侧板数据
        sensor2_index = (sideboardSns & SWD_SET) >> 13;           // RC 发射器上的 SWD 用于激活/停用弱磁
        sensor2_trig  = sensor2_index != sensor2_prev;            // 上升沿或下降沿变化检测
        if (inIdx != inIdx_prev) {                                // 在输入索引变化时强制更新一次
          sensor2_trig  = 1;
        }
        sensor2_prev  = sensor2_index;
      }else{
        sensor2_trig  = (sensors & SENSOR2_SET) && !sensor2_prev;   // 上升沿检测
        sensor2_prev  =  sensors & SENSOR2_SET;
      }

      #ifdef CRUISE_CONTROL_SUPPORT                                 // 巡航控制激活/停用
        if (sensor2_trig) {
          cruiseControl(sensor2_trig);
        }
      #else
        if (sensor2_trig) {
          switch (sensor2_index) {
            case 0:     // 弱磁禁用
              rtP_Left.b_fieldWeakEna  = 0; 
              rtP_Right.b_fieldWeakEna = 0;
              Input_Lim_Init();
              break;
            case 1:     // 弱磁启用
              rtP_Left.b_fieldWeakEna  = 1; 
              rtP_Right.b_fieldWeakEna = 1;
              Input_Lim_Init();
              break; 
          }
          if (inIdx == inIdx_prev) { beepShortMany(sensor2_index + 1, 1); }
          if (++sensor2_index > 1) { sensor2_index = 0; }
        }
      #endif  // CRUISE_CONTROL_SUPPORT
  #endif
}



/* =========================== 关机函数 =========================== */

 /*
 * 将配置保存到 Flash
 * 该函数确保断电后数据不会丢失
 */
void saveConfig() {
  #ifdef VARIANT_TRANSPOTTER
    if (saveValue_valid) {
      HAL_FLASH_Unlock();
      EE_WriteVariable(VirtAddVarTab[0], saveValue);
      HAL_FLASH_Lock();
    }
  #endif
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    if (inp_cal_valid || cur_spd_valid) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Saving configuration to EEprom\r\n");
      #endif

      HAL_FLASH_Unlock();
      EE_WriteVariable(VirtAddVarTab[0] , (uint16_t)FLASH_WRITE_KEY);
      EE_WriteVariable(VirtAddVarTab[1] , (uint16_t)rtP_Left.i_max);
      EE_WriteVariable(VirtAddVarTab[2] , (uint16_t)rtP_Left.n_max);
      for (uint8_t i=0; i<INPUTS_NR; i++) {
        EE_WriteVariable(VirtAddVarTab[ 3+8*i] , (uint16_t)input1[i].typ);
        EE_WriteVariable(VirtAddVarTab[ 4+8*i] , (uint16_t)input1[i].min);
        EE_WriteVariable(VirtAddVarTab[ 5+8*i] , (uint16_t)input1[i].mid);
        EE_WriteVariable(VirtAddVarTab[ 6+8*i] , (uint16_t)input1[i].max);
        EE_WriteVariable(VirtAddVarTab[ 7+8*i] , (uint16_t)input2[i].typ);
        EE_WriteVariable(VirtAddVarTab[ 8+8*i] , (uint16_t)input2[i].min);
        EE_WriteVariable(VirtAddVarTab[ 9+8*i] , (uint16_t)input2[i].mid);
        EE_WriteVariable(VirtAddVarTab[10+8*i] , (uint16_t)input2[i].max);
      }
      HAL_FLASH_Lock();
    }
  #endif 
}


void poweroff(void) {
  enable = 0;
  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  printf("-- Motors disabled --\r\n");
  #endif
  buzzerCount = 0; 
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  saveConfig();
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while(1) {}
}


void poweroffPressCheck(void) {
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      uint16_t cnt_press = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        HAL_Delay(10);
        if (cnt_press++ == 5 * 100) { beepShort(5); }
      }

      if (cnt_press > 8) enable = 0;

      if (cnt_press >= 5 * 100) {                        
        HAL_Delay(1000);
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  
          while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
          #ifdef VARIANT_BBCAR
          if (ABS((int)speedRL) < 5) {
            beepLong(16);
            HAL_Delay(200);
            beepLong(16);
            enable = 0;
            printf("# motors disabled\r\n");
            rtP_Left.z_ctrlTypSel = FOC_CTRL;
            rtP_Right.z_ctrlTypSel = FOC_CTRL;
            printf("# switched to FOC_CTRL\r\n");
            enable = 1;
            printf("# motors enabled\r\n");
            speedRL = 0.0;
          }
          #else
            beepLong(8);
            updateCurSpdLim();
            beepShort(5);
          #endif
        } else {                                          
          #ifdef AUTO_CALIBRATION_ENA
          beepLong(16); 
          adcCalibLim();
          beepShort(5);
          speedRL = 0.0;
          #endif
        }
      } else if (cnt_press > 8) {                         
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
          printf("Powering off, button has been pressed\r\n");
        #endif
      poweroff();
      }
    }
  #elif defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
      beepShort(5);
      HAL_Delay(300);
      if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
        beepLong(5);
        HAL_Delay(350);
        poweroff();
      } else {
        setDistance += 0.25;
        if (setDistance > 2.6) {
          setDistance = 0.5;
        }
        beepShort(setDistance / 0.25);
        saveValue = setDistance * 1000;
        saveValue_valid = 1;
      }
    }
  #else
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;                                             // 禁用电机
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // 等待按钮释放
      poweroff();                                             // 释放电源锁存
    }
  #endif
}



/* =========================== 滤波函数 =========================== */

  /* 低通滤波器定点 32 位：fixdt(1,32,16)
  * 最大值：32767.99998474121
  * 最小值：-32768
  * 分辨率：1.52587890625e-05
  * 
  * 输入：u = int16 或 int32
  * 输出：y = fixdt(1,32,16)
  * 参数：coef = fixdt(0,16,16) = [0,65535U]
  * 
  * 示例：
  * 如果 coef = 0.8（浮点数），则 coef = 0.8 * 2^16 = 52429（定点数）
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // 整数输出是定点输出右移 16 位后的结果
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // 溢出保护：2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}
  // 旧滤波器
  // 输入：u = int16
  // 输出：y = fixdt(1,32,20)
  // 参数：coef = fixdt(0,16,16) = [0,65535U]
  // yint = (int16_t)(y >> 20); // 整数输出是定点输出右移 20 位后的结果
  // void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
  //   int32_t tmp;  
  //   tmp = (int16_t)(u << 4) - (*y >> 16);  
  //   tmp = CLAMP(tmp, -32768, 32767);  // 溢出保护
  //   *y  = coef * tmp + (*y);
  // }


  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * 输入：u = int16
  * 输出：y = fixdt(1,16,4)
  * 参数：rate = fixdt(1,16,4) = [0, 32767] 不要将 rate 设为负值 (>32767)
  */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) {
    q0 = rate;
  } else {
    q1 = -rate;
    if (q0 < q1) {
      q0 = q1;
    }
  }

  *y = q0 + *y;
}


  /* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL); 
  * 输入：rtu_speed, rtu_steer = fixdt(1,16,4)
  * 输出：rty_speedR, rty_speedL = int16_t
  * 参数：SPEED_COEFFICIENT, STEER_COEFFICIENT = fixdt(0,16,14)
  */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL) {
    int16_t prodSpeed;
    int16_t prodSteer;
    int32_t tmp;

    prodSpeed   = (int16_t)((rtu_speed * (int16_t)SPEED_COEFFICIENT) >> 14);
    prodSteer   = (int16_t)((rtu_steer * (int16_t)STEER_COEFFICIENT) >> 14);

    tmp         = prodSpeed - prodSteer;  
    tmp         = CLAMP(tmp, -32768, 32767);  // 溢出保护
    *rty_speedR = (int16_t)(tmp >> 4);        // 从定点数转换为整数
    *rty_speedR = CLAMP(*rty_speedR, INPUT_MIN, INPUT_MAX);

    tmp         = prodSpeed + prodSteer;
    tmp         = CLAMP(tmp, -32768, 32767);  // 溢出保护
    *rty_speedL = (int16_t)(tmp >> 4);        // 从定点数转换为整数
    *rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}



/* =========================== 多次点击检测函数 =========================== */

  /* multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
  * 该函数检测多次点击按下，例如双击、三击等。
  * 输入：u = int16_t（输入信号）；timeNow = uint32_t（当前时间）
  * 输出：x->b_multipleTap（在此获取输出）
  */
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x) {
  uint8_t 	b_timeout;
  uint8_t 	b_hyst;
  uint8_t 	b_pulse;
  uint8_t 	z_pulseCnt;
  uint8_t   z_pulseCntRst;
  uint32_t 	t_time; 

  // 检测迟滞
  if (x->b_hysteresis) {
    b_hyst = (u > MULTIPLE_TAP_LO);
  } else {
    b_hyst = (u > MULTIPLE_TAP_HI);
  }

  // 检测脉冲
  b_pulse = (b_hyst != x->b_hysteresis);

  // 记录首次检测到脉冲的时间
  if (b_hyst && b_pulse && (x->z_pulseCntPrev == 0)) {
    t_time = timeNow;
  } else {
    t_time = x->t_timePrev;
  }

  // 创建超时布尔量
  b_timeout = (timeNow - t_time > MULTIPLE_TAP_TIMEOUT);

  // 创建脉冲计数器
  if ((!b_hyst) && (x->z_pulseCntPrev == 0)) {
    z_pulseCnt = 0U;
  } else {
    z_pulseCnt = b_pulse;
  }

  // 如果检测到完整的点击按下或发生超时，则重置计数器
  if ((x->z_pulseCntPrev >= MULTIPLE_TAP_NR) || b_timeout) {
    z_pulseCntRst = 0U;
  } else {
    z_pulseCntRst = x->z_pulseCntPrev;
  }
  z_pulseCnt = z_pulseCnt + z_pulseCntRst;

  // 检查是否检测到完整的点击按下且未超时
  if ((z_pulseCnt >= MULTIPLE_TAP_NR) && (!b_timeout)) {
    x->b_multipleTap = !x->b_multipleTap;	// 切换输出
  }

  // 更新状态
  x->z_pulseCntPrev = z_pulseCnt;
  x->b_hysteresis 	= b_hyst;
  x->t_timePrev 	  = t_time;
}


