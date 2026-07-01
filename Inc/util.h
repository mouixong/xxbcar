/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


// Rx Structures USART
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
  #ifdef CONTROL_IBUS
    typedef struct{
      uint8_t  start;
      uint8_t  type; 
      uint8_t  channels[IBUS_NUM_CHANNELS*2];
      uint8_t  checksuml;
      uint8_t  checksumh;
    } SerialCommand;
  #else
    typedef struct{
      uint16_t  start;
      int16_t   steer;
      int16_t   speed;
      uint16_t  buttons;    // APP扩展控制位
      uint16_t  checksum;
    } SerialCommand;
  #endif
#endif

// SerialCommand buttons位定义 (APP扩展控制)
#define BTN_EPB         (1 << 0)   // 电子手刹
#define BTN_CRUISE      (1 << 1)   // 定速巡航
#define BTN_HEADLIGHT   (1 << 2)   // 大灯
#define BTN_REVERSE     (1 << 3)   // 倒挡
#define BTN_MODE1       (1 << 4)   // 模式1 (学步)
#define BTN_MODE2       (1 << 5)   // 模式2 (标准)
#define BTN_MODE3       (1 << 6)   // 模式3 (乐趣)
#define BTN_MODE4       (1 << 7)   // 模式4 (动力)
#define BTN_BEEP        (1 << 8)   // 喇叭/蜂鸣器

// APP扩展命令全局状态 (由usart_process_command更新)
extern volatile uint16_t serial_buttons;
extern volatile uint8_t  serial_buttons_updated;
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
    typedef struct{
      uint16_t  start;
      int16_t   pitch;      // Angle
      int16_t   dPitch;     // Angle derivative
      int16_t   cmd1;       // RC Channel 1
      int16_t   cmd2;       // RC Channel 2
      uint16_t  sensors;    // RC Switches and Optical sideboard sensors
      uint16_t  checksum;
    } SerialSideboard;
#endif

// Input Structure
typedef struct {
  int16_t   raw;    // raw input
  int16_t   cmd;    // command
  uint8_t   typ;    // type
  uint8_t   typDef; // type Defined
  int16_t   min;    // minimum
  int16_t   mid;    // middle
  int16_t   max;    // maximum
  int16_t   dband;  // deadband
} InputStruct;

// Initialization Functions
void BLDC_Init(void);
void Input_Lim_Init(void);
void Input_Init(void);
void UART_DisableRxErrors(UART_HandleTypeDef *huart);

// General Functions
void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepLong(uint8_t freq);
void beepShort(uint8_t freq);
void beepShortMany(uint8_t cnt, int8_t dir);
void calcAvgSpeed(void);
void adcCalibLim(void);
void updateCurSpdLim(void);
void standstillHold(void);
void electricBrake(uint16_t speedBlend, uint8_t reverseDir);
void cruiseControl(uint8_t button);
int  checkInputType(int16_t min, int16_t mid, int16_t max);

// Input Functions
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max);
void readInputRaw(void);
void handleTimeout(void);
void readCommand(void);
void usart2_rx_check(void);
void usart3_rx_check(void);
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
void usart_process_debug(uint8_t *userCommand, uint32_t len);
#endif
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx);
#endif
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
void usart_process_sideboard(SerialSideboard *Sideboard_in, SerialSideboard *Sideboard_out, uint8_t usart_idx);
#endif

// Sideboard functions
void sideboardLeds(uint8_t *leds);
void sideboardSensors(uint8_t sensors);

#ifdef VARIANT_BBCAR
  void adc_error_melody();
  int32_t isAroundMin(int32_t value, int32_t minn, int32_t maxx);
  int32_t isAroundMax(int32_t value, int32_t minn, int32_t maxx);
  void bbcarDetectDrivingMode();
  int16_t bbcarLoop();
  
  // DIY卡丁车固件 v17.03 功能
  #ifdef DUAL_CONTROL_ENABLE
    void dualControlInit(void);
    void dualControlProcess(int16_t adc_cmdL, int16_t adc_cmdR, int16_t *cmdL, int16_t *cmdR);
    void dualControlSetUartCommand(int16_t cmdL, int16_t cmdR);
    uint8_t dualControlUartConnected(void);
  #endif
  
  #ifdef CRUISE_CONTROL_ENABLE
    void cruiseControlInit(void);
    void cruiseControlEnable(uint8_t enable);
    uint8_t cruiseControlGetState(void);
    void cruiseControlSetSpeed(int16_t speed);
    int16_t cruiseControlProcess(int16_t cmd, int16_t speedAvg);
  #endif
  
  #ifdef SMART_NEUTRAL_ENABLE
    void smartNeutralInit(void);
    void smartNeutralProcess(void);
    uint8_t smartNeutralGetState(void);
  #endif
  
  #ifdef ELECTRONIC_PARKING_BRAKE_ENABLE
    void epbInit(void);
    void epbEnable(uint8_t enable);
    uint8_t epbGetState(void);
    int16_t epbProcess(int16_t cmd, int16_t speed);
  #endif
  
  #ifdef AUTO_HALL_CALIBRATION_ENABLE
    void autoHallCalibrationStart(void);
    void autoHallCalibrationProcess(void);
    uint8_t autoHallCalibrationGetState(void);
    int16_t autoHallCalibrationGetSpeed(void);
  #endif
  
  #ifdef BOOST_ENABLE
    int16_t boostProcess(int16_t cmd, int16_t current);
    uint8_t boostGetLevel(void);
  #endif
  
  #ifdef AUTO_DECEL_ENABLE
    int16_t autoDecelProcess(int16_t cmd);
  #endif
  
  // 一线通仪表功能
  #ifdef YXT_DISPLAY_ENABLE
    void yxtDisplayInit(void);
    void yxtDisplayUpdate(void);
    void yxtSetSpeed(int16_t speed);
    void yxtSetVoltage(int16_t voltage);
    void yxtSetCurrent(int16_t current);
    void yxtSetTemperature(int16_t temp);
    void yxtSetGear(uint8_t gear);
    void yxtSetFault(uint8_t fault);
    void yxtClearFault(uint8_t fault);
    void yxtSendPacket(void);
  #endif
  
  // 蜂鸣器提示音系统
  void bbcarBeep(uint8_t type);
#endif

// Poweroff Functions
void saveConfig(void);
void poweroff(void);
void poweroffPressCheck(void);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL);

// Multiple Tap Function
typedef struct {
  uint32_t  t_timePrev;
  uint8_t   z_pulseCntPrev;
  uint8_t   b_hysteresis;
  uint8_t   b_multipleTap;
} MultipleTap;
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x);

#endif

