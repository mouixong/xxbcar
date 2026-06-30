/*
 * 项目名称 : DIY卡丁车固件扩展功能模块
 * 版    本 : V17.03
 * 说    明 : 基于hoverboard-firmware-hack-FOC-bbcar扩展实现PDF《DIY卡丁车固件功能说明_v17.03》全部功能
 *              包含：蓝牙APP通信、自动相序校准、定速巡航、双控系统、智能空挡、电子手刹、
 *                    差速转向、一线通仪表、大灯控制、蜂鸣器提示音完善等
 *
 * 修改记录 :
 *     版本号  日期        作者     说明
 *     V17.03  2026-06-30  AI助手   整合所有扩展功能，全部中文注释
 */

#ifndef XXBCAR_FEATURES_H
#define XXBCAR_FEATURES_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ========================== 蓝牙串口通信协议定义 ========================== */

// 通信帧头，用于识别有效数据包
#define PROTOCOL_HEAD_1         0xAB
#define PROTOCOL_HEAD_2         0xCD

// 命令类型定义（APP下发到主板的命令）
#define CMD_SET_MODE            0x01    // 设置驾驶模式
#define CMD_SET_SPEED           0x02    // 设置目标速度（定速巡航或遥控）
#define CMD_SET_STEER           0x03    // 设置转向（遥控模式差速转向）
#define CMD_SET_BRAKE           0x04    // 设置刹车力度（自动减速优化）
#define CMD_SET_BOOST           0x05    // 设置增压强度（五档增压）
#define CMD_SET_HAND_BRAKE      0x06    // 控制电子手刹
#define CMD_SET_HEADLIGHT       0x07    // 控制大灯
#define CMD_REQ_CALIB           0x08    // 请求自动相序校准
#define CMD_REQ_STATUS          0x09    // 请求状态上报
#define CMD_SET_GEAR            0x0A    // 设置档位（1-4档）
#define CMD_SET_REVERSE         0x0B    // 设置倒挡开关

// 状态上报类型（主板上报到APP）
#define STATUS_UPLOAD           0x80    // 周期性状态上报
#define CALIB_START_ACK         0x81    // 校准开始确认
#define CALIB_DONE_ACK          0x82    // 校准完成确认
#define BEEP_EVENT              0x83    // 蜂鸣器事件上报
#define FAULT_EVENT             0x84    // 故障事件上报

// 蓝牙通信接收缓冲区大小
#define BLE_RX_BUF_SIZE         64
#define BLE_TX_BUF_SIZE         64

/* ========================== 驾驶模式枚举定义 ========================== */

// 四种驾驶模式，对应PDF说明
typedef enum {
    MODE_KARTING = 0,       // 卡丁车模式：油门前进、刹车后退、双控有效
    MODE_SEDAN,             // 轿车模式：摇杆左右为刹车、倒挡切换、双控有效
    MODE_REMOTE,            // 遥控模式：差速转向、低速高扭矩、倒挡切换方向
    MODE_SPIN_TURN          // 原地掉头模式：油门左掉头、刹车右掉头
} DriveMode_t;

/* ========================== 一线通仪表协议定义 ========================== */

// 一线通仪表通常使用单线UART通信，波特率1200或2400
// 数据格式：起始位 + 8位数据 + 停止位，无校验
#define YXT_BAUDRATE            1200
#define YXT_TX_BUF_SIZE         8

// 一线通仪表显示状态标志位
#define YXT_FLAG_NORMAL         0x00    // 正常显示
#define YXT_FLAG_THROTTLE_FAULT 0x01    // 转把故障灯（轿车模式）
#define YXT_FLAG_CTRL_FAULT     0x02    // 控制器故障灯（遥控模式）
#define YXT_FLAG_BRAKE_LIGHT    0x04    // 刹车灯（手刹或踩刹）
#define YXT_FLAG_MOTOR_FAULT    0x08    // 电机故障灯（霍尔故障）

/* ========================== 蜂鸣器提示音事件定义 ========================== */

typedef enum {
    BEEP_NONE = 0,
    BEEP_GEAR_UP,           // 加档：响1声
    BEEP_GEAR_DOWN,         // 减档：响1声
    BEEP_REVERSE,           // 倒挡：响1声
    BEEP_HAND_BRAKE_ON,     // 手刹开启：响1声
    BEEP_HAND_BRAKE_OFF,    // 手刹关闭：响1声
    BEEP_CRUISE_ON,         // 定速巡航开启：响1声
    BEEP_CRUISE_OFF,        // 定速巡航关闭：响1声
    BEEP_HEADLIGHT_ON,      // 大灯开启：响1声
    BEEP_HEADLIGHT_OFF,     // 大灯关闭：响1声
    BEEP_MODE_CHANGE,       // 切换驾驶模式：响2声
    BEEP_REMOTE_CONNECTED,  // 遥控器连接成功：响3声
    BEEP_CALIB_START,       // 开始自动校准：响4声
    BEEP_CALIB_DONE,        // 校准完成：响5声
    BEEP_HALL_FAULT         // 霍尔故障：滴……滴……滴……
} BeepEvent_t;

/* ========================== 外部变量声明 ========================== */

// 蓝牙/遥控器连接状态：1=已连接，0=未连接
extern uint8_t remoteConnected;

// 当前驾驶模式（四种模式之一）
extern DriveMode_t currentDriveMode;

// 电子手刹状态：1=开启，0=关闭
extern uint8_t handBrakeEnabled;

// 智能空挡状态：1=空挡中，0=正常驱动
extern uint8_t neutralGearActive;

// 定速巡航状态：1=开启，0=关闭
extern uint8_t cruiseControlEnabled;

// 大灯状态：1=开启，0=关闭
extern uint8_t headLightEnabled;

// 倒挡状态：1=倒挡，0=前进挡
extern uint8_t reverseGearEnabled;

// APP可调参数
extern int16_t appBrakeForce;       // 自动刹车力度，范围0-50，默认0
extern int16_t appMaxCurrent;       // 主板最大电流，默认25（数值越小极速越低）
extern int16_t appTargetSpeed;      // 定速巡航目标速度
extern int16_t appSteerValue;       // 遥控模式转向值

// 差速转向相关
extern int16_t leftMotorOffset;     // 左电机差速偏移
extern int16_t rightMotorOffset;    // 右电机差速偏移

// 自动相序校准状态
extern uint8_t autoCalibActive;     // 1=校准中
extern uint8_t autoCalibDone;       // 1=校准完成

/* ========================== 函数声明 ========================== */

// 初始化函数：在main.c中系统初始化后调用
void XXBCAR_Init(void);

// 主循环处理函数：在main.c的主循环中每周期调用
void XXBCAR_MainLoop(void);

/* ---------- 蓝牙串口通信 ---------- */
void BLE_ProcessRx(void);                       // 处理蓝牙接收数据
void BLE_SendStatus(void);                      // 周期性发送状态到APP
void BLE_ParsePacket(uint8_t *data, uint8_t len); // 解析APP下发的命令包
void BLE_SendPacket(uint8_t type, uint8_t *data, uint8_t len); // 发送数据包到APP

/* ---------- 自动相序校准 ---------- */
void AutoCalib_Start(void);                     // 启动自动相序校准
void AutoCalib_Process(void);                   // 校准过程处理（主循环中调用）
void AutoCalib_Stop(void);                      // 停止校准

/* ---------- 定速巡航 ---------- */
void CruiseControl_Enable(void);                // 开启定速巡航
void CruiseControl_Disable(void);               // 关闭定速巡航
void CruiseControl_Process(void);               // 定速巡航过程控制

/* ---------- 电子手刹 ---------- */
void HandBrake_Enable(void);                    // 开启电子手刹
void HandBrake_Disable(void);                   // 关闭电子手刹
void HandBrake_Process(void);                   // 手刹状态处理

/* ---------- 智能空挡 ---------- */
void NeutralGear_Process(void);                 // 智能空挡检测与处理

/* ---------- 差速转向 ---------- */
void DiffSteering_Process(void);                // 差速转向计算（遥控模式）

/* ---------- 倒挡切换 ---------- */
void ReverseGear_Process(void);                 // 倒挡逻辑处理

/* ---------- 一线通仪表 ---------- */
void YXT_Init(void);                            // 初始化一线通仪表串口
void YXT_SendData(void);                        // 发送数据到一线通仪表
void YXT_SetDisplayFlags(uint8_t flags);        // 设置仪表显示标志

/* ---------- 大灯控制 ---------- */
void HeadLight_Init(void);                      // 初始化大灯GPIO
void HeadLight_On(void);                        // 开大灯
void HeadLight_Off(void);                       // 关大灯
void HeadLight_Toggle(void);                    // 切换大灯状态

/* ---------- 蜂鸣器提示音完善 ---------- */
void BeepEvent_Trigger(BeepEvent_t event);      // 触发蜂鸣器事件
void BeepEvent_Process(void);                   // 蜂鸣器事件处理（主循环中调用）

/* ---------- APP参数调节 ---------- */
void APP_SetBrakeForce(int16_t force);          // 设置自动刹车力度
void APP_SetMaxCurrent(int16_t current);        // 设置最大电流
void APP_SetTargetSpeed(int16_t speed);         // 设置目标速度

#endif // XXBCAR_FEATURES_H
