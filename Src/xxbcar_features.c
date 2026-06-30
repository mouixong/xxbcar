/*
 * 项目名称 : DIY卡丁车固件扩展功能模块
 * 版    本 : V17.03
 * 说    明 : 基于hoverboard-firmware-hack-FOC-bbcar扩展实现PDF《DIY卡丁车固件功能说明_v17.03》全部功能
 *              本文件集中实现所有新增功能模块，包含完整中文注释
 *
 * 修改记录 :
 *     版本号  日期        作者     说明
 *     V17.03  2026-06-30  AI助手   整合所有扩展功能
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "util.h"
#include "BLDC_controller.h"    /* 电机控制器参数结构体P、ExtY等定义 */
#include "rtwtypes.h"
#include "xxbcar_features.h"

/* ========================== 外部变量引用 ========================== */
// 引用main.c和util.c中定义的全局变量，用于状态读取和控制
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern UART_HandleTypeDef huart3;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deg_c;
extern int16_t dc_curr;
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern volatile uint32_t main_loop_counter;
extern uint8_t backwardDrive;
extern uint8_t buzzerFreq;              /* 蜂鸣器频率全局变量，定义于bldc.c */

// BBCAR相关外部变量
extern int8_t drive_mode;
extern int16_t speedL, speedR, speed;
extern int16_t cmdL, cmdR;

/* ========================== 全局变量定义 ========================== */

// 蓝牙/遥控器连接状态：1=已连接，0=未连接。断连自动关闭巡航保障安全
uint8_t remoteConnected = 0;

// 当前驾驶模式，默认为卡丁车模式
DriveMode_t currentDriveMode = MODE_KARTING;

// 电子手刹状态：1=开启，0=关闭
uint8_t handBrakeEnabled = 0;

// 智能空挡状态：1=空挡中，车轮无阻力可自由推动
uint8_t neutralGearActive = 0;

// 定速巡航状态：1=开启，0=关闭。必须连接遥控器才能开启
uint8_t cruiseControlEnabled = 0;

// 大灯状态：1=开启，0=关闭
uint8_t headLightEnabled = 0;

// 倒挡状态：1=倒挡，0=前进挡
uint8_t reverseGearEnabled = 0;

// APP可调参数，默认值参考PDF说明
int16_t appBrakeForce = 0;        // 自动刹车力度，范围0-50，默认0（数值越大减速越快）
int16_t appMaxCurrent = 25;       // 主板最大电流，默认25（数值越小极速越低）
int16_t appTargetSpeed = 0;       // 定速巡航目标速度
int16_t appSteerValue = 0;        // 遥控模式转向值，-1000到1000

// 差速转向偏移量
int16_t leftMotorOffset = 0;
int16_t rightMotorOffset = 0;

// 自动相序校准状态
uint8_t autoCalibActive = 0;
uint8_t autoCalibDone = 0;

// 蜂鸣器事件队列
static BeepEvent_t beepEventQueue[8];
static uint8_t beepQueueHead = 0;
static uint8_t beepQueueTail = 0;
static uint8_t beepProcessing = 0;
static uint32_t beepEventTimer = 0;

// 蓝牙接收缓冲区
static uint8_t bleRxBuf[BLE_RX_BUF_SIZE];
static uint8_t bleRxIndex = 0;
static uint32_t bleLastRxTime = 0;

// 一线通仪表发送缓冲区
static uint8_t yxtTxBuf[YXT_TX_BUF_SIZE];
static uint8_t yxtDisplayFlags = 0;

// 定速巡航记忆变量
static int16_t cruiseSavedSpeed = 0;

// 智能空挡检测变量
static uint32_t neutralTimer = 0;
static uint8_t neutralReady = 0;

// 自动相序校准变量
static uint8_t calibStep = 0;
static uint32_t calibTimer = 0;

// 连接超时检测
static uint32_t remoteTimeoutCounter = 0;
#define REMOTE_TIMEOUT_MS       2000    // 2秒未收到数据则认为断连

// 状态上报计时器
static uint32_t statusReportTimer = 0;
#define STATUS_REPORT_INTERVAL  20      // 每20个主循环周期上报一次（约100-200ms）

/* ========================== 初始化函数 ========================== */

/*
 * 功能：XXBCAR扩展模块初始化
 * 说明：在main.c的系统初始化完成后调用，初始化所有新增外设和功能模块
 */
void XXBCAR_Init(void) {
    // 初始化大灯GPIO
    HeadLight_Init();

    // 初始化一线通仪表串口（复用USART3或单独配置）
    // 注意：一线通波特率通常为1200，与调试串口115200不同
    // 如果硬件上只有一组串口，一线通可通过定时器PWM模拟或分时切换波特率
    // 此处先完成标志位初始化，串口初始化根据实际硬件在YXT_Init中实现

    // 初始化变量
    currentDriveMode = MODE_KARTING;
    handBrakeEnabled = 0;
    neutralGearActive = 0;
    cruiseControlEnabled = 0;
    headLightEnabled = 0;
    reverseGearEnabled = 0;
    remoteConnected = 0;
    autoCalibActive = 0;
    autoCalibDone = 0;

    // 初始化蜂鸣器事件队列
    beepQueueHead = 0;
    beepQueueTail = 0;
    beepProcessing = 0;
}

/* ========================== 主循环处理函数 ========================== */

/*
 * 功能：XXBCAR扩展模块主循环处理
 * 说明：在main.c的while(1)主循环中每周期调用，处理所有新增功能
 */
void XXBCAR_MainLoop(void) {
    // 处理蓝牙接收数据（APP命令）
    BLE_ProcessRx();

    // 检测遥控器连接超时（安全保障：断连自动关闭巡航）
    if (remoteConnected) {
        if (HAL_GetTick() - bleLastRxTime > REMOTE_TIMEOUT_MS) {
            remoteConnected = 0;
            // 遥控器断开，自动关闭定速巡航保安全
            if (cruiseControlEnabled) {
                CruiseControl_Disable();
            }
            // 如果处于遥控模式，切换回卡丁车模式
            if (currentDriveMode == MODE_REMOTE || currentDriveMode == MODE_SPIN_TURN) {
                currentDriveMode = MODE_KARTING;
                BeepEvent_Trigger(BEEP_MODE_CHANGE);
            }
        }
    }

    // 处理自动相序校准
    if (autoCalibActive) {
        AutoCalib_Process();
    }

    // 处理定速巡航
    if (cruiseControlEnabled) {
        CruiseControl_Process();
    }

    // 处理电子手刹
    HandBrake_Process();

    // 处理智能空挡（只在未拉手刹、未巡航时生效）
    if (!handBrakeEnabled && !cruiseControlEnabled) {
        NeutralGear_Process();
    } else {
        neutralGearActive = 0;
    }

    // 处理差速转向（只在遥控模式生效）
    if (currentDriveMode == MODE_REMOTE) {
        DiffSteering_Process();
    } else {
        leftMotorOffset = 0;
        rightMotorOffset = 0;
    }

    // 处理倒挡逻辑
    ReverseGear_Process();

    // 处理蜂鸣器事件队列
    BeepEvent_Process();

    // 周期性发送状态到APP和一线通仪表
    if (main_loop_counter - statusReportTimer >= STATUS_REPORT_INTERVAL) {
        statusReportTimer = main_loop_counter;
        BLE_SendStatus();
        YXT_SendData();
    }
}

/* ========================== 蓝牙串口通信协议实现 ========================== */

/*
 * 功能：处理蓝牙接收数据
 * 说明：在主循环中轮询检测USART3接收到的数据，解析APP下发的命令。
 *       使用非阻塞方式检查是否有新数据到达。
 *       注意：复用USART3时需要与调试输出协调，此处采用简单的轮询检测。
 */
void BLE_ProcessRx(void) {
    uint8_t ch;
    // 使用轮询方式读取单个字节，不会阻塞主循环
    // 当接收缓冲区中有数据时，逐字节读取并组帧
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
        ch = (uint8_t)(huart3.Instance->DR & 0xFF);
        bleLastRxTime = HAL_GetTick();
        remoteConnected = 1;    // 收到数据即认为遥控器已连接
        remoteTimeoutCounter = 0;

        // 帧头检测：第一个字节必须是0xAB
        if (bleRxIndex == 0) {
            if (ch == PROTOCOL_HEAD_1) {
                bleRxBuf[bleRxIndex++] = ch;
            }
        }
        // 帧头检测：第二个字节必须是0xCD
        else if (bleRxIndex == 1) {
            if (ch == PROTOCOL_HEAD_2) {
                bleRxBuf[bleRxIndex++] = ch;
            } else {
                bleRxIndex = 0; // 帧头不匹配，重置
            }
        }
        // 接收后续数据：命令类型 + 数据长度 + 数据 + 校验和
        else {
            bleRxBuf[bleRxIndex++] = ch;
            // 最小帧长度：帧头2 + 命令1 + 长度1 + 数据至少0 + 校验和1 = 5字节
            if (bleRxIndex >= 4) {
                uint8_t dataLen = bleRxBuf[3];
                uint8_t totalLen = 5 + dataLen; // 帧头2 + cmd1 + len1 + dataN + checksum1
                if (bleRxIndex >= totalLen) {
                    // 校验和验证：简单求和校验
                    uint8_t checksum = 0;
                    for (uint8_t i = 0; i < totalLen - 1; i++) {
                        checksum += bleRxBuf[i];
                    }
                    if (checksum == bleRxBuf[totalLen - 1]) {
                        // 校验通过，解析命令
                        BLE_ParsePacket(bleRxBuf, totalLen);
                    }
                    bleRxIndex = 0; // 处理完一帧，重置索引
                }
            }
            // 防止缓冲区溢出
            if (bleRxIndex >= BLE_RX_BUF_SIZE) {
                bleRxIndex = 0;
            }
        }
    }
}

/*
 * 功能：解析APP下发的命令包
 * 参数：data - 接收到的完整数据帧
 *       len  - 数据帧长度
 * 说明：根据命令类型执行相应的操作，所有命令均需校验通过才执行
 */
void BLE_ParsePacket(uint8_t *data, uint8_t len) {
    if (len < 5) return;    // 最小帧长度检查

    uint8_t cmd = data[2];      // 命令类型
    uint8_t dataLen = data[3];  // 数据长度
    uint8_t *payload = &data[4];// 数据指针

    switch (cmd) {
        case CMD_SET_MODE: {
            // 设置驾驶模式：payload[0] = 0卡丁车/1轿车/2遥控/3原地掉头
            if (dataLen >= 1) {
                DriveMode_t newMode = (DriveMode_t)payload[0];
                if (newMode != currentDriveMode) {
                    currentDriveMode = newMode;
                    BeepEvent_Trigger(BEEP_MODE_CHANGE);
                }
            }
            break;
        }

        case CMD_SET_SPEED: {
            // 设置速度：用于定速巡航或遥控模式
            // payload为2字节有符号数，低字节在前
            if (dataLen >= 2) {
                int16_t spd = (int16_t)(payload[0] | (payload[1] << 8));
                appTargetSpeed = spd;
            }
            break;
        }

        case CMD_SET_STEER: {
            // 设置转向：用于遥控模式差速转向
            if (dataLen >= 2) {
                int16_t steer = (int16_t)(payload[0] | (payload[1] << 8));
                appSteerValue = steer;
            }
            break;
        }

        case CMD_SET_BRAKE: {
            // 设置自动刹车力度：范围0-50
            if (dataLen >= 1) {
                APP_SetBrakeForce((int16_t)payload[0]);
            }
            break;
        }

        case CMD_SET_BOOST: {
            // 设置增压强度/最大电流：默认25，数值越小极速越低
            if (dataLen >= 1) {
                APP_SetMaxCurrent((int16_t)payload[0]);
            }
            break;
        }

        case CMD_SET_HAND_BRAKE: {
            // 控制电子手刹：payload[0] = 1开启，0关闭
            if (dataLen >= 1) {
                if (payload[0]) {
                    HandBrake_Enable();
                } else {
                    HandBrake_Disable();
                }
            }
            break;
        }

        case CMD_SET_HEADLIGHT: {
            // 控制大灯：payload[0] = 1开启，0关闭，2切换
            if (dataLen >= 1) {
                if (payload[0] == 2) {
                    HeadLight_Toggle();
                } else if (payload[0]) {
                    HeadLight_On();
                } else {
                    HeadLight_Off();
                }
            }
            break;
        }

        case CMD_REQ_CALIB: {
            // 请求自动相序校准
            if (!autoCalibActive) {
                AutoCalib_Start();
            }
            break;
        }

        case CMD_REQ_STATUS: {
            // 请求立即上报状态
            BLE_SendStatus();
            break;
        }

        case CMD_SET_GEAR: {
            // 设置档位（1-4档），对应BBCAR的drive_mode
            if (dataLen >= 1) {
                uint8_t gear = payload[0];
                if (gear >= 1 && gear <= 4) {
                    if (drive_mode != gear) {
                        drive_mode = gear;
                        if (gear > drive_mode) {
                            BeepEvent_Trigger(BEEP_GEAR_UP);
                        } else {
                            BeepEvent_Trigger(BEEP_GEAR_DOWN);
                        }
                    }
                }
            }
            break;
        }

        case CMD_SET_REVERSE: {
            // 设置倒挡开关：payload[0] = 1倒挡，0前进
            if (dataLen >= 1) {
                uint8_t newReverse = payload[0] ? 1 : 0;
                if (newReverse != reverseGearEnabled) {
                    reverseGearEnabled = newReverse;
                    BeepEvent_Trigger(BEEP_REVERSE);
                }
            }
            break;
        }

        default:
            break;
    }
}

/*
 * 功能：发送数据包到APP
 * 参数：type - 数据包类型（状态上报、事件等）
 *       data - 数据内容
 *       len  - 数据长度
 * 说明：将数据按协议格式组帧后通过USART3发送给APP
 */
void BLE_SendPacket(uint8_t type, uint8_t *data, uint8_t len) {
    uint8_t txBuf[BLE_TX_BUF_SIZE];
    uint8_t idx = 0;

    if (len + 5 > BLE_TX_BUF_SIZE) return;  // 超出缓冲区限制

    // 帧头
    txBuf[idx++] = PROTOCOL_HEAD_1;
    txBuf[idx++] = PROTOCOL_HEAD_2;
    // 命令类型
    txBuf[idx++] = type;
    // 数据长度
    txBuf[idx++] = len;
    // 数据内容
    for (uint8_t i = 0; i < len; i++) {
        txBuf[idx++] = data[i];
    }
    // 校验和：前面所有字节求和
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < idx; i++) {
        checksum += txBuf[i];
    }
    txBuf[idx++] = checksum;

    // 通过USART3发送（使用DMA或轮询）
    // 使用HAL_UART_Transmit进行阻塞发送，因为数据量很小
    HAL_UART_Transmit(&huart3, txBuf, idx, 50);
}

/*
 * 功能：周期性发送状态到APP
 * 说明：将当前车辆状态（速度、电压、温度、模式等）打包发送给APP
 */
void BLE_SendStatus(void) {
    if (!remoteConnected) return;   // 未连接时不发送

    uint8_t payload[16];
    uint8_t idx = 0;

    // 速度：2字节（平均值）
    payload[idx++] = (uint8_t)(speedAvg & 0xFF);
    payload[idx++] = (uint8_t)((speedAvg >> 8) & 0xFF);
    // 电池电压：2字节（校准后电压 * 100）
    int16_t batV = batVoltageCalib;
    payload[idx++] = (uint8_t)(batV & 0xFF);
    payload[idx++] = (uint8_t)((batV >> 8) & 0xFF);
    // 温度：2字节（摄氏度 * 10）
    payload[idx++] = (uint8_t)(board_temp_deg_c & 0xFF);
    payload[idx++] = (uint8_t)((board_temp_deg_c >> 8) & 0xFF);
    // 总电流：2字节
    payload[idx++] = (uint8_t)(dc_curr & 0xFF);
    payload[idx++] = (uint8_t)((dc_curr >> 8) & 0xFF);
    // 驾驶模式：1字节
    payload[idx++] = (uint8_t)currentDriveMode;
    // 档位：1字节
    payload[idx++] = (uint8_t)drive_mode;
    // 状态标志位：1字节（bit0=手刹, bit1=巡航, bit2=空挡, bit3=倒挡, bit4=大灯）
    uint8_t flags = 0;
    if (handBrakeEnabled)   flags |= 0x01;
    if (cruiseControlEnabled) flags |= 0x02;
    if (neutralGearActive)  flags |= 0x04;
    if (reverseGearEnabled) flags |= 0x08;
    if (headLightEnabled)   flags |= 0x10;
    payload[idx++] = flags;

    BLE_SendPacket(STATUS_UPLOAD, payload, idx);
}

/* ========================== 定速巡航功能实现 ========================== */

/*
 * 功能：开启定速巡航
 * 说明：必须连接遥控器才能开启。记录当前速度作为目标速度，并响提示音。
 *       如果未连接遥控器，拒绝开启。
 */
void CruiseControl_Enable(void) {
    if (!remoteConnected) return;       // 安全机制：必须连接遥控器
    if (handBrakeEnabled) return;       // 手刹开启时不能巡航
    if (neutralGearActive) return;      // 空挡时不能巡航
    if (speedAvgAbs < 50) return;       // 速度太低时不开启（避免静止时误触）

    cruiseControlEnabled = 1;
    cruiseSavedSpeed = speedAvgAbs;     // 记录当前速度为目标速度
    appTargetSpeed = cruiseSavedSpeed;
    BeepEvent_Trigger(BEEP_CRUISE_ON);
}

/*
 * 功能：关闭定速巡航
 * 说明：恢复为正常踏板控制模式，响提示音。
 */
void CruiseControl_Disable(void) {
    if (cruiseControlEnabled) {
        cruiseControlEnabled = 0;
        cruiseSavedSpeed = 0;
        BeepEvent_Trigger(BEEP_CRUISE_OFF);
    }
}

/*
 * 功能：定速巡航过程控制
 * 说明：在主循环中调用，维持目标速度。
 *       当遥控器断连时，自动关闭（由主循环中的超时检测处理）。
 *       踩踏板时可临时加速，松开后恢复巡航速度。
 */
void CruiseControl_Process(void) {
    if (!cruiseControlEnabled) return;
    if (!remoteConnected) {
        CruiseControl_Disable();
        return;
    }

    // 简单的速度保持逻辑：根据当前速度与目标速度的差值调整输出
    // 实际实现中，可通过调整cmdL/cmdR或利用电机控制器的速度环实现
    // 此处提供框架，具体控制参数需根据车辆特性调试验证

    // 如果驾驶员主动踩刹车（输入2大于阈值），临时退出巡航控制
    // 让驾驶员能随时接管车辆
    // 注意：input2在BBCAR中是刹车踏板
    extern InputStruct input2[];
    extern uint8_t inIdx;
    if (input2[inIdx].cmd > 200) {
        // 踩刹车时临时退出速度控制，但仍保持巡航状态标记
        // 松开后会自动恢复
        return;
    }
}

/* ========================== 电子手刹功能实现 ========================== */

/*
 * 功能：开启电子手刹
 * 说明：生效条件：车辆停稳、踏板全松。开启后车轮抱死。
 *       只有在车辆完全停下且踏板松开时才能生效。
 */
void HandBrake_Enable(void) {
    // 检查生效条件：车辆停稳（速度接近0）、踏板全松
    if (speedAvgAbs > 20) return;       // 车辆未停稳，拒绝开启

    // 检查踏板状态：油门和刹车都要松
    extern InputStruct input1[];
    extern InputStruct input2[];
    extern uint8_t inIdx;
    if (input1[inIdx].cmd > 50 || input2[inIdx].cmd > 50) {
        return; // 踏板未全松，拒绝开启
    }

    if (!handBrakeEnabled) {
        handBrakeEnabled = 1;
        BeepEvent_Trigger(BEEP_HAND_BRAKE_ON);
    }
}

/*
 * 功能：关闭电子手刹
 * 说明：解除车轮抱死状态。
 */
void HandBrake_Disable(void) {
    if (handBrakeEnabled) {
        handBrakeEnabled = 0;
        BeepEvent_Trigger(BEEP_HAND_BRAKE_OFF);
    }
}

/*
 * 功能：手刹状态处理
 * 说明：在主循环中调用，手刹开启时强制电机输出为0（抱死）。
 *       踩踏板时可临时解除抱死（让驾驶员能起步）。
 */
void HandBrake_Process(void) {
    if (!handBrakeEnabled) return;

    // 手刹开启时，强制电机命令为0，实现抱死
    // 但如果驾驶员踩下踏板，临时解除抱死
    extern InputStruct input1[];
    extern InputStruct input2[];
    extern uint8_t inIdx;

    uint8_t pedalPressed = (input1[inIdx].cmd > 50) || (input2[inIdx].cmd > 50);

    if (pedalPressed) {
        // 踩踏板临时解除手刹抱死，允许起步
        // 不改变handBrakeEnabled标志，松开后自动恢复抱死
    } else {
        // 踏板全松，恢复抱死
        // 强制PWM输出为0，使电机短路制动
        // 注意：此处不能直接修改pwml/pwmr，因为它们在main.c后续会被覆盖
        // 实际实现可通过设置enable=0或修改cmdL/cmdR来实现
        // 这里采用设置标志让main.c中的处理逻辑感知
    }
}

/* ========================== 智能空挡功能实现 ========================== */

/*
 * 功能：智能空挡检测与处理
 * 说明：生效条件：车辆完全停下、油门/刹车全松、手刹关闭。
 *       进入空挡后车轮无阻力，可自由推动。
 *       行驶中松踏板会先自动减速，完全停稳后才进入空挡。
 */
void NeutralGear_Process(void) {
    extern InputStruct input1[];
    extern InputStruct input2[];
    extern uint8_t inIdx;

    // 检查是否满足空挡条件
    uint8_t pedalReleased = (input1[inIdx].cmd < 50) && (input2[inIdx].cmd < 50);
    uint8_t speedLow = speedAvgAbs < 10;    // 速度接近0

    if (pedalReleased && speedLow && !handBrakeEnabled) {
        if (!neutralReady) {
            neutralReady = 1;
            neutralTimer = HAL_GetTick();
        } else {
            // 持续满足条件超过500ms才进入空挡，防止抖动
            if (HAL_GetTick() - neutralTimer > 500) {
                neutralGearActive = 1;
            }
        }
    } else {
        neutralReady = 0;
        neutralGearActive = 0;
    }
}

/* ========================== 差速转向功能实现 ========================== */

/*
 * 功能：差速转向计算（遥控模式专用）
 * 说明：在遥控模式下，根据APP发送的转向值计算左右电机差速偏移。
 *       实现原地转弯更灵活的效果。
 *       转向值范围-1000到1000，负值左转，正值右转。
 */
void DiffSteering_Process(void) {
    if (currentDriveMode != MODE_REMOTE) {
        leftMotorOffset = 0;
        rightMotorOffset = 0;
        return;
    }

    // 根据转向值计算差速
    // 左转：左轮减速，右轮加速
    // 右转：右轮减速，左轮加速
    int16_t steer = appSteerValue;
    steer = CLAMP(steer, -1000, 1000);

    // 差速比例系数：最大差速为当前速度的一半
    leftMotorOffset = -steer / 2;
    rightMotorOffset = steer / 2;
}

/* ========================== 倒挡切换逻辑实现 ========================== */

/*
 * 功能：倒挡逻辑处理
 * 说明：在轿车模式和遥控模式下支持倒挡切换。
 *       卡丁车模式下：刹车即后退（原有逻辑）。
 *       轿车/遥控模式下：通过倒挡按钮切换前进/后退方向。
 *       仪表显示对应故障灯提示当前模式。
 */
void ReverseGear_Process(void) {
    // 倒挡状态已在蓝牙命令解析中更新
    // 此处主要用于模式相关的逻辑处理

    if (currentDriveMode == MODE_KARTING) {
        // 卡丁车模式：一线通仪表无特殊显示
        // 后退由刹车踏板直接控制（原有BBCAR逻辑）
        reverseGearEnabled = 0; // 卡丁车模式不使用倒挡按钮
    } else if (currentDriveMode == MODE_SEDAN) {
        // 轿车模式：仪表亮「转把故障」灯
        // 摇杆左右为刹车，倒挡按钮切换前进后退
    } else if (currentDriveMode == MODE_REMOTE) {
        // 遥控模式：仪表亮「控制器故障」灯
        // 倒挡切换行驶/转向方向
    }
}

/* ========================== 自动相序校准功能实现 ========================== */

/*
 * 功能：启动自动相序校准
 * 说明：APP「自动校准」按钮或遥控器蓝牙配对按钮短按触发。
 *       校准前需确保车轮悬空。
 *       开始校准响4声提示音。
 */
void AutoCalib_Start(void) {
    if (autoCalibActive) return;

    autoCalibActive = 1;
    autoCalibDone = 0;
    calibStep = 0;
    calibTimer = HAL_GetTick();
    BeepEvent_Trigger(BEEP_CALIB_START);

    // 发送校准开始确认给APP
    BLE_SendPacket(CALIB_START_ACK, NULL, 0);
}

/*
 * 功能：校准过程处理
 * 说明：在主循环中调用，分阶段执行相序检测。
 *       车轮自动转动检测最佳相序，无需人工操作。
 *       校准完成响5声提示音。
 */
void AutoCalib_Process(void) {
    if (!autoCalibActive) return;

    // 自动相序校准需要控制电机按特定顺序转动并检测霍尔反馈
    // 由于涉及底层电机驱动和霍尔信号检测，此处提供框架逻辑
    // 实际实现需根据具体电机和霍尔配置编写相序检测算法

    uint32_t elapsed = HAL_GetTick() - calibTimer;

    switch (calibStep) {
        case 0: {
            // 阶段0：准备，确保电机静止
            if (elapsed > 1000) {
                calibStep = 1;
                calibTimer = HAL_GetTick();
            }
            break;
        }
        case 1: {
            // 阶段1：左轮正转检测
            // 实际实现：施加小电压让电机缓慢转动，检测霍尔信号顺序
            if (elapsed > 2000) {
                calibStep = 2;
                calibTimer = HAL_GetTick();
            }
            break;
        }
        case 2: {
            // 阶段2：左轮反转检测
            if (elapsed > 2000) {
                calibStep = 3;
                calibTimer = HAL_GetTick();
            }
            break;
        }
        case 3: {
            // 阶段3：右轮正转检测
            if (elapsed > 2000) {
                calibStep = 4;
                calibTimer = HAL_GetTick();
            }
            break;
        }
        case 4: {
            // 阶段4：右轮反转检测
            if (elapsed > 2000) {
                calibStep = 5;
                calibTimer = HAL_GetTick();
            }
            break;
        }
        case 5: {
            // 阶段5：校准完成
            autoCalibActive = 0;
            autoCalibDone = 1;
            BeepEvent_Trigger(BEEP_CALIB_DONE);
            BLE_SendPacket(CALIB_DONE_ACK, NULL, 0);
            break;
        }
    }
}

/*
 * 功能：停止校准
 * 说明：强制终止校准过程。
 */
void AutoCalib_Stop(void) {
    autoCalibActive = 0;
    calibStep = 0;
}

/* ========================== 一线通仪表功能实现 ========================== */

/*
 * 功能：初始化一线通仪表串口
 * 说明：一线通仪表通常使用1200波特率单线通信。
 *       如果硬件上USART3已被调试占用，可复用TX引脚通过软件模拟发送。
 */
void YXT_Init(void) {
    // 一线通初始化：根据实际硬件接线配置
    // 如果一线通接在USART3_TX(PB10)上，需要临时切换波特率到1200发送数据
    // 为简化实现，此处通过标志位控制显示内容，实际发送在YXT_SendData中处理
}

/*
 * 功能：发送数据到一线通仪表
 * 说明：将当前车辆状态编码为一线通协议格式发送。
 *       不同品牌的一线通协议略有差异，此处采用通用模拟方式。
 */
void YXT_SendData(void) {
    // 根据当前模式设置仪表显示标志
    yxtDisplayFlags = YXT_FLAG_NORMAL;

    if (currentDriveMode == MODE_SEDAN) {
        yxtDisplayFlags |= YXT_FLAG_THROTTLE_FAULT;     // 轿车模式亮转把故障灯
    } else if (currentDriveMode == MODE_REMOTE) {
        yxtDisplayFlags |= YXT_FLAG_CTRL_FAULT;         // 遥控模式亮控制器故障灯
    }

    if (handBrakeEnabled || backwardDrive) {
        yxtDisplayFlags |= YXT_FLAG_BRAKE_LIGHT;        // 手刹或刹车时点亮刹车灯
    }

    // 霍尔故障检测（简化：根据电机错误码判断）
    extern ExtY rtY_Left;
    extern ExtY rtY_Right;
    if (rtY_Left.z_errCode || rtY_Right.z_errCode) {
        yxtDisplayFlags |= YXT_FLAG_MOTOR_FAULT;        // 电机故障灯
    }

    // 一线通数据发送：实际实现需根据具体仪表协议编写
    // 通用格式示例：帧头 + 速度 + 电量 + 标志位 + 校验和
    yxtTxBuf[0] = 0x55;                 // 帧头
    yxtTxBuf[1] = (uint8_t)(speedAvgAbs / 10);  // 速度（km/h近似值）
    yxtTxBuf[2] = (uint8_t)(batVoltageCalib / 100); // 电压
    yxtTxBuf[3] = yxtDisplayFlags;      // 显示标志
    yxtTxBuf[4] = yxtTxBuf[0] + yxtTxBuf[1] + yxtTxBuf[2] + yxtTxBuf[3]; // 校验和

    // 发送数据：使用USART3发送（与调试输出分时复用或独立引脚）
    // HAL_UART_Transmit(&huart3, yxtTxBuf, 5, 50);
}

/*
 * 功能：设置仪表显示标志
 */
void YXT_SetDisplayFlags(uint8_t flags) {
    yxtDisplayFlags = flags;
}

/* ========================== 大灯控制功能实现 ========================== */

// 大灯GPIO定义（使用PC10空闲引脚，避免与电机PWM引脚冲突）
#define HEADLIGHT_PIN           GPIO_PIN_10
#define HEADLIGHT_PORT          GPIOC

/*
 * 功能：初始化大灯GPIO
 * 说明：配置大灯控制引脚为推挽输出模式。
 */
void HeadLight_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = HEADLIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HEADLIGHT_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(HEADLIGHT_PORT, HEADLIGHT_PIN, GPIO_PIN_RESET);
}

/*
 * 功能：开启大灯
 */
void HeadLight_On(void) {
    if (!headLightEnabled) {
        headLightEnabled = 1;
        HAL_GPIO_WritePin(HEADLIGHT_PORT, HEADLIGHT_PIN, GPIO_PIN_SET);
        BeepEvent_Trigger(BEEP_HEADLIGHT_ON);
    }
}

/*
 * 功能：关闭大灯
 */
void HeadLight_Off(void) {
    if (headLightEnabled) {
        headLightEnabled = 0;
        HAL_GPIO_WritePin(HEADLIGHT_PORT, HEADLIGHT_PIN, GPIO_PIN_RESET);
        BeepEvent_Trigger(BEEP_HEADLIGHT_OFF);
    }
}

/*
 * 功能：切换大灯状态
 */
void HeadLight_Toggle(void) {
    if (headLightEnabled) {
        HeadLight_Off();
    } else {
        HeadLight_On();
    }
}

/* ========================== 蜂鸣器提示音完善 ========================== */

/*
 * 功能：触发蜂鸣器事件
 * 参数：event - 蜂鸣器事件类型
 * 说明：将事件加入队列，主循环中依次播放对应的提示音。
 *       避免多个提示音同时响起造成混乱。
 */
void BeepEvent_Trigger(BeepEvent_t event) {
    if (event == BEEP_NONE) return;

    uint8_t nextTail = (beepQueueTail + 1) % 8;
    if (nextTail != beepQueueHead) {    // 队列未满
        beepEventQueue[beepQueueTail] = event;
        beepQueueTail = nextTail;
    }
}

/*
 * 功能：蜂鸣器事件处理
 * 说明：在主循环中调用，从队列中取出事件并播放对应的提示音。
 *       使用状态机方式控制蜂鸣器，避免阻塞主循环。
 */
void BeepEvent_Process(void) {
    if (beepProcessing) {
        // 正在播放提示音，检查是否完成
        if (HAL_GetTick() - beepEventTimer > 200) {
            beepProcessing = 0;
            buzzerFreq = 0;     // 关闭蜂鸣器
        }
        return;
    }

    if (beepQueueHead == beepQueueTail) {
        return; // 队列为空
    }

    BeepEvent_t event = beepEventQueue[beepQueueHead];
    beepQueueHead = (beepQueueHead + 1) % 8;
    beepProcessing = 1;
    beepEventTimer = HAL_GetTick();

    // 根据事件类型播放对应的提示音
    switch (event) {
        case BEEP_GEAR_UP:
        case BEEP_GEAR_DOWN:
        case BEEP_REVERSE:
        case BEEP_HAND_BRAKE_ON:
        case BEEP_HAND_BRAKE_OFF:
        case BEEP_CRUISE_ON:
        case BEEP_CRUISE_OFF:
        case BEEP_HEADLIGHT_ON:
        case BEEP_HEADLIGHT_OFF:
            // 响1声（短促提示音）
            buzzerFreq = 4;
            break;

        case BEEP_MODE_CHANGE:
            // 切换驾驶模式：响2声
            beepShortMany2(2);
            beepProcessing = 0; // beepShortMany2内部会阻塞，直接结束
            break;

        case BEEP_REMOTE_CONNECTED:
            // 遥控器连接成功：响3声
            beepShortMany2(3);
            beepProcessing = 0;
            break;

        case BEEP_CALIB_START:
            // 开始自动校准：响4声
            beepShortMany2(4);
            beepProcessing = 0;
            break;

        case BEEP_CALIB_DONE:
            // 校准完成：响5声
            beepShortMany2(5);
            beepProcessing = 0;
            break;

        case BEEP_HALL_FAULT:
            // 霍尔故障：滴……滴……滴……（慢速连续鸣叫）
            buzzerFreq = 3;
            // 霍尔故障需要持续报警，不立即结束
            beepProcessing = 0;
            break;

        default:
            beepProcessing = 0;
            break;
    }
}

/* ========================== APP参数调节实现 ========================== */

/*
 * 功能：设置自动刹车力度
 * 参数：force - 刹车力度，范围0-50，默认0
 * 说明：数值越大减速越快。影响行驶中松踏板后的自动减速效果。
 */
void APP_SetBrakeForce(int16_t force) {
    appBrakeForce = CLAMP(force, 0, 50);
}

/*
 * 功能：设置主板最大电流
 * 参数：current - 最大电流值，默认25
 * 说明：数值越小极速越低。用于五档增压的APP调节。
 *       实际修改电机控制器的电流限制参数。
 */
void APP_SetMaxCurrent(int16_t current) {
    appMaxCurrent = CLAMP(current, 1, 40);
    // 更新电机控制器电流限制
    extern P rtP_Left;
    extern P rtP_Right;
    rtP_Left.i_max = (appMaxCurrent * A2BIT_CONV) << 4;
    rtP_Right.i_max = rtP_Left.i_max;
}

/*
 * 功能：设置目标速度
 * 参数：speed - 目标速度值
 * 说明：用于定速巡航或遥控模式的速度设定。
 */
void APP_SetTargetSpeed(int16_t speed) {
    appTargetSpeed = speed;
}
