#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f1xx_hal.h"


// /* TMC2209 UART命令 */
// #define TMC2209_SYNC_BYTE       0x05
// #define TMC2209_MASTER_ADDR     0x00

// /* 寄存器地址 */
// #define REG_GCONF               0x00
// #define REG_GSTAT               0x01
// #define REG_IFCNT               0x02
// #define REG_SLAVECONF           0x03
// #define REG_OTP_PROG            0x04
// #define REG_OTP_READ            0x05
// #define REG_IOIN                0x06
// #define REG_FACTORY_CONF        0x07
// #define REG_IHOLD_IRUN          0x10
// #define REG_TPOWERDOWN          0x11
// #define REG_TSTEP               0x12
// #define REG_TPWMTHRS            0x13
// #define REG_VACTUAL             0x22
// #define REG_TCOOLTHRS           0x14
// #define REG_THIGH               0x15
// #define REG_RAMPMODE            0x20
// #define REG_XACTUAL             0x21
// #define REG_DRV_STATUS          0x6F
// #define REG_PWMCONF             0x70
// #define REG_PWM_SCALE           0x71
// #define REG_PWM_AUTO            0x72
// #define REG_COOLCONF            0x42
// #define REG_SG_RESULT           0x41
// #define REG_CHOPCONF            0x6C

// /* 堵转检测相关定义 */
// #define STALLGUARD_THRESHOLD_MIN    -64
// #define STALLGUARD_THRESHOLD_MAX    63
// #define SG_RESULT_STALLED           0

// /* DIAG引脚配置 */
// #define DIAG_MODE_DISABLED          0x00  // 禁用DIAG输出
// #define DIAG_MODE_STEP_LOSS         0x01  // 步进丢失检测
// #define DIAG_MODE_STALLGUARD        0x02  // Stallguard堵转检测
// #define DIAG_MODE_INDEX             0x03  // 索引标记
// #define DIAG_ACTIVE_HIGH            0x00  // 高电平有效
// #define DIAG_ACTIVE_LOW             0x04  // 低电平有效
// #define DIAG_PULSE_MODE             0x08  // 脉冲模式（否则为电平模式）

// /* 电流设置相关定义 */
// #define CURRENT_MAX                  31    // 最大电流值（0-31对应0-100%电流）
// #define HOLD_DELAY_MAX               15    // 最大保持延迟
// #define DEFAULT_RUN_CURRENT         16    // 默认运行电流（50%）
// #define DEFAULT_HOLD_CURRENT        8     // 默认保持电流（25%）
// #define DEFAULT_HOLD_DELAY          5     // 默认保持延迟
// #define DEFAULT_MICROSTEP          4     // 默认微步细分(1/4)

// /* TPOWER DOWN相关定义 */
// #define TPOWERDOWN_MIN               0     // 最小断电延迟（立即断电）
// #define TPOWERDOWN_MAX             255     // 最大断电延迟（255个单位）
// #define TPOWERDOWN_UNIT_MS         10     // 每个单位对应的毫秒数
// #define DEFAULT_TPOWERDOWN          10     // 默认断电延迟（100ms）

// typedef struct {
//     UART_HandleTypeDef *huart;
//     uint8_t slave_addr;
// } TMC2209_Handle;

// /* 函数原型 */
// void TMC2209_Init(TMC2209_Handle *handle, UART_HandleTypeDef *huart, uint8_t slave_addr);
// HAL_StatusTypeDef TMC2209_ReadReg(TMC2209_Handle *handle, uint8_t reg, uint32_t *data);
// HAL_StatusTypeDef TMC2209_WriteReg(TMC2209_Handle *handle, uint8_t reg, uint32_t data);
// HAL_StatusTypeDef TMC2209_SetSpeed(TMC2209_Handle *handle, int32_t speed);
// HAL_StatusTypeDef TMC2209_SetCurrent(TMC2209_Handle *handle, uint8_t current);
// HAL_StatusTypeDef TMC2209_SetCurrentDetailed(TMC2209_Handle *handle, uint8_t run_current, uint8_t hold_current, uint8_t hold_delay);
// HAL_StatusTypeDef TMC2209_SetPowerDownDelay(TMC2209_Handle *handle, uint16_t tpowerdown);
// HAL_StatusTypeDef TMC2209_Stop(TMC2209_Handle *handle);
// uint8_t TMC2209_CalcCRC(uint8_t *data, uint8_t len);
// HAL_StatusTypeDef TMC2209_SetStallguardThreshold(TMC2209_Handle *handle, int8_t sgt);
// HAL_StatusTypeDef TMC2209_SetStallguardSpeed(TMC2209_Handle *handle, uint32_t speed_threshold);
// HAL_StatusTypeDef TMC2209_CheckStalled(TMC2209_Handle *handle, uint8_t *is_stalled);
// HAL_StatusTypeDef TMC2209_ConfigureDIAG(TMC2209_Handle *handle, uint8_t diag_config);
// HAL_StatusTypeDef TMC2209_SetMicrostep(TMC2209_Handle *handle, uint16_t microstep);

#endif /* TMC2209_H */
