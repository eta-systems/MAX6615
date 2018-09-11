
/**
  * @file       max6615.h
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright  MIT license
  * @date       2018
  * @brief      C library for the MAX6615 PWM Fan Controller for STM32 HAL.
  * @details
  * @see        github.com/eta-systems
  * @see        https://datasheets.maximintegrated.com/en/ds/MAX6615-MAX6616.pdf
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAX_6615_H
#define __MAX_6615_H

/**
  * @note  tested using STM32F373
  */
#ifndef STM32F3XX_H
#include "stm32f3xx_hal.h"
#endif

#ifndef STM32F3XX_HAL_I2C_H
#include "stm32f3xx_hal_i2c.h"
#endif

#ifndef MAIN_H
#include "main.h"
#endif

/**
  * @see    Datasheet p.12 - Table 3. Register Map
  */
#define MAX6615_TEMP_CH_1                   0x00
#define MAX6615_TEMP_CH_2                   0x01
#define MAX6615_CONF_BYTE                   0x02
#define MAX6615_TEMP_CH_1_OT_LIMIT          0x03
#define MAX6615_TEMP_CH_2_OT_LIMIT          0x04
#define MAX6615_OT_STATUS                   0x05
#define MAX6615_OT_MASK                     0x06
#define MAX6615_PWM_1_START_DC              0x07
#define MAX6615_PWM_2_START_DC              0x08
#define MAX6615_PWM_1_MAX_DC                0x09
#define MAX6615_PWM_2_MAX_DC                0x0A
#define MAX6615_PWM_1_TARGET_DC             0x0B
#define MAX6615_PWM_2_TARGET_DC             0x0C
#define MAX6615_PWM_1_INSTA_DC              0x0D
#define MAX6615_PWM_2_INSTA_DC              0x0E
#define MAX6615_CH_1_FAN_START_TEMP         0x0F
#define MAX6615_CH_2_FAN_START_TEMP         0x10
#define MAX6615_FAN_CONF                    0x11
#define MAX6615_DUTY_CYCLE_RATE_CHANGE      0x12
#define MAX6615_DUTY_CYCLE_STEP_SIZE        0x13
#define MAX6615_PWM_FREQ_SEL                0x14
#define MAX6615_GPIO_FUNCTION               0x15
#define MAX6615_GPIO_VALUE                  0x16
#define MAX6615_THERMISTOR_OFFSET           0x17
#define MAX6615_TACH_1_VAL                  0x18
#define MAX6615_TACH_2_VAL                  0x19
#define MAX6615_TACH_1_LIMIT                0x1A
#define MAX6615_TACH_2_LIMIT                0x1B
#define MAX6615_FAN_STATUS                  0x1C
#define MAX6615_TEMP_CH_1_LSB               0x1E
#define MAX6615_TEMP_CH_2_LSB               0x1F
#define MAX6615_DEVICE_REV                  0xFD
#define MAX6615_DEVICE_ID                   0xFE
#define MAX6615_MANUFACTURER_ID             0xFF

typedef struct {
  uint16_t devAddress;
  I2C_HandleTypeDef *wireIface;
} MAX6615;

MAX6615 new_MAX6615               (void);
uint8_t MAX6615_Write8            (MAX6615* chip, uint8_t reg, uint8_t val);
uint8_t MAX6615_Read8             (MAX6615* chip, uint8_t reg, uint8_t* val);
uint8_t MAX6615_Init              (MAX6615* chip, I2C_HandleTypeDef* wireIface, uint16_t devAddress);
uint8_t MAX6615_ReadTemperature   (MAX6615* chip, uint8_t channel, float* temp);




#endif /* __MAX_6615_H */
/************************ (C) COPYRIGHT ETA Systems *****END OF FILE****/

