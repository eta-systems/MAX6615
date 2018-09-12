
/**
  * @file       max6615.c
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright  MIT license
  * @date       2018.09.12
  * @brief      C library for the MAX6615 PWM Fan Controller for STM32 HAL.
  * @details
  * @see        github.com/eta-systems
  * @see        https://datasheets.maximintegrated.com/en/ds/MAX6615-MAX6616.pdf
  */

#include "max6615.h"
#include "main.h"

/**
  * @brief    Initalizer function
  * @param    *chip, pointer to the MAX6615 typedef struct
  * @param    *wireIface a pointer to a HAL I2C_HandleTypeDef
  * @param    address of the chip on the I2C bus
  * @return   0 on success, 1 on I2C transmission error
  * @pre      all Inputs & Outputs must be declared before calling begin()
  */
uint8_t MAX6615_Init(MAX6615 *chip, I2C_HandleTypeDef *wireIface, uint16_t address){
  chip->wireIface = wireIface;
  chip->devAddress = address;
  return 0;
}

/**
  * @brief    writes a single value into a MAX6615 register
  * @param    *chip, pointer to the MAX6615 typedef struct
  * @param    reg, the destination register's address
  * @param    val, the value for the destination register
  */
uint8_t MAX6615_Write8(MAX6615 *chip, uint8_t reg, uint8_t val){
  if(HAL_I2C_Mem_Write(chip->wireIface, chip->devAddress, reg, 1, &val, 1, 10) != HAL_OK) 
    return 1;
  return 0;
}

/**
  * @brief    reads a single value from a MAX6615 register
  * @param    *chip, pointer to the MAX6615 typedef struct
  * @param    reg, the destination register's address
  * @param    val, pointer to the location where the value shall be stored
  * @return   0 on success, 1 on transmission error
  */
uint8_t MAX6615_Read8(MAX6615 *chip, uint8_t reg, uint8_t *val){
  if(HAL_I2C_Mem_Read(chip->wireIface, chip->devAddress, reg, 1, val, 1, 10) != HAL_OK) 
    return 1;
  return 0;
}

uint8_t MAX6615_ReadTemperature(MAX6615* chip, uint8_t channel, float* temp){
  uint8_t msb = 0, lsb = 0;
  if(channel == 1){
    MAX6615_Read8(chip, MAX6615_TEMP_CH_1,     &msb);
    MAX6615_Read8(chip, MAX6615_TEMP_CH_1_LSB, &lsb);
  } else if(channel == 2) {
    MAX6615_Read8(chip, MAX6615_TEMP_CH_2,     &msb);
    MAX6615_Read8(chip, MAX6615_TEMP_CH_2_LSB, &lsb);
  } else {
    return 1;
  }
  *temp = (float)(msb) + ((float)(lsb)/256.0f);
  return 0;
}

// untested
uint8_t MAX6615_ReadTacho(MAX6615* chip, uint8_t channel, uint8_t* tacho){
  if(channel == 1){
    MAX6615_Read8(chip, MAX6615_TACH_1_VAL, tacho);
  } else if(channel == 2) {
    MAX6615_Read8(chip, MAX6615_TACH_2_VAL, tacho);
  } else {
    return 1;
  }
  return 0;
}

uint8_t MAX6615_PWM_EnableManual(MAX6615* chip, uint8_t channel){
  uint8_t regval = 0;
  if(channel == 1){
    MAX6615_Read8(chip, MAX6615_FAN_CONF, &regval);
    regval &= ~((1 << 5) | (1 << 4));    // clear D5 / D4
    MAX6615_Write8(chip, MAX6615_FAN_CONF, regval);
  } else if(channel == 2){
    MAX6615_Read8(chip, MAX6615_FAN_CONF, &regval);
    regval &= ~((1 << 3) | (1 << 2));    // clear D3 / D2
    MAX6615_Write8(chip, MAX6615_FAN_CONF, regval);
  } else {
    return 1;
  }
  // set the rate of change to immediately change the PWM
  // Datasheet p.15 - Duty-Cycle Rate of Change (12h)
  regval = 0x00;
  MAX6615_Write8(chip, MAX6615_DUTY_CYCLE_RATE_CHANGE, regval);
  return 0;
}

uint8_t MAX6615_PWM_EnableAutomatic(MAX6615* chip, uint8_t channel, uint8_t fanStartDC, uint8_t fanStartTemp){
  uint8_t regval = 0;
  uint8_t pwm = (uint8_t) ((float)fanStartDC * 2.4f);
  if(channel == 1){
    MAX6615_Write8(chip, MAX6615_PWM_1_START_DC, pwm);
    MAX6615_Write8(chip, MAX6615_CH_1_FAN_START_TEMP, fanStartTemp);
    MAX6615_Write8(chip, MAX6615_PWM_1_MAX_DC, 240);
    MAX6615_Write8(chip, MAX6615_CH_1_FAN_START_TEMP, fanStartTemp);
    MAX6615_Read8(chip, MAX6615_FAN_CONF, &regval);
    regval |= (1 << 5);
    MAX6615_Write8(chip, MAX6615_FAN_CONF, regval);
  } else if(channel == 2){
    MAX6615_Write8(chip, MAX6615_PWM_2_START_DC, pwm);
    MAX6615_Write8(chip, MAX6615_CH_2_FAN_START_TEMP, fanStartTemp);
    MAX6615_Write8(chip, MAX6615_PWM_2_MAX_DC, 240);
    MAX6615_Write8(chip, MAX6615_CH_2_FAN_START_TEMP, fanStartTemp);
    MAX6615_Read8(chip, MAX6615_FAN_CONF, &regval);
    regval |= (1 << 2);
    MAX6615_Write8(chip, MAX6615_FAN_CONF, regval);
  } else {
    return 1;
  }

  // set the rate of change to immediately change the PWM
  // Datasheet p.15 - Duty-Cycle Rate of Change (12h)
  regval = (1 << 6) | (1 << 3);
  MAX6615_Write8(chip, MAX6615_DUTY_CYCLE_RATE_CHANGE, regval);

  // add: bool disableWhenCold to the function parameters to specify the minimum fan duty cycle
  /*
  MAX6615_Read8(chip, MAX6615_CONF_BYTE, &regval);
  if(disableWhenCold)
    regval &= ~(1<<2);    // clear D2 - minimum duty cycle = 0%
  else
    regval |= (1<<2);     // set D2 - minimum duty cycle = fanStartDutyCycle
  MAX6615_Write8(chip, MAX6615_CONF_BYTE, regval);
  */
  return 0;
}

uint8_t MAX6615_PWM_SetPWM(MAX6615* chip, uint8_t channel, uint8_t percent){
  uint8_t pwm = (uint8_t) ((float)percent * 2.4f);
  if(channel == 1){
    MAX6615_Write8(chip, MAX6615_PWM_1_TARGET_DC, pwm);
  } else if(channel == 2){
    MAX6615_Write8(chip, MAX6615_PWM_2_TARGET_DC, pwm);
  } else {
    return 1;
  }
	return 0;
}

uint8_t MAX6615_SetTempOffset(MAX6615* chip, uint8_t channel, int8_t degrees){
  if(channel > 2)
    return 1;
  if(degrees > 15)
    (degrees = 15);     // range check
  if(degrees < -15)
    (degrees = -15);    // range check
  uint8_t val = degrees/2;
  if(degrees < 0)
    val = ((degrees/2) & 0x0F) | 0x08;  // right shift: LSB = 2°C / clear top 4bits / add minus sign
  if(channel == 1)
    val = val << 4;
  uint8_t regval = 0;
  MAX6615_Read8(chip, MAX6615_THERMISTOR_OFFSET, &regval);
  (channel == 1) ? (regval &= 0x0F) : (regval &= 0xF0);     // clear befor or-masking value
  regval |= val;
  MAX6615_Write8(chip, MAX6615_THERMISTOR_OFFSET, regval);
  return 0;
}
