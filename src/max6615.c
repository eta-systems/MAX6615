
/**
  * @file       max6615.c
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright  MIT license
  * @date       2018
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


