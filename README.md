# MAX6615 C Library for STM32 HAL

### Dual-Channel Temperature Monitors and Fan-Speed Controllers with Thermistor Inputs

This library is made for the STM32 HAL (Hardware Abstraction Library) platform. The example code is for STM32CubeMX and Keil uVision 5 IDE.

---

### Usage

```c
/* USER CODE BEGIN Includes */
#include "max6615.h"           // include the library
#define MAX6615_address 0x30
```
```c
/* USER CODE BEGIN 0 */
MAX6615 fanDriver;

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 2 */

    // initialize using the MAX6615 instance, the I2C handle and the chip address
    MAX6615_Init (&fanDriver, &hi2c1, SG_ADDRESS_MAX6615);
    
    // enable manual PWM control on both channels
    MAX6615_PWM_EnableManual(&fanDriver, 1);
    MAX6615_PWM_EnableManual(&fanDriver, 2);

    // OR...
    // enable automatic / temperature controlled PWM
    // channel, start duty cycle (10%), start temperature (50°C)
    MAX6615_PWM_EnableAutomatic(&fanDriver, 1, 10, 50);
    MAX6615_PWM_EnableAutomatic(&fanDriver, 2, 10, 50);
    MAX6615_SetTempOffset(&fanDriver, 1, -5);   // temperature offset -4°C
    MAX6615_SetTempOffset(&fanDriver, 2, -6);   // temperature offset -6°C

    float temp1 = 0.0f, temp2 = 0.0;
    uint8_t pwmspeed = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /* Automatic PWM mode */
        MAX6615_ReadTemperature(&fanDriver, 1, &temp1);     // read temperature
        MAX6615_ReadTemperature(&fanDriver, 2, &temp2);
        MAX6615_Read8(&fanDriver, MAX6615_PWM_1_INSTA_DC, &pwmspeed1); // read speed
        MAX6615_Read8(&fanDriver, MAX6615_PWM_2_INSTA_DC, &pwmspeed2);
        printf("Temp 1: %.3f°C Temp 2: %.3f°C \tFan 1: %d%%Fan2: %d%%\n", temp1, temp2, (int)(pwmspeed1/2.4f), (int)(pwmspeed2/2.4));

        /* Manual PWM mode */
        MAX6615_PWM_SetPWM(&fanDriver, 1, pwmspeed  );
        MAX6615_PWM_SetPWM(&fanDriver, 2, pwmspeed++);
    }
  /* USER CODE END 3 */

}

```

---

### Restrictions

Please note, that many features are not implemented or tested.
The default library is set to affect the PWM settings immediately without the ramping over time. Furthermore, the GPIO features of the MAX6616 chip have not been implemented. Please also note, that the tacho readout feature has not been tested.

---

### Sources

MAX6615/16 Datasheet [maximintegrated.com](https://datasheets.maximintegrated.com/en/ds/MAX6615-MAX6616.pdf)

---

### License

MIT License

Copyright (c) 2018 ETA Systems

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.