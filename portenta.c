// Based on the k210 and esp variants of this library  
// As we are on MBed for the Portenta we can utilise the timing libraries from that platform
#if defined(ARDUINO_PORTENTA_H7_M7)

#include <stdbool.h>	// Erroring in Arduino.h without this include
#include <mbed.h>
#include <Arduino.h>

void  portentaShow(
    uint8_t pin, uint8_t *pixels, uint32_t numBytes, boolean is800KHz)
{

//From https://forum.arduino.cc/t/fast-gpio-toggle-with-portenta-h7-using-hal/682470
// ------------------------ GPIO Pin Initialization for STM32 ------------------------
  /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOH_CLK_ENABLE();

 GPIO_InitTypeDef GPIO_InitStruct; 

 /* TODO: Map these based on "pin" passed in */
 uint16_t GPIO_Pin = GPIO_PIN_15; // Pin15  -> D0 on arduino portenta  
 GPIO_TypeDef *GIO_Port = GPIOH;   // D0 on     GPIO-H

  /*Configure GPIO pin */
 GPIO_InitStruct.Pin = GPIO_Pin;       
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output, push-pull configuration
 GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
 HAL_GPIO_Init(GIO_Port, &GPIO_InitStruct);  
 HAL_GPIO_WritePin(GIO_Port, GPIO_Pin, GPIO_PIN_RESET);

#define CYCLES_800_T0H (125) // 0.3us ( minus 0.15us for IO
#define CYCLES_800_T1H (500) // 0.7us
#define CYCLES_800 (800)        // 1.25us per bit
#define CYCLES_800_RESET (6000) // 60ms reset pulse
#define CYCLES_400_T0H (500) // 0.5uS
#define CYCLES_400_T1H (1200)  // 1.2us
#define CYCLES_400 (2500)      // 2.5us per bit
#define CYCLES_400_RESET (12000)      // 120ms per bit

    uint8_t *p, *end, pix, mask;
    uint32_t th,tl, time0, time1, period, reset;

    p = pixels;
    end = p + numBytes;
    pix = *p++;
    mask = 0x80;

    if (is800KHz)
    {
        time0 = CYCLES_800_T0H;
        time1 = CYCLES_800_T1H;
        period = CYCLES_800;
        reset = CYCLES_800_RESET;
    }
    else
    { // 400 KHz bitstream
        time0 = CYCLES_400_T0H;
        time1 = CYCLES_400_T1H;
        period = CYCLES_400;
        reset = CYCLES_400_RESET;
    }

    wait_ns(reset);

    while (true) // Wait until break
    {
        if (pix & mask) { th = time1; tl = period-th; } 
        else { th = time0; tl = period-th; }
        HAL_GPIO_WritePin(GIO_Port, GPIO_Pin, GPIO_PIN_SET);
        wait_ns(th); 
        HAL_GPIO_WritePin(GIO_Port, GPIO_Pin, GPIO_PIN_RESET);
        wait_ns(tl); 

        if (!(mask >>= 1))
        { // Next bit/byte
            if (p >= end)
                break;
            pix = *p++;
            mask = 0x80;
        }
    }
    wait_ns(period);
        ; // Wait for last bit
}

#endif // ARDUINO_PORTENTA_H7_M7
