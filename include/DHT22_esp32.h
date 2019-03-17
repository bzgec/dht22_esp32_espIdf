#ifndef __DHT22_h
#define __DHT22_h

#include <stdio.h>
#include "Types.h"
#include "driver/gpio.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"


#define _DHT22_MIN_INTERVAL   2000*1000  // in microseconds
#define _DHT22_DEBUG  // Uncomment to enable printing out debug messages.
#define _DHT22_TIMEOUT -1

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW  0
#endif


#ifndef INTERRUPTS_DISABLE
#define INTERRUPTS_DISABLE()  portDISABLE_INTERRUPTS()
#endif  // INTERRUPTS_DISABLE

#ifndef INTERRUPTS_ENABLE
#define INTERRUPTS_ENABLE()   portENABLE_INTERRUPTS()
#endif  // INTERRUPTS_ENABLE


#define CONVERT_C_TO_F(degC) ((degC)*1.8 + 32)
#define CONVERT_F_TO_C(degF) (((degF) - 32) * 0.55555)    // 5/9 = 0.5555555555555556...

#ifdef __cplusplus
extern "C" {
#endif

void dht22_init(BYTE pin);
BOOL dht22_readTemp(float* pfTemp);
BOOL dht22_readHumidity(float* pfHumidity);
BOOL dht22_read(float* pfTemp, float* pfHumidity);
float dht22_computeHeatIndex(float fTemperature, float fPercentHumidity);

#ifdef __cplusplus
}
#endif




#endif  // __DHT22_h
