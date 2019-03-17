#ifndef __main_h
#define __main_h

#ifndef INTERRUPTS_DISABLE
#define INTERRUPTS_DISABLE()  portDISABLE_INTERRUPTS()
#define INTERRUPTS_ENABLE()   portENABLE_INTERRUPTS()
#endif

#define ENABLE_SERIAL_PRINT
#define SERIAL_BAUD_RATE      115200


//#define F_CPU 240000000L  // ESP32 default i guess

#define ESP_INTR_FLAG_DEFAULT 0

void init_gpio();

#endif  // __main_h
