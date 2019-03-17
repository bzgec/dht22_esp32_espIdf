//#include <Arduino.h>
#include "main.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "DHT22_esp32.h"
#include "Types.h"


#define DHTPIN 27     // what digital pin we're connected to
#define INTERRUPT_PIN 26
#define LED_BUILTIN 2
#define LED_TEST 13

DWORD g_dwErrors = 0;
DWORD g_dwReadings = 0;


#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
  //INTERRUPTS_DISABLE();
  static DWORD s_dwInterruptCounter = 0;
  static DWORD s_dwLastCall = 0;

  if (esp_timer_get_time() - s_dwLastCall >= 250*1000)
  {
    if(++s_dwInterruptCounter % 2)
    {
      gpio_set_level(LED_BUILTIN, 1);
    }
    else
    {
      gpio_set_level(LED_BUILTIN, 0);
    }
    s_dwLastCall = esp_timer_get_time();
  }
  
  
  //INTERRUPTS_ENABLE();
}

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(LED_BUILTIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
    while(1) 
    {
        /* Blink off (output low) */
        gpio_set_level(LED_TEST, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(LED_TEST, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void read_dht22(void *pvParameter)
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float fHumidity;
  float fTemperature;
  BOOL bSuccess = FALSE;

  while(1) 
  {
    g_dwReadings++;
    bSuccess = dht22_read(&fTemperature, &fHumidity);
    if(!bSuccess)
    {
      g_dwErrors++;
      printf("Failed to read from DHT sensor!\n");
    }
    else
    {
      // Compute heat index in Celsius (isFahreheit = false)
      float fHeatIndex = dht22_computeHeatIndex(fTemperature, fHumidity);
      float fInternalTemp = (temprature_sens_read() - 32) / 1.8;

      printf("Temperature: %.1f*C\t", fTemperature);
      printf("Humidity: %.1f%%\t", fHumidity);
      printf("Heat index: %.1f*C\t", fHeatIndex);
      printf("Errors/Readings: %u/%u\n", g_dwErrors, g_dwReadings);
      printf("Internal temperature: %.1f*C\n", fInternalTemp);
    }
    
    vTaskDelay(4000 / portTICK_PERIOD_MS);

  }
}

void app_main()
{
#ifdef ENABLE_SERIAL_PRINT
  //Serial.begin(SERIAL_BAUD_RATE);
  //Serial.println(); Serial.println(); Serial.println(); Serial.println(); Serial.println(); Serial.println();
  //Serial.setDebugOutput(true);
#endif

  dht22_init(DHTPIN);
  init_gpio();

  xTaskCreate(&read_dht22, "dht22_task", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);
  xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}

void init_gpio()
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
  //set as output mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1UL<<INTERRUPT_PIN);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, (void*) INTERRUPT_PIN);


  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1UL<<LED_TEST);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //enable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}