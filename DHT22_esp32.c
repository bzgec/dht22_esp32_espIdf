/*
 * This was copied and edited from https://github.com/adafruit/DHT-sensor-library, and also from here https://github.com/Andrey-m/DHT22-lib-for-esp-idf
 * 
 * First you need to call dht22_init() function.
 * Than you can call functions dht22_readTemp(), dht22_readHumidity, dht22_read()
 * all the time BUT if time between readings is less than 2s it will just return 
 * the last last operation status (if reading
 * was successful) and since you pass your variables the last reading which was done).
 * Note that functions will retrun FALSE only if error happened during reading sensor data,
 * but not if there was less then 2s between calling functions mentioned above.
 * Also note that if last reading failed the data is not going to change. So the last
 * good reading of temperature/humidity is "returned".
 */

#include "DHT22_esp32.h"

static DWORD fs_dwLastReadTime;
static BYTE  fs_byPin;
static BYTE  fs_abyData[5];   // 40 bits of data is sent from AM2302 or DHT22
static BOOL  fs_bLastReadingSuccessful;  // saves if last reading was successful or not
                                         // in case the last reading failed and next reading tries
                                         // to be done in under of 2s we need to return the bool of
                                         // the last reading


static BOOL timePassed();
static BOOL read();
static DWORD expectPulse(BOOL bLevel, WORD wTimeOut);
static void setPin_inputPullUp();

void dht22_init(BYTE byPin)
{
  fs_byPin = byPin;
  setPin_inputPullUp();
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  fs_dwLastReadTime = xTaskGetTickCount() - _DHT22_MIN_INTERVAL;
}

static void setPin_inputPullUp()
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1UL<<fs_byPin);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

// Check if sensor was read less than two seconds ago
BOOL timePassed()
{
  DWORD dwCurrentTime;
  dwCurrentTime = esp_timer_get_time();
  if ((dwCurrentTime - fs_dwLastReadTime) < _DHT22_MIN_INTERVAL) 
  {
    return FALSE;
  }
  else
  {
    fs_dwLastReadTime = dwCurrentTime;
    return TRUE;
  }
}


BOOL dht22_readTemp(float* pfTemp)
{
  BOOL bSuccess;
  if (read()) 
  {
    float fTemp;
    fTemp = ((WORD)(fs_abyData[2] & 0x7F)) << 8 | fs_abyData[3];
    fTemp *= 0.1;
    if (fs_abyData[2] & 0x80) 
    {
      fTemp *= -1;
    }

    *pfTemp = fTemp;

    bSuccess = TRUE;
  }
  else
  {
    bSuccess = FALSE;
  }
  
  fs_bLastReadingSuccessful = bSuccess;
  return bSuccess;
}

BOOL dht22_readHumidity(float* pfHumidity)
{
  BOOL bSuccess;
  if (read()) 
  {
    float fHumidity;
    fHumidity = ((WORD)fs_abyData[0]) << 8 | fs_abyData[1];
    fHumidity *= 0.1;
    *pfHumidity = fHumidity;

    bSuccess = TRUE;
  }
  else
  {
    bSuccess = FALSE;
  }

  fs_bLastReadingSuccessful = bSuccess;
  return bSuccess;
}

BOOL dht22_read(float* pfTemp, float* pfHumidity)
{
  BYTE bySuccess = 0x00;

  bySuccess |= dht22_readTemp(pfTemp);  // store value of this opeartion to lsb (in next step it is shifted to left so it is not lsb any more...)
  bySuccess <<= 1;  // shift bits to left so we make space for incoming bit
  bySuccess |= dht22_readHumidity(pfHumidity);  // store value of this opearion to lsb
  if (bySuccess == 0x03) 
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

float dht22_computeHeatIndex(float fTemperature, float fPercentHumidity) 
{
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float fHeatIndex;

  fTemperature = CONVERT_C_TO_F(fTemperature);

  fHeatIndex = 0.5 * (fTemperature + 61.0 + ((fTemperature - 68.0) * 1.2) + (fPercentHumidity * 0.094));

  if (fHeatIndex > 79) 
  {
    fHeatIndex = -42.379 +
                  2.04901523 * fTemperature +
                 10.14333127 * fPercentHumidity +
                 -0.22475541 * fTemperature*fPercentHumidity +
                 -0.00683783 * pow(fTemperature, 2) +
                 -0.05481717 * pow(fPercentHumidity, 2) +
                  0.00122874 * pow(fTemperature, 2) * fPercentHumidity +
                  0.00085282 * fTemperature*pow(fPercentHumidity, 2) +
                 -0.00000199 * pow(fTemperature, 2) * pow(fPercentHumidity, 2);

    if((fPercentHumidity < 13) && (fTemperature >= 80.0) && (fTemperature <= 112.0))
    {
      fHeatIndex -= ((13.0 - fPercentHumidity) * 0.25) * sqrt((17.0 - abs(fTemperature - 95.0)) * 0.05882);
    }
    else if((fPercentHumidity > 85.0) && (fTemperature >= 80.0) && (fTemperature <= 87.0))
    {
      fHeatIndex += ((fPercentHumidity - 85.0) * 0.1) * ((87.0 - fTemperature) * 0.2);
    }
  }

  return CONVERT_F_TO_C(fHeatIndex);
}

static BOOL read() 
{
  DWORD adwDataCycles[80];  // not 40 because for one bit of data we need to measure two pulses (low and high)

  for (BYTE i = 0; i < 80; i++)
  {
    adwDataCycles[i] = 0;
  }

  if(!timePassed())  // if time from last reading is not higher than 2s, do not read sensor data, return last readings
  {
    return fs_bLastReadingSuccessful;
  }
  
  // Reset 40 bits of received data to zero.
  fs_abyData[0] = fs_abyData[1] = fs_abyData[2] = fs_abyData[3] = fs_abyData[4] = 0;

  // ##################################################################
  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  gpio_set_direction(fs_byPin, GPIO_MODE_OUTPUT);
  // First set data line low for 3 ms (for a smooth and nice wake up) 
  if(gpio_set_level(fs_byPin, 0) != ESP_OK)
  {
#ifdef _DHT22_DEBUG
    printf("ERROR SETTING GPIO LOW!!!");
#endif
  }

  ets_delay_us(1100);

  // pull up for 25 us for a gentile asking for data
  if(gpio_set_level(fs_byPin, 1) != ESP_OK)
  {
#ifdef _DHT22_DEBUG
    printf("ERROR SETTING GPIO HIGH!!!");
#endif
  }

  ets_delay_us(25);

  // End the start signal by setting data line high for 40 microseconds.
  gpio_set_direction(fs_byPin, GPIO_MODE_INPUT); // change to input mode

  // Turn off interrupts temporarily because the next sections
  // are timing critical and we don't want any interruptions.
  INTERRUPTS_DISABLE();

  // ##################################################################
  // Waiting for sensor's response

  // First expect a low signal for ~80 microseconds (as response/acknowledgement signal)
  if (expectPulse(LOW, 85) == _DHT22_TIMEOUT) 
  {
#ifdef _DHT22_DEBUG
    printf("DHT timeout waiting for start signal low pulse.\n");
#endif
    INTERRUPTS_ENABLE();
    return FALSE;
  }

  if (expectPulse(HIGH, 85) == _DHT22_TIMEOUT) 
  {
#ifdef _DHT22_DEBUG
  	printf("DHT timeout waiting for start signal high pulse.\n");
#endif
    INTERRUPTS_ENABLE();
    return FALSE;
  }

  // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
  // microsecond low pulse followed by a variable length high pulse.  If the
  // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
  // then it's a 1.  We measure the cycle count of the initial 50us low pulse
  // and use that to compare to the cycle count of the high pulse to determine
  // if the bit is a 0 (high state cycle count < low state cycle count), or a
  // 1 (high state cycle count > low state cycle count). Note that for speed all
  // the pulses are read into a array and then examined in a later step.
  for (BYTE i=0; i<80; i+=2) 
  {
    adwDataCycles[i]   = expectPulse(LOW, 56);
    adwDataCycles[i+1] = expectPulse(HIGH, 75);
  }

  // Timing critical code is now complete.
  INTERRUPTS_ENABLE();

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (BYTE i=0; i<40; ++i) 
  {
    DWORD dwLowCycles  = adwDataCycles[2*i];
    DWORD dwHighCycles = adwDataCycles[2*i+1];
    if ((dwLowCycles == _DHT22_TIMEOUT) || (dwHighCycles == _DHT22_TIMEOUT)) 
    {
#ifdef _DHT22_DEBUG
      printf("DHT timeout waiting for pulse.\n");
#endif
      return FALSE;
    }

    // move selected 8 bits (one byte as it is the type of this array) 
    // to left to make place for incoming bit and then save this bit to the space just made
    fs_abyData[i/8] <<= 1;  
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (dwHighCycles > dwLowCycles) 
    {
      // High cycles are greater than 50us low cycle count, must be a 1.
      fs_abyData[i/8] |= 1;  // save bit to the space just made
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

#ifdef _DHT22_DEBUG
  //DEBUG_PRINTLN(F("Received from DHT:"));
  //DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  //DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  //DEBUG_PRINTLN((fs_abyData[0] + fs_abyData[1] + fs_abyData[2] + fs_abyData[3]) & 0xFF, HEX);
#endif

  // Check we read 40 bits and that the checksum matches.
  if (fs_abyData[4] == ((fs_abyData[0] + fs_abyData[1] + fs_abyData[2] + fs_abyData[3]) & 0xFF)) 
  {
    return TRUE;
  }
  else 
  {
#ifdef _DHT22_DEBUG
    printf("DHT checksum failure!\n");
#endif
    return FALSE;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
static DWORD expectPulse(BOOL bLevel, WORD wTimeOut) 
{
  DWORD dwMicroSeconds = 0;

  while ((gpio_get_level(fs_byPin)) == bLevel)
  {
    if (dwMicroSeconds++ >= wTimeOut) 
    {
      return _DHT22_TIMEOUT; // Exceeded timeout, fail. It is OK if it sends out -1 because in should never reach 4294967295 in clock cycles...
    }
    ets_delay_us(1); // uSec delay    
  }
  return dwMicroSeconds;
}
