/*******************************************************************************
* eTapeContinuous Fluid Level Sensor PN-12110215TC-24  PN-6573TC-24
* Sensor Length: 660mm
* Active Sensor Length:12.4" 660mm
* Reference Resistor (Rref): 3000Ω, ±10%    gemessen:   3200Ω
* Sensor Output empty:       3000Ω          
* Sensor Output: full:        300Ω          gemessen:   400Ω
* Resistance Gradient: 44/cm, ±10%          
* PowerRating:  0.5 Watts (VMax = 10V)
*
* Voltage Gradient bei 3.3V:                gemessen:   48.7 V/cm
* s [cm] = 48.7cm/V * U[V] - 17.3cm         gerechnet:  46,5 V/cm
*
* https://github.com/LilyGO/TTGO-LORA32
* https://github.com/bertrik/LoraWanPmSensor/blob/master/Esp32PmSensor/Esp32PmSensor.ino
* https://github.com/manuelbl/ttn-esp32
*/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include <Arduino.h>
#include "display.h"
#include "voltage.h"
#include "lora.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program();

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */

const unsigned TX_INTERVAL = 5;

void arduinoTask(void *pvParameter)
{
    // pinMode(LED_RED, OUTPUT);
    while (1)
    {
        // digitalWrite(LED_RED, !digitalRead(LED_RED));
        delay(200);
    }
}

extern "C" void app_main(void)
{
    Serial.begin(115200);
    Serial.println("Start!");

    initArduino();
    initLora();
    initDisplay();
    initVoltage();
    readSensorValues();
    printValues();

    // xTaskCreate(&arduinoTask, "arduino_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    printf("left app_main() program\n");
}

static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Configure ADC channel */
    /* Note: when changing channel here, also change 'adc_channel' constant
       in adc.S */
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    /* Set (low and high) thresholds, approx. 0.86V*/
    ulp_low_thr = 1;
    ulp_high_thr = 900;

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 2000000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

static void start_ulp_program()
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}