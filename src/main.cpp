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
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

#include "TheThingsNetwork.h"
#include <Arduino.h>
#include "display.h"
#include "voltage.h"



extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program();

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */

const char *devEui = "0013BB291B9FE7C5";
const char *appEui = "70B3D57ED0029C1F";
const char *appKey = "1D48B6FABE11BCC164028B0F0E9F4990";

// Pins and other resources
#define TTN_SPI_HOST HSPI_HOST
#define TTN_SPI_DMA_CHAN 1
#define TTN_PIN_SPI_SCLK 5
#define TTN_PIN_SPI_MOSI 27
#define TTN_PIN_SPI_MISO 19
#define TTN_PIN_NSS 18
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST TTN_NOT_CONNECTED
#define TTN_PIN_DIO0 26
#define TTN_PIN_DIO1 33

TheThingsNetwork ttn;


#define LED_GREEN (gpio_num_t) 
#define LED_RED 25

const unsigned TX_INTERVAL = 30;

void sendMessages(void *pvParameter)
{
    while (1)
    {
        printf("Sending message...\n");
        readSensorValues();
        // printValues();

        TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}

void arduinoTask(void *pvParameter) {
    pinMode(LED_RED, OUTPUT);
    while(1) {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        delay(200);
    }
}

extern "C" void app_main(void)
{
    initArduino();
    initDisplay();
    initVoltage();
    readSensorValues();
    printValues();
    
    xTaskCreate(&arduinoTask, "arduino_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);


    Serial.begin(115200);
    Serial.println("Start!");

    esp_err_t err;

    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    // spi_bus_config.intr_flags = spi_bus_config.intr_flags | !(ESP_INTR_FLAG_HIGH|ESP_INTR_FLAG_EDGE|ESP_INTR_FLAG_INTRDISABLED);
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 2048 * 4, (void *)0, 3, nullptr);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }

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