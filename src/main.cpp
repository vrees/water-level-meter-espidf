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
#include "TheThingsNetwork.h"
#include <Arduino.h>
#include "display.h"
#include "voltage.h"

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
static uint8_t msgData[] = "Hello, world";

void sendMessages(void *pvParameter)
{
    while (1)
    {
        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}

void arduinoTask(void *pvParameter) {
    pinMode(LED_RED, OUTPUT);
    while(1) {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        Serial.println("Hello!");
        delay(200);
    }
}

extern "C" void app_main(void)
{
    initArduino();
    initDisplay();
    initVoltage();
    
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
}
