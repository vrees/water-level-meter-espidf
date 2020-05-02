#include <Arduino.h>
#include "esp_event.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
// #include "esp_sleep.h"
// #include "soc/rtc_cntl_reg.h"
// #include "soc/sens_reg.h"
// #include "driver/gpio.h"
// #include "driver/rtc_io.h"

#include "TheThingsNetwork.h"
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
#define TTN_PIN_RST 23
#define TTN_PIN_DIO0 26
#define TTN_PIN_DIO1 33
#define TTN_PIN_DIO2 32
#define TTN_PIN_RXTX TTN_NOT_CONNECTED

TheThingsNetwork ttn;


void sendMessages(void *pvParameter)
{

  printf("Sending message...\n");
  readSensorValues();
  // printValues();

  TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
  printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
}

void initLora()
{
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
    // xTaskCreate(sendMessages, "send_messages", 2048 * 4, (void *)0, 3, nullptr);
    sendMessages(NULL);
  }
  else
  {
    printf("Join failed. Goodbye\n");
  }
}
