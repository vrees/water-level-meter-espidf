/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * https://github.com/manuelbl/ttn-esp32
 *******************************************************************************/


#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "TheThingsNetwork.h"
#include "ttn.h"




extern "C" void app_main(void)
{



    ttnInit();    

    
}
