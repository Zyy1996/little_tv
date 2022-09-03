/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// #include <stdio.h>
// #include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "unity.h"
// #include "test_utils.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_system.h"
#include "soc/soc_caps.h"
#include "test_spi_board.h"


// #include "lvgl_helpers.h"

void app_main(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    lcd_initialize_spi(&io_handle, NULL, NULL, 8, 8, false);
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_LCD_RST_GPIO,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };
    TEST_ESP_OK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    lcd_panel_test(io_handle, panel_handle);
    while (1);
}

// void app_main(void)
// {
//     printf("Hello world!\n");

//     /* Print chip information */
//     esp_chip_info_t chip_info;
//     esp_chip_info(&chip_info);
//     printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
//             CONFIG_IDF_TARGET,
//             chip_info.cores,
//             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
//             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

//     printf("silicon revision %d, ", chip_info.revision);

//     printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
//             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

//     printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

//     for (int i = 10; i >= 0; i--) {
//         printf("Restarting in %d seconds...\n", i);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     printf("Restarting now.\n");
//     fflush(stdout);
//     esp_restart();
// }
