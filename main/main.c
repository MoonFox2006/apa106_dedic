#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "APA106.h"

#define LED_GPIO    GPIO_NUM_18

apa106_handle_t apa;

void app_main() {
    ESP_ERROR_CHECK(apa106_init(LED_GPIO, &apa));

    for (uint16_t r = 0; r <= 255; r = (r << 1) | 1) {
        apa106_rgb(apa, r, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    for (uint16_t g = 0; g <= 255; g = (g << 1) | 1) {
        apa106_rgb(apa, 0, g, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    for (uint16_t b = 0; b <= 255; b = (b << 1) | 1) {
        apa106_rgb(apa, 0, 0, b);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    for (uint16_t w = 0; w <= 255; w = (w << 1) | 1) {
        apa106_rgb(apa, w, w, w);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    apa106_rgb(apa, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        apa106_rgb(apa, random() % 128, random() % 128, random() % 128);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
