#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/gpio.h>
#include <driver/dedic_gpio.h>

typedef dedic_gpio_bundle_handle_t apa106_handle_t;

esp_err_t apa106_init(gpio_num_t apa_pin, apa106_handle_t *apa_handle);
esp_err_t apa106_deinit(apa106_handle_t apa_handle);
esp_err_t apa106_rgb(apa106_handle_t apa_handle, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
