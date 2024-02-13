#include "sdkconfig.h"
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "APA106.h"

#define USE_ASM

#ifdef USE_ASM
#define CSR_GPIO_OUT    0x0805
#define CSR_MPCCR       0x07E2
#endif

#ifndef F_CPU
#define F_CPU   (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000U)
#endif

#define APA_RESET   50 // 50 us
#define APA_LONG    ((uint64_t)F_CPU * 1360 / 1000000000U) // 1.36 us
#define APA_SHORT   ((uint64_t)F_CPU * 350 / 1000000000U) // 0.35 us

esp_err_t apa106_init(gpio_num_t apa_pin, apa106_handle_t *apa_handle) {
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1UL << apa_pin,
        .mode = GPIO_MODE_OUTPUT
    };

    esp_err_t result;

    result = gpio_config(&gpio_cfg);
    if (result == ESP_OK) {
        int dedic_gpio = apa_pin;

        const dedic_gpio_bundle_config_t dedic_cfg = {
            .gpio_array = &dedic_gpio,
            .array_size = 1,
            .flags.out_en = true
        };

        result = dedic_gpio_new_bundle(&dedic_cfg, apa_handle);
    }
    return result;
}

inline esp_err_t apa106_deinit(apa106_handle_t apa_handle) {
    return dedic_gpio_del_bundle(apa_handle);
}

#ifdef USE_ASM
static IRAM_ATTR void dedic_rgb(uint32_t mask, uint32_t rgb) {
    asm volatile (
        "li     t0, 24\n"
        "loop:\n"
        "csrr   t2, %[COUNTER]\n"
        "csrrs  zero, %[GPIO_OUT], a0\n"
        "andi   t1, a1, 0x01\n"
        "beq    t1, zero, bit0\n"
        "addi   t3, t2, %[LONG_DELAY]\n"
        "wait0:\n"
        "csrr   t2, %[COUNTER]\n"
        "bltu   t2, t3, wait0\n"
        "csrrc  zero, %[GPIO_OUT], a0\n"
        "addi   t3, t2, %[SHORT_DELAY]\n"
        "wait1:\n"
        "csrr   t2, %[COUNTER]\n"
        "bltu   t2, t3, wait1\n"
        "j      next\n"
        "bit0:\n"
        "addi   t3, t2, %[SHORT_DELAY]\n"
        "wait2:\n"
        "csrr   t2, %[COUNTER]\n"
        "bltu   t2, t3, wait2\n"
        "csrrc  zero, %[GPIO_OUT], a0\n"
        "addi   t3, t2, %[LONG_DELAY]\n"
        "wait3:\n"
        "csrr   t2, %[COUNTER]\n"
        "bltu   t2, t3, wait3\n"
        "next:\n"
        "srli   a1, a1, 1\n"
        "addi   t0, t0, -1\n"
        "bne    t0, zero, loop\n"
    : : [COUNTER] "i" (CSR_MPCCR), [GPIO_OUT] "i" (CSR_GPIO_OUT), [SHORT_DELAY] "i" (APA_SHORT), [LONG_DELAY] "i" (APA_LONG));
}
#endif

esp_err_t apa106_rgb(apa106_handle_t apa_handle, uint8_t r, uint8_t g, uint8_t b) {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    uint32_t offset;
#ifndef USE_ASM
    uint32_t start;
#endif
    uint32_t rgb = (b << 16) | (g << 8) | r;
    esp_err_t result;

    result = dedic_gpio_get_out_offset(apa_handle, &offset);
    if (result == ESP_OK) {
        dedic_gpio_bundle_write(apa_handle, 0x01, 0);
        esp_rom_delay_us(APA_RESET);
        taskENTER_CRITICAL(&mux);
#ifdef USE_ASM
        dedic_rgb(1 << offset, rgb);
#else
        for (uint8_t i = 0; i < 24; ++i) {
            start = esp_cpu_get_cycle_count();
            dedic_gpio_bundle_write(apa_handle, 0x01, 1);
            if (rgb & 0x01) {
                while (esp_cpu_get_cycle_count() - start < APA_LONG) {}
                start = esp_cpu_get_cycle_count();
                dedic_gpio_bundle_write(apa_handle, 0x01, 0);
                while (esp_cpu_get_cycle_count() - start < APA_SHORT) {}
            } else {
                while (esp_cpu_get_cycle_count() - start < APA_SHORT) {}
                start = esp_cpu_get_cycle_count();
                dedic_gpio_bundle_write(apa_handle, 0x01, 0);
                while (esp_cpu_get_cycle_count() - start < APA_LONG) {}
            }
            rgb >>= 1;
        }
#endif
        taskEXIT_CRITICAL(&mux);
    }
    return result;
}
