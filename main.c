#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lvgl.h"

#define TAG "ST7701S_SPI_LCD"

// SPI pins
#define SPI_MOSI 1
#define SPI_SCLK 2
#define SPI_CS   42
#define SPI_DC   3
#define SPI_RST  4

// LCD resolution
#define LCD_H_RES 480
#define LCD_V_RES 480

static spi_device_handle_t lcd_spi = NULL;

// --- Helper SPI functions ---
static void st7701s_cmd(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0
    };
    spi_device_polling_transmit(lcd_spi, &t);
}

static void st7701s_data(const uint8_t *data, int len) {
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1
    };
    spi_device_polling_transmit(lcd_spi, &t);
}

// --- Custom ST7701S initialization ---
static void st7701s_init(void) {
    gpio_set_level(SPI_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(SPI_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Command set sequence
    st7701s_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    // Command set control
    st7701s_cmd(0xFF);
    uint8_t cmdset[] = {0x77, 0x01, 0x00, 0x00, 0x10};
    st7701s_data(cmdset, 5);

    // Display line setting: 480 lines
    st7701s_cmd(0xC0);
    uint8_t c0[] = {0x3B, 0x00}; // (59+1)*8 = 480 lines
    st7701s_data(c0, 2);

    // Porch
    st7701s_cmd(0xC1);
    uint8_t c1[] = {0x0C, 0x02};
    st7701s_data(c1, 2);

    // Memory access control
    st7701s_cmd(0x36);
    uint8_t madctl = 0x00; // No rotation
    st7701s_data(&madctl, 1);

    // Pixel format (RGB565)
    st7701s_cmd(0x3A);
    uint8_t pf = 0x55;
    st7701s_data(&pf, 1);

    // Display on
    st7701s_cmd(0x29);
}

// --- SPI initialization ---
static void spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS,
        .queue_size = 7,
        .pre_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &lcd_spi));

    gpio_set_direction(SPI_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_RST, GPIO_MODE_OUTPUT);
}

// --- LVGL flush callback ---
static void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_map) {
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    // Set column
    uint8_t col_addr[] = {
        (area->x1 >> 8) & 0xFF,
        area->x1 & 0xFF,
        (area->x2 >> 8) & 0xFF,
        area->x2 & 0xFF
    };
    st7701s_cmd(0x2A); // Column addr
    st7701s_data(col_addr, 4);

    // Set row
    uint8_t row_addr[] = {
        (area->y1 >> 8) & 0xFF,
        area->y1 & 0xFF,
        (area->y2 >> 8) & 0xFF,
        area->y2 & 0xFF
    };
    st7701s_cmd(0x2B); // Row addr
    st7701s_data(row_addr, 4);

    // Write RAM
    st7701s_cmd(0x2C);
    st7701s_data((uint8_t*)color_map, w * h * 2);

    lv_disp_flush_ready(disp);
}

// --- LVGL tick timer ---
static void lv_tick_task(void *arg) {
    while (1) {
        lv_tick_inc(2);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main(void) {
    spi_init();
    st7701s_init();

    // Initialize LVGL
    lv_init();
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[LCD_H_RES * 40]; // 40 rows per buffer
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_H_RES * 40);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // LVGL tick task
    xTaskCreate(lv_tick_task, "lv_tick", 1024 * 2, NULL, 5, NULL);

    // Demo: draw a red rectangle
    lv_obj_t *rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect, 480, 480);
    lv_obj_set_style_bg_color(rect, lv_color_hex(0xFF0000), 0);

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
