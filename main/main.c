#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

// LVGL+OLED includes
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "esp_lcd_panel_vendor.h"


#define GPIO_INPUT_BUTTON_0 1
#define GPIO_INPUT_BUTTON_1 2
#define GPIO_INPUT_BUTTON_2 41
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_BUTTON_0) | (1ULL<<GPIO_INPUT_BUTTON_1) | (1ULL<<GPIO_INPUT_BUTTON_2)) 

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EXAMPLE_IR_TX_GPIO_NUM))

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           3
#define EXAMPLE_PIN_NUM_SCL           4
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8


extern void example_lvgl_demo_ui(lv_disp_t *disp, const char *text);

static const char *TAG = "main";


static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));

            // if(io_num == GPIO_INPUT_BUTTON_0) {
            //     printf("VOLUME UP\n");
            //     rmt_transmit(tx_channel, nec_encoder, &scan_code_volup, sizeof(scan_code_volup), &transmit_config);
            // }

            // if(gpio_get_level(GPIO_INPUT_BUTTON_1)) {
            //     printf("VOLUME DOWN\n");
            //     ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code_voldown, sizeof(scan_code_voldown), &transmit_config));
            // }

            // if(gpio_get_level(GPIO_INPUT_BUTTON_2)) {
            //     printf("POWER\n");
            //     ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code_power, sizeof(scan_code_power), &transmit_config));
            // }
        }
    }
}


void app_main(void) {

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //enable pull-down
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;

    gpio_config(&io_conf);

    // //interrupt of rising edge
    // io_conf.intr_type = GPIO_INTR_POSEDGE;
    // //io_conf.intr_type = GPIO_INTR_DISABLE;
    // //bit mask of the pins, use GPIO4/5 hereP
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //set as input mode
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // //disable pull-down mode
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // //enable pull-up mode
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_0, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_1, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_1);
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_2, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_2);


    // Screen Setup
    // -------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_HOST,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0))
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
#endif
    };

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,3,0))
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
#endif
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_I1,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = false,
#endif
            .sw_rotate = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {
        /* Rotation of the screen */
        lv_disp_set_rotation(disp, LV_DISPLAY_ROTATION_270);

        example_lvgl_demo_ui(disp, "Enter Command");
        // Release the mutex
        lvgl_port_unlock();
    }



    while(1) {
        
    }
}