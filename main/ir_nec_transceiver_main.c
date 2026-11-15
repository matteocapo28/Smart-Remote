/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"

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

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       18
#define EXAMPLE_IR_RX_GPIO_NUM       19
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EXAMPLE_IR_TX_GPIO_NUM))

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250


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

static const char *TAG = "example";

/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
} 

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    printf("NEC frame start---\r\n");
    for (size_t i = 0; i < symbol_num; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols)) {
            printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols)) {
            printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}


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


void app_main(void)
{
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



    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };

    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    // this example won't send NEC frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = EXAMPLE_IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));


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


    // timeout, transmit predefined IR NEC packets

    // LG TV
    const ir_nec_scan_code_t scan_code_volup = {
        .address = 0xFB04,
        .command = 0xFD02,
    };

    /* Roku 
        Up
        address: EA C2 00 00
        command: 19 E6 00 00

        name: Back
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 66 99 00 00
        # 
        name: Home
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 03 FC 00 00
        # 
        name: Up
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 19 E6 00 00
        # 
        name: Down
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 33 CC 00 00
        # 
        name: Left
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 1E E1 00 00
        # 
        name: Right
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 2D D2 00 00
        # 
        name: Ok
        type: parsed
        protocol: NECext
        address: EA C2 00 00
        command: 2A D5 00 00

    */

    const ir_nec_scan_code_t scan_code_voldown = {
        .address = 0xFB04,
        .command = 0xFC03,
    };

    const ir_nec_scan_code_t scan_code_ok = {
        .address = 0xC2EA,
        .command = 0xD52A,
    };

     const ir_nec_scan_code_t scan_code_power = {
        .address = 0xFB04,
        .command = 0xF708,
    };

    //gpio_set_level(EXAMPLE_IR_TX_GPIO_NUM, 1);
    
    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    while (1) {
        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(100)) == pdPASS) {
            // parse the receive symbols and print the result
            example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }

        if(gpio_get_level(GPIO_INPUT_BUTTON_0)) {
            printf("VOLUME UP\n");
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code_volup, sizeof(scan_code_volup), &transmit_config));

            if (lvgl_port_lock(0)) {
                lv_obj_clean(lv_screen_active());
                example_lvgl_demo_ui(disp, "VOLUME UP");
                lvgl_port_unlock();
            }
        }

        if(gpio_get_level(GPIO_INPUT_BUTTON_1)) {
            printf("VOLUME DOWN\n");
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code_voldown, sizeof(scan_code_voldown), &transmit_config));
        
            if (lvgl_port_lock(0)) {
                lv_obj_clean(lv_screen_active());
                example_lvgl_demo_ui(disp, "VOLUME DOWN");
                lvgl_port_unlock();
            }
        }

        if(gpio_get_level(GPIO_INPUT_BUTTON_2)) {
            printf("OK\n");
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code_ok, sizeof(scan_code_ok), &transmit_config));
        
            if (lvgl_port_lock(0)) {
                lv_obj_clean(lv_screen_active());
                example_lvgl_demo_ui(disp, "OK");
                lvgl_port_unlock();
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
