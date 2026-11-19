
#include "input.h"

#include "driver/gpio.h"


#define GPIO_INPUT_BUTTON_0 1
#define GPIO_INPUT_BUTTON_1 2
#define GPIO_INPUT_BUTTON_2 41
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_BUTTON_0) | (1ULL<<GPIO_INPUT_BUTTON_1) | (1ULL<<GPIO_INPUT_BUTTON_2)) 

#define ESP_INTR_FLAG_DEFAULT 0


static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void* arg) {
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

void input_init(void) {
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
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_0, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_1, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_1);
    gpio_isr_handler_add(GPIO_INPUT_BUTTON_2, gpio_isr_handler, (void*) GPIO_INPUT_BUTTON_2);
}
