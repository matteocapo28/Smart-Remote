
#include "input.h"


static const char *TAG = "main";


void app_main(void) {
    input_init();

    while(1) {
        printf("Main Running\n\r");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}