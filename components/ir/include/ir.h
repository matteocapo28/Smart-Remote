

#define IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define IR_TX_GPIO_NUM       18
#define IR_RX_GPIO_NUM       19

#define IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

void ir_init(void);
void ir_send();
void ir_receive();
