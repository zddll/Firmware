#ifndef I2C_HELPER_HEADER_H
#define I2C_HELPER_HEADER_H

typedef enum {
   LED_STATE_BOTH_OFF = 0xFF,
   LED_STATE_GREEN_ON = 0xBF,
   LED_STATE_RED_ON = 0x7F,
   LED_STATE_BOTH_ON = 0x3F
} led_state_t;

#endif
