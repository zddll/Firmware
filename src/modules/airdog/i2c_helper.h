#ifndef I2C_HELPER_HEADER_H
#define I2C_HELPER_HEADER_H

#define DEVICE_FREQUENCY 100000

typedef enum {
    SYMBOL_0 = 0xFC,
    SYMBOL_1 = 0x60,
    SYMBOL_2 = 0xDA,
    SYMBOL_3 = 0xF2,
    SYMBOL_4 = 0x66,
    SYMBOL_5 = 0xB6,
    SYMBOL_6 = 0xBE,
    SYMBOL_7 = 0xE0,
    SYMBOL_8 = 0xFF,
    SYMBOL_9 = 0xF6,
    SYMBOL_A = 0xEE,
    SYMBOL_U = 0x7C,
    SYMBOL_Y = 0x76,
    SYMBOL_EMPTY = 0x00
} symbol_t;

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif
    void start_listener(int adress);
    void stop_listener(void);
    void set_symbols(symbol_t first, symbol_t second, symbol_t third);
    void set_green_led_on(bool set_on);
    void set_red_led_on(bool set_on);
#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif
#endif
