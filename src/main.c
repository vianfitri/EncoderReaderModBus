#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846f
#define ENCODER_PPR 1024
#define TOTAL_PPR (ENCODER_PPR * 4)

// Alamat Flash Emulasi EEPROM (Page 127 - 1KB terakhir)
#define FLASH_CONFIG_ADDR 0x0801FC00

// Struktur Data Konfigurasi
typedef struct {
    uint16_t slave_id;
    uint16_t baudrate_code;
    float wheel_diameter;
    uint16_t checksum;    
} DeviceConfig;

DeviceConfig current_config;

// Modbus Register
typedef struct {
    int32_t pulse_count;
    float distance;
    uint16_t homing_status; // 0 = belum, 1 = siap/sudah zero
    int16_t total_laps; // jumlah putaran penuh dari Channel Z
} LiveData;

LiveData live_data;

// Modbus Buffer
uint8_t modbus_rx_buf[64];
uint8_t modbus_tx_buf[64];
uint8_t modbus_rx_idx = 0;

void clock_setuo(void) {
    // API clock 72 MHz
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    // Enable Clock Peripheral
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_TIM5);
}

void gpio_setup(void) {
    // RS-485 TX (PA9) Alternate Function Push-Pull
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    // RS-485 RX (PA10) Floating Input
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    // RS-485 DE/RE Direction Pin (PA8)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_clear(GPIOA, GPIO8); // Default RX Mode for DE/RE

    // Encoder Pins (PA0 dan PA1) input pull-up
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0 | GPIO1);
    gpio_set(GPIOA, GPIO0 | GPIO1); // Reactivate internal Pull-Up if bad external resistor

    // Pin PB0 for Home Switch (Pull-Up for NPN)
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    gpio_set(GPIOB, GPIO0); // Pull-Up active
}
