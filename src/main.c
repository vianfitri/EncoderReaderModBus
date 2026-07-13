#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
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
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_TIM5);
}
