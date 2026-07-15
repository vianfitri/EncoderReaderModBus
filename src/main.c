#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>
#include <math.h>
#include <stdio.h> // untuk sprintf ke UARTS

#define PI 3.14159265358979323846f

// Alamat Flash Emulasi EEPROM (Page 127 - 1KB terakhir)
#define FLASH_CONFIG_ADDR 0x0801FC00

// Struktur Data Konfigurasi
typedef struct {
    uint16_t slave_id;
    uint16_t baudrate_code;
    float wheel_diameter;
    uint16_t encoder_ppr;
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


// --------------------------------------
// UJI STREAMING MELALUI UART
// --------------------------------------

// variable untuk melacak waktu SysTick
volatile uint32_t system_millis = 0;

// SysTick Interrupt Handler (dipanggil setiap 1 ms)
void sys_tick_handler(void) {
    system_millis++;
}

// Inisialisasi SysTick untuk interrupt 1ms pada 72 MHz
void systick_setup(void) {
    systick_set_frequency(1000, 72000000); // 1000Hz (1ms) pada clock 72MHz
    systick_interrupt_enable();
    systick_counter_enable();
}

// --------------------------------------

void clock_setup(void) {
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

    // RS-485 DE/RE Direction Pin (PA8) mirror to Pin (PB15)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
    //gpio_clear(GPIOA, GPIO8); // Default RX Mode for DE/RE
    //gpio_clear(GPIOB, GPIO15);
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO15);

    // Encoder Pins (PA0 dan PA1) input pull-up
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0 | GPIO1);
    gpio_set(GPIOA, GPIO0 | GPIO1); // Reactivate internal Pull-Up if bad external resistor

    // Pin PB0 for Home Switch (Pull-Up for NPN)
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    gpio_set(GPIOB, GPIO0); // Pull-Up active
}

// External Interrupt Setup for HOME Switch
void exti_setup(void){
    exti_select_source(EXTI0, GPIOB); // Map EXTI0 to Port B

    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); // Set trigger pada Falling Edge (Transisi dari High ke Low saat Sensor NPN aktif)

    exti_enable_request(EXTI0); // Aktifkan interupsi pada lini EXTI0

    // Aktifkan di NVIC (Core CPU) dan set prioritas
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_set_priority(NVIC_EXTI0_IRQ, 1); // prioritas tinggi untuk akurasi posisi
}

// Interrupt Service Routine (ISR) for EXTI0
void exti0_isr(void) {
    // Validate interrupt only from EXTI0
    if (exti_get_flag_status(EXTI0)) {

        // Cek arah putaran dari register TIM5 CR1 DIR bit (0 = Upcount/Maju, 1 = Downcount/Mundur)
        if ((TIM_CR1(TIM5) & TIM_CR1_DIR_DOWN) == 0) {
            live_data.total_laps++;
        } else {
            live_data.total_laps--;
        }

        // Reset Pulse Count dan Hardware Timer ke posisi nol (Homing)
        live_data.pulse_count = 0;
        timer_set_counter(TIM5, 0);

        // Tandai status homing sudah sukses
        live_data.homing_status = 1;

        // Clear flag interrupsi agar tidak looping berkelanjutan
        exti_reset_request(EXTI0);
    }
}

void encoder_setup(void){
    // reset timer
    timer_set_counter(TIM5, 0);

    // konfigurasi periode maksimal (32-bit/16-bit bertindak sebagai auto-reload)
    timer_set_period(TIM5, 0xFFFF);

    // Konfigurasi Encoder Mode X4 (membaca transisi kedua channel)
    timer_slave_set_mode(TIM5, TIM_SMCR_SMS_EM3);

    // setup input capture untuk membedakan arah
    timer_ic_set_input(TIM5, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM5, TIM_IC2, TIM_IC_IN_TI2);

    // Menyalakan Timer
    timer_enable_counter(TIM5);
}

void usart_setup(uint32_t baudrate) {
    usart_set_baudrate(USART1, baudrate);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);

    //usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_mode(USART1, USART_MODE_TX);

    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

// Fungsi kirim string teks lewat USART
void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}

// Fungsi untuk kirim data encoder dalam format Teks CSV
void stream_encoder_data(void) {
    char tx_buffer[100];

    // Format data menjadi string: "PULSE,JARAK,HOMING,LAPS\r\n"
    sprintf(
        tx_buffer,
        "%ld,%.2f,%d,%d\r\n",
        live_data.pulse_count,
        live_data.distance,
        live_data.homing_status,
        live_data.total_laps
    );

    usart_send_string(tx_buffer);
}

// --- Fungsi Flash / EEPROM Emulation ---
void load_config(void) {
    DeviceConfig *flash_data = (DeviceConfig *)FLASH_CONFIG_ADDR;

    // Validasi sederhana dengan Checksum
    uint16_t calc_chk = flash_data->slave_id + flash_data->baudrate_code + (uint16_t)flash_data->wheel_diameter + flash_data->encoder_ppr;
    if (flash_data->checksum == calc_chk && flash_data->slave_id != 0xFFFF) {
        memcpy(&current_config, flash_data, sizeof(DeviceConfig));
    } else {
        // Default Config jika flash kosong/corrupt
        current_config.slave_id = 1;
        current_config.baudrate_code = 0; // 9600
        current_config.wheel_diameter = 200.0f; // 200mm
        current_config.encoder_ppr = 20;
    }
}

void save_config_to_flash(void) {
    current_config.checksum = current_config.slave_id + current_config.baudrate_code + (uint16_t)current_config.wheel_diameter + current_config.encoder_ppr;

    flash_unlock();
    flash_erase_page(FLASH_CONFIG_ADDR);

    uint16_t *src = (uint16_t *)&current_config;
    uint32_t dst = FLASH_CONFIG_ADDR;

    for (uint32_t i = 0; i < sizeof(DeviceConfig); i += 2) {
        flash_program_half_word(dst, *src);
        dst += 2;
        src++;
    }
    flash_lock();
}

// --- Modbus RTU Logic Helper ---
uint16_t modbus_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void rs485_send(uint8_t *buf, uint8_t len) {
    gpio_set(GPIOA, GPIO8); // TX Mode EN
    for (uint8_t i = 0; i < len; i++) {
        usart_send_blocking(USART1, buf[i]);
    }

    // Tunggu transmisi selesai sebelum memindahkan jalur ke RX mode
    while (!((USART_SR(USART1) & USART_SR_TC)));
    gpio_clear(GPIOA, GPIO8);

}

// --- Modbus RTU Read Update ---
void process_modbus(void) {
    if (modbus_rx_idx < 4) return;
    if (modbus_rx_buf[0] != current_config.slave_id) { modbus_rx_idx = 0; return; }

    uint16_t crc_calc = modbus_crc(modbus_rx_buf, modbus_rx_idx - 2);
    uint16_t crc_rcvd = (modbus_rx_buf[modbus_rx_idx - 1] << 8) | modbus_rx_buf[modbus_rx_idx - 2];
    if (crc_calc != crc_rcvd) { modbus_rx_idx = 0; return; }
    
    uint8_t function_code = modbus_rx_buf[1];
    uint16_t start_reg = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];
    uint16_t reg_count = (modbus_rx_buf[4] << 8) | modbus_rx_buf[5];

    if (function_code == 0x03) { // Read Holding Registers
        uint8_t byte_count = reg_count * 2;
        modbus_tx_buf[0] = current_config.slave_id;
        modbus_tx_buf[1] = 0x03;
        modbus_tx_buf[2] = byte_count;
        
        uint8_t tx_idx = 3;
        for (uint16_t i = 0; i < reg_count; i++) {
            uint16_t current_reg = start_reg + i;
            uint16_t reg_val = 0;
            
            if (current_reg == 0) reg_val = current_config.slave_id;
            else if (current_reg == 1) reg_val = current_config.baudrate_code;
            else if (current_reg == 2) reg_val = (live_data.pulse_count >> 16) & 0xFFFF;
            else if (current_reg == 3) reg_val = live_data.pulse_count & 0xFFFF;
            else if (current_reg == 4) reg_val = ((*((uint32_t*)&current_config.wheel_diameter)) >> 16) & 0xFFFF;
            else if (current_reg == 5) reg_val = (*((uint32_t*)&current_config.wheel_diameter)) & 0xFFFF;
            else if (current_reg == 6) reg_val = ((*((uint32_t*)&live_data.distance)) >> 16) & 0xFFFF;
            else if (current_reg == 7) reg_val = (*((uint32_t*)&live_data.distance)) & 0xFFFF;
            // BARU: Tambahan baca register status homing & Laps
            else if (current_reg == 9) reg_val = live_data.homing_status;
            else if (current_reg == 10) reg_val = (uint16_t)live_data.total_laps;
            else if (current_reg == 11) reg_val = current_config.encoder_ppr;
            
            modbus_tx_buf[tx_idx++] = (reg_val >> 8) & 0xFF;
            modbus_tx_buf[tx_idx++] = reg_val & 0xFF;
        }
        
        uint16_t crc = modbus_crc(modbus_tx_buf, tx_idx);
        modbus_tx_buf[tx_idx++] = crc & 0xFF;
        modbus_tx_buf[tx_idx++] = (crc >> 8) & 0xFF;
        rs485_send(modbus_tx_buf, tx_idx);
    } else if (function_code == 0x10) { // Write Multiple Registers
        uint8_t data_idx = 7;
        
        for (uint16_t i = 0; i < reg_count; i++) {
            uint16_t current_reg = start_reg + i;
            uint16_t reg_val = (modbus_rx_buf[data_idx] << 8) | modbus_rx_buf[data_idx+1];
            data_idx += 2;
            
            if (current_reg == 0) current_config.slave_id = reg_val;
            else if (current_reg == 1) current_config.baudrate_code = reg_val;
            else if (current_reg == 2) {
                live_data.pulse_count = (live_data.pulse_count & 0x0000FFFF) | ((uint32_t)reg_val << 16);
                timer_set_counter(TIM5, live_data.pulse_count & 0xFFFF);
            }
            else if (current_reg == 3) {
                live_data.pulse_count = (live_data.pulse_count & 0xFFFF0000) | reg_val;
                timer_set_counter(TIM5, live_data.pulse_count & 0xFFFF);
            }
            else if (current_reg == 4) {
                uint32_t temp = (*((uint32_t*)&current_config.wheel_diameter) & 0x0000FFFF) | ((uint32_t)reg_val << 16);
                current_config.wheel_diameter = *((float*)&temp);
            }
            else if (current_reg == 5) {
                uint32_t temp = (*((uint32_t*)&current_config.wheel_diameter) & 0xFFFF0000) | reg_val;
                current_config.wheel_diameter = *((float*)&temp);
            }
            else if (current_reg == 11) current_config.encoder_ppr = reg_val;
            else if (current_reg == 8 && reg_val == 0xAAAA) {
                save_config_to_flash(); // Simpan saat menerima command 0xAAAA
            }
        }
        
        // Response Modbus OK
        memcpy(modbus_tx_buf, modbus_rx_buf, 6);
        uint16_t crc = modbus_crc(modbus_tx_buf, 6);
        modbus_tx_buf[6] = crc & 0xFF;
        modbus_tx_buf[7] = (crc >> 8) & 0xFF;
        rs485_send(modbus_tx_buf, 8);
    }

    modbus_rx_idx = 0; // Reset buffer setelah diproses
}

int main(void) {
    clock_setup();
    gpio_setup();
    exti_setup();

    // aktifkan penjelajah waktu 1 ms
    systick_setup();

    load_config();

    // Tentukan baudrate berdasarkan data konfigurasi
    uint32_t baud = 9600;
    if (current_config.baudrate_code == 1) baud = 19200;
    else if (current_config.baudrate_code == 2) baud = 115200;

    usart_setup(baud);
    encoder_setup();

    int32_t last_timer_val = 0;
    live_data.homing_status = 0; // Default belum melewati titik nol
    live_data.total_laps = 0;

    // penampung waktu stream
    uint32_t last_stream_time = 0;

    while(1) {
        // 1. Baca Raw Encoder dari hardware timer (16-bit handling untuk akumulasi int32) 
        int16_t timer_val = (int16_t)timer_get_counter(TIM5);
        live_data.pulse_count += (timer_val - (int16_t)last_timer_val);
        last_timer_val = timer_val;

        // 2. Kalkulasi Jarak (mm) berdasarkan formula keliling roda
        uint32_t dynamic_total_ppr = current_config.encoder_ppr * 4;

        if (dynamic_total_ppr > 0) {
            live_data.distance = ((float)live_data.pulse_count / dynamic_total_ppr) * PI * current_config.wheel_diameter;
        } else {
            live_data.distance = 0.0f;
        }

        /*
        // 3. Polling Data Serial Modbus (Non-blocking)
        if ((USART_SR(USART1) &USART_SR_RXNE)) {
            modbus_rx_buf[modbus_rx_idx++] = usart_recv(USART1);
            if (modbus_rx_idx >= sizeof(modbus_rx_buf)) modbus_rx_idx = 0; // proteksi overflow
        }
        
        // Modbus RTU Butuh timeout deteksi end of frame.
        // Untuk penyederhanaan tanpa hardware timer tambahan, pemrosesan dieksekusi berkala
        for (volatile int i = 0; i < 2000; i++);
        if (modbus_rx_idx > 0) {
            process_modbus();
        }
        */

        // 3. Cek apakah sudah berlalu 100ms secara non-blocking
        if ((system_millis - last_stream_time) >= 100) {
            last_stream_time = system_millis;
            stream_encoder_data(); // Kirim data ke USART
        }
    }
}
