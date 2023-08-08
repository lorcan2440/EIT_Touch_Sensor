#ifndef EIT_H
#define EIT_H

#include <Arduino.h>
#include <vector>

#define SPI_FREQ_FAST      4000000UL
#define SPI_FREQ_SLOW      500000UL
#define HSPI_MOSI_PIN      26
#define HSPI_SCK_PIN       27
#define VSPI_MOSI_PIN      11
#define VSPI_SCK_PIN       13

#define MUX_EN             1
#define MUX_DIS            0
#define NUM_ELECTRODES     32
#define NUM_MEAS           NUM_ELECTRODES*NUM_ELECTRODES  // 1024

#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          50000
#define NUM_PERIODS        4        // 4: Number of signal periods to measure
#define ADC_AVG            5        // 5: Number of ADC samples to average for each analog reading

// AD5270 commands
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

// AD55930 register addresses
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D

#define CHIP_SEL_AD5930    3  // Chip select pin for AD5930
#define CHIP_SEL_DRIVE     0  // Chip select pin for driving digital rheostat
#define CHIP_SEL_MEAS      1 // Chip select pin for measuring digital rheostat
// #define CHIP_SEL_MUX_SRC   24 // Chip select pin for source electrodes MUX - mux1
// #define CHIP_SEL_MUX_SINK  28 // Chip select pin for sink electrodes MUX - mux1
// #define CHIP_SEL_MUX_VP    30 // Chip select for voltage measurement positive electrodes MUX - mux1
// #define CHIP_SEL_MUX_VN    32 // Chip select for voltage measurement negative electrodes MUX - mux1

#define CHIP_SEL_MUX_SRC   29 // Chip select pin for source electrodes MUX - mux2
#define CHIP_SEL_MUX_SINK 25 // Chip select pin for sink electrodes MUX - mux2
#define CHIP_SEL_MUX_VP    33 // Chip select for voltage measurement positive electrodes MUX - mux2
#define CHIP_SEL_MUX_VN    31 // Chip select for voltage measurement negative electrodes MUX - mux2

#define AD5930_MSBOUT_PIN  6
#define AD5930_INT_PIN     5  // Pulse high to reset internal state machine
#define AD5930_CTRL_PIN    4  // Pull high to start frequency sweep. Pull low to end the burst. Pull high again to increment frequency
#define AD5930_STANDBY_PIN 2  // Pull high to power down 

#define ADS_PWR            9
#define ADS_OE             10

extern std::vector<int> values_to_send;
extern int16_t sine_table[1024];

typedef enum { AD, OP, MONO } meas_t;

extern volatile uint32_t F_CPU_ACTUAL;

// GPIO Pin to analog channel mapping from Arduino\hardware\teensy\avr\cores\teensy4\analog.c
extern const uint8_t pin_to_channel[42];

// Mapping of electrode number (input) to MUX channel (output)
//const uint8_t elec_to_mux[32] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15, 14, 13, 12, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16 };
const uint8_t elec_to_mux[32] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };
// const uint8_t elec_to_mux2[32] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

// Global calibration parameters
extern uint16_t current_gain, voltage_gain;
extern float sample_rate;
extern uint16_t samples_per_period;
extern uint16_t num_samples;
extern double ref_signal_mag;
extern double phase_offset;

extern double signal_rms[NUM_MEAS];    // Store signal RMS data
extern double signal_mag[NUM_MEAS];    // Store signal magnitude data
extern double signal_phase[NUM_MEAS];  // Store signal phase data

extern double cur_frame[NUM_MEAS];
extern uint32_t frame_delay;
extern uint32_t frame_delay_prev;

extern uint8_t pin_num;
extern uint16_t rheo_val;

void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val);
void AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data);
void AD5270_Lock(const int chip_sel, uint8_t lock);
void AD5270_Shutdown(const int chip_sel, uint8_t shutdown);
void AD5270_Set(const int chip_sel, uint16_t val);
void AD5930_Write(uint8_t reg, uint16_t data);
void AD5930_Set_Start_Freq(uint32_t freq);
void mux_write(const int chip_sel, uint8_t pin_sel, uint8_t enable);
uint16_t analog_read();
uint16_t gpio_read();
uint16_t gpio_convert(uint16_t gpio_reg);
uint16_t sine_compare(uint16_t * signal, uint16_t pk_pk, uint16_t points_per_period, uint8_t num_periods);
uint32_t read_signal(double * rms, double * mag, double * phase, uint16_t * error_rate, uint8_t debug);
void calibrate_samples();
void calibrate_gain(meas_t drive_type, meas_t meas_type);
void calibrate_signal(meas_t drive_type, meas_t meas_type);
void read_frame(meas_t drive_type, meas_t meas_type, double * rms_array, double * mag_array, double * phase_array, uint8_t num_elec);

#endif  // EIT_H