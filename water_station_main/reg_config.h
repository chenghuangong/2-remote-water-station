// data send out threshold
// ℃
#define SHT30_TEMP_THRESHOLD 0.2
// %
#define SHT30_RH_THRESHOLD 0.3
// ℃
#define BMP280_TEMP_THRESHOLD 0.3
// kpa
#define BMP280_PRESSURE_THRESHOLD 0.005

// ==============================================UART==============================================
#define UART_BAUD_RATE 115200
#define UART_ID uart0
#define UART_RX_BUFFER_LENGTH 128

// ==============================================PUMP PWM==============================================

#define PUMP1_PIN 10
#define PUMP2_PIN 11
#define PWM_WRAP_VALUE 4000
#define PWM_OUTPUT 2000
#define PUMP_OFFSET (PUMP1_PIN - 1)

// ==============================================SHT30==============================================

#define ADDR_SHT30 _u(0x44)

// ==============================================BMP280==============================================
// 并且需要将SDO连接到GND上，这样地址就是0x76
// device has default bus address of 0x76
#define ADDR_BMP _u(0x76)

// 硬件的寄存器地址，我们就从这些寄存器中读取数据
// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// 校准寄存器的地址
// calibration registers
// LSB 和 MSB表示两种BIT到达的顺序
// 例如0x12h, 00010010b，用MSB表示时，到达的顺序是00010010b，用LSB表示时，到达的顺序是01001000b

// 注意下面的LSB和MSB仅仅表示数据的大头和小头部分，而不是数据的到达序列！！！
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24



// 数据需要进行校正
// 定义数据的结构体
struct bmp280_calib_param 
{
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};