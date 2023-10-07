#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include "reg_config.h"
#include <string>

// LED indicator
bool led_status = false;
bool stop_flash = false;
bool on_led_timeout(repeating_timer_t* rt);

// sht30
float sht30_data[2]; // temperature + humidity
void sht30_init();
void sht30_read_data(float* data);
void sht30_read_status();

// bmp280
float bmp_data[2];   // temperature + pressure
void bmp280_init();
void bmp280_read_raw(int32_t* temp, int32_t* pressure);
void bmp280_reset();
void bmp280_get_calib_params(bmp280_calib_param* params);
// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bmp280_convert(int32_t temp, bmp280_calib_param* params);
int32_t bmp280_convert_temp(int32_t temp, bmp280_calib_param* params);
int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, bmp280_calib_param* params);
void bmp280_read_data(int32_t* temp, int32_t* pressure, float* data, bmp280_calib_param* params);


// esp8266
char rx_buffer[UART_RX_BUFFER_LENGTH];
uint32_t buffer_length = 0;
void esp8266_init_mqtt();
void esp8266_init_direct_transfer();
void esp8266_topic_subscribe();
void esp8266_on_uart_rx();
void esp8266_handle_recv_mqtt_msg(char* buffer);


struct sensor_data
{
    // SHT30 DATA
    float sht_temp = 0;
    float sht_rh = 0;

    // BMP280 DATA
    float bmp_temp = 0;
    float bmp_pressure = 0;

    // soil sensor
    float soil_value = 0;
};


int main()
{
    stdio_init_all();
    sleep_ms(10000);
    sensor_data sensor;

    // flash led to indicate mcu status
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    repeating_timer led_timer;
    add_repeating_timer_ms(1000, &on_led_timeout, nullptr, &led_timer);
     

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);  // 设置波特率
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // init sht30
    sht30_init();
    sleep_ms(1000);

    // init bmp280
    bmp280_init();
    bmp280_calib_param params;
    bmp280_get_calib_params(&params);
    int32_t raw_temperature;
    int32_t raw_pressure;

    // init uart
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(100);

    esp8266_init_mqtt();
    esp8266_topic_subscribe();
    // add uart rx interrupt
    irq_set_exclusive_handler(UART0_IRQ, esp8266_on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);


    // init pump
    gpio_set_function(PUMP1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PUMP2_PIN, GPIO_FUNC_PWM);
    uint slice_num_1 = pwm_gpio_to_slice_num(PUMP1_PIN);
    uint slice_num_2 = pwm_gpio_to_slice_num(PUMP2_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    pwm_config_set_clkdiv(&config, 10);
    pwm_set_gpio_level(PUMP1_PIN, 0);
    pwm_set_gpio_level(PUMP2_PIN, 0);
    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);


    
    while (true)
    {
        sht30_read_data(sht30_data);
        bmp280_read_data(&raw_temperature, &raw_pressure, bmp_data, &params);
        
        sensor.sht_temp = sht30_data[0];
        sensor.sht_rh = sht30_data[1];
        sensor.bmp_temp = bmp_data[0];
        sensor.bmp_pressure = bmp_data[1];

        printf("AT+MQTTPUB=0,\"water_station/info\",\"{\\\"sht_temp\\\":%.2f\\,\\\"sht_rh\\\":%.2f\\,\\\"bmp_temp\\\":%.2f\\,\\\"bmp_pressure\\\":%.3f}\",1,0\r\n", 
                sensor.sht_temp, sensor.sht_rh, sensor.bmp_temp, sensor.bmp_pressure);
        
        sleep_ms(2000);

    }
    return 0;
}


void sht30_init()
{
    uint8_t buf[2];

    buf[0] = 0x21;
    buf[1] = 0x30;

    // start Periodic Mode
    i2c_write_blocking(i2c_default, ADDR_SHT30, buf, 2, false);
}

void sht30_read_data(float* data)
{
    uint8_t buf[2];
    uint8_t buf2[6];

    buf[0] = 0xE0;
    buf[1] = 0x00;

    i2c_write_blocking(i2c_default, ADDR_SHT30, buf, 2, true);  // fetch data
    i2c_read_blocking(i2c_default, ADDR_SHT30, buf2, 6, false); // data format: temp msb, temp lsb, crc, hum msb, hum lsb, crc 

    float rh = 100 * ( ((buf2[3] << 8) | buf2[4] ) / 65535.0);
    float temp = 175 * ( ((buf2[0] << 8) | buf2[1] ) / 65535.0) - 45;

    data[0] = temp;
    data[1] = rh;
    // printf("temp = %.2f℃, rh = %.2f%\n", t, rh);
}


void bmp280_init()
{
    // 使用手持设备动态优化的设置
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];


    // 设置采样时间为500ms
    // 500ms sampling time, x16 filter，这个在bosch的手册中可以找到
    // 地址为0xf5的config寄存器, 我们现在要用的有效bit位是bit7-bit2，所有使用1111 1100
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;  // 100 101 00 & 1111 1100, 100表示500ms, 为什么使用0x05不知道???

    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    // 直接调用函数就可以写入，阻塞
    // 写入的buf内容都是(寄存器地址+data)?
    i2c_write_blocking(i2c_default, ADDR_BMP, buf, 2, false);

    // 设置控制测量，oversampling，采样过密
    // osrs_t control oversampling of temperature data, osrs_4 control oversampling of pressure data
    // osrs_t x1, osrs_p x4, normal mode operation(power mode)
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);

    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(i2c_default, ADDR_BMP, buf, 2, false);
}


void bmp280_read_raw(int32_t* temp, int32_t* pressure)
{
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes
    // 0xfc是 temp_xlsb, 0xf7 press_msb, 这之间一共有六个数，我们可以一次性读取
    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    
    // 读取寄存器时，需要写入起始读取的寄存器
    i2c_write_blocking(i2c_default, ADDR_BMP, &reg, 1, true);  // true to keep master control of bus，表示从0xf7开始，读6位数据，一直到0xfc
    i2c_read_blocking(i2c_default, ADDR_BMP, buf, 6, false);   // false - finished with bus

    // store the 20 bit read in a 32 bit signed integer for conversion
    // 原始数据一共20位，msb [19:12], lsb [11:4], xlsb [3:0]
    // _msb, _lsb, xlsb
    // msb: buf[0], lsb: buf[1], xlsb: buf[2]
    // msb本身是一个8bit数，因此只需要移动12位，就是20位了
    // 1111 1111 1111 1111 1111
    // msb       lsb       xlsb
    // xlsb是高位含有数据，因此使用右移动，参见memory mapping图
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}


void bmp280_reset()
{
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c_default, ADDR_BMP, buf, 2, false);
}


void bmp280_get_calib_params(bmp280_calib_param* params)
{
    // 读取的时候都使用uint，然后再进行转化
    uint8_t buf[NUM_CALIB_PARAMS];
    uint8_t reg = REG_DIG_T1_LSB;

    i2c_write_blocking(i2c_default, ADDR_BMP, &reg, 1, true);
    i2c_read_blocking(i2c_default, ADDR_BMP, buf, NUM_CALIB_PARAMS, false);

    // LSB + MSB，因此buf[1]是高位，获得的数据是16位
    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}


int32_t bmp280_convert(int32_t temp, bmp280_calib_param* params)
{
    // 使用博世给的代码进行校准
    // 应该要先读取校准数据，然后校准温度
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}


int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) 
{
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = bmp280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}


int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params) 
{
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}


void bmp280_read_data(int32_t* raw_temp, int32_t* raw_pressure, float* data, bmp280_calib_param* params)
{
    bmp280_read_raw(raw_temp, raw_pressure);
    int32_t temperature = bmp280_convert_temp(*raw_temp, params);
    int32_t pressure = bmp280_convert_pressure(*raw_pressure, *raw_temp, params);
    data[0] = temperature / 100.f; // unit ℃
    data[1] = pressure / 1000.f;   // unit kpa
}


void esp8266_init_mqtt()
{
    uart_puts(UART_ID, "AT+MQTTUSERCFG=0,1,\"clientID\",\"username\",\"password\",0,0,\"\"\r\n");
    sleep_ms(1500);
    uart_puts(UART_ID, "AT+MQTTCONN=0,\"192.168.31.7\",1883,0\r\n");
    sleep_ms(1500);
}


void esp8266_topic_subscribe()
{
    uart_puts(UART_ID, "AT+MQTTSUB=0,\"water_station/pump_cmd\",1\r\n");
    sleep_ms(1500);
}


void esp8266_init_direct_transfer()
{
    // 连接到服务器
    uart_puts(UART_ID, "AT+CIPSTART=\"TCP\",\"192.168.31.17\",12800\r\n");
    sleep_ms(1500);
    uart_puts(UART_ID, "AT+CIPSTART=\"TCP\",\"192.168.31.17\",12800\r\n");
    sleep_ms(1500);
    // 开启透传
    uart_puts(UART_ID, "AT+CIPMODE=1\r\n");
    sleep_ms(500);
    uart_puts(UART_ID, "AT+CIPMODE=1\r\n");
    sleep_ms(500);
    uart_puts(UART_ID, "AT+CIPSEND\r\n");
    sleep_ms(500);
}


void esp8266_on_uart_rx()
{
    // 1. read whole data from RX check if data come from topic subscribe
    while (uart_is_readable(UART_ID)) 
    {
        // read useless data
        char tempc = uart_getc(UART_ID);
        // found maybe useful data, break
        if (tempc == '+')
        {
            break;
        }
    }

    while (uart_is_readable_within_us(UART_ID, 100) && 
           buffer_length < UART_RX_BUFFER_LENGTH)
    {
        rx_buffer[buffer_length++] = uart_getc(UART_ID);
        // printf("%c", rx_buffer[buffer_length - 1]);
    }

    // 1. read whole data from RX another possible way to read whole data(wrong)
    // add data has a /n in the end 
    // do
    // {
    //     if (uart_is_readable(UART_ID))
    //     {
    //         rx_buffer[buffer_length++] = uart_getc(UART_ID);
    //         printf("%c", rx_buffer[buffer_length--]);
    //     }
    // } while (rx_buffer[buffer_length - 1] != '\n' && buffer_length < UART_RX_BUFFER_LENGTH);
    

    // MQTT recv msg: MQTTSUBRECV:0,"water_station/pump_cmd",6,114000\n
    // first 1 = PUMP_INDEX
    // second 1 = PUMP ON/OFF
    // 4000 is pwm wrap value
    if (buffer_length == 49 && rx_buffer[39] == '6')
    {
        // TODO data received
        stop_flash = !stop_flash;
        esp8266_handle_recv_mqtt_msg(rx_buffer);
    }
    buffer_length = 0;
}


// MQTT recv msg: MQTTSUBRECV:0,"water_station/pump_cmd",6,114000\n
void esp8266_handle_recv_mqtt_msg(char* buffer)
{
    uint32_t pump_index = (buffer[41] - '0') + PUMP_OFFSET;
    bool pump_status = buffer[42] - '0';

    pwm_set_enabled(pump_index, pump_status);
    if (pump_status)
    {    
        pwm_set_gpio_level(pump_index, 2000);
    } else 
    {
        pwm_set_gpio_level(pump_index, 0);  
    }
    
    printf("AT+MQTTPUB=0,\"water_station/pump_info\",\"{\\\"pump_index\\\":%d\\,\\\"pump_status\\\":%d}\",1,0\r\n", 
                (pump_index - PUMP_OFFSET), pump_status);   
}


bool on_led_timeout(repeating_timer_t* rt)
{
    if (!stop_flash)
    {
        led_status = !led_status;
        gpio_put(PICO_DEFAULT_LED_PIN, led_status);
    }
    return true;
}