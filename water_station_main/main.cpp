#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include "reg_config.h"
#include <string>

// LED indicator
bool led_status = false;
repeating_timer led_timer;
volatile uint8_t fast_blink_counter = 0;
bool on_led_timeout(repeating_timer_t* rt);
void flash_led(uint8_t cycles);

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


// read adc value
void adc_read_data(uint16_t* data);


struct sensor_data
{
    // SHT30 DATA
    float sht_temp = 0;
    float sht_temp_arr[5];
    
    float sht_rh = 0;
    float sht_rh_arr[5];

    // BMP280 DATA
    float bmp_temp = 0;
    float bmp_temp_arr[5];

    float bmp_pressure = 0;
    float bmp_pressure_arr[5];

    // soil sensor
    uint16_t soil_value = 0;
    uint32_t soil_send_out_counter = 0; // send out data per 60s

    // all sensor use same position
    uint32_t arr_pos = 0;
    // sht_temp, sht_rh, bmp280 temp, bmp280 pressure
    float previous_arr[4] = {0,0,0,0};
    float sendout_threshold[4] = {SHT30_TEMP_THRESHOLD, SHT30_RH_THRESHOLD, BMP280_TEMP_THRESHOLD, BMP280_PRESSURE_THRESHOLD}; 
};


struct sensor_data_pointer
{
    float* sht30_raw_data;

    int32_t* raw_temperature_data;
    int32_t* raw_pressure_data;
    float* bmp_raw_data;
    bmp280_calib_param* bmp280_calib_param_data;

    sensor_data* sensors;
};


// add timer
bool on_data_acqusition_timeout(repeating_timer_t* rt);


int main()
{
    stdio_init_all();
    sleep_ms(10000);
    sensor_data sensor;

    // flash led to indicate mcu status
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    add_repeating_timer_ms(LED_BLINK_TIME_INTERVAL, &on_led_timeout, nullptr, &led_timer);
     

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


    // init adc for soil detect
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(ADC_SOIL_DETECTOR_PIN);

    // timer data structure
    sensor_data_pointer raw_sensor_data_pointer;
    raw_sensor_data_pointer.sht30_raw_data = sht30_data;
    raw_sensor_data_pointer.raw_temperature_data = &raw_temperature;
    raw_sensor_data_pointer.raw_pressure_data = &raw_pressure;
    raw_sensor_data_pointer.bmp_raw_data = bmp_data;
    raw_sensor_data_pointer.bmp280_calib_param_data = &params;
    raw_sensor_data_pointer.sensors = &sensor;

    // add a data acqusition timer
    repeating_timer data_acqusition_timer;
    add_repeating_timer_ms(GLOBAL_TIMER_INTERVAL, &on_data_acqusition_timeout, &raw_sensor_data_pointer, &data_acqusition_timer);


    while (true)
    { 
        tight_loop_contents();     
    }

    return 0;
}


void sht30_init()
{
    uint8_t buf[2];

    buf[0] = 0x21;
    buf[1] = 0x30;

    #ifdef ENABLE_SHT30_SINGLE_READ
    
    #else
    // start Periodic Mode
    i2c_write_blocking(i2c_default, ADDR_SHT30, buf, 2, false);
    #endif
}

void sht30_read_data(float* data)
{ 
    uint8_t sht30_raw_data[6];
    
    #ifdef ENABLE_SHT30_SINGLE_READ
    uint8_t command[2] = {0x2c, 0x06};
    i2c_write_blocking(i2c_default, ADDR_SHT30, command, 2, false);
    i2c_read_blocking(i2c_default, ADDR_SHT30, sht30_raw_data, 6, false);
    #else
    // Periodic Mode just read value
    uint8_t command[2] = {0xe0, 0x00};
    i2c_write_blocking(i2c_default, ADDR_SHT30, command, 2, true);            // fetch data
    i2c_read_blocking(i2c_default, ADDR_SHT30, sht30_raw_data, 6, false); // data format: temp msb, temp lsb, crc, hum msb, hum lsb, crc
    #endif

    float rh = 100 * ( ((sht30_raw_data[3] << 8) | sht30_raw_data[4] ) / 65535.0);
    float temp = 175 * ( ((sht30_raw_data[0] << 8) | sht30_raw_data[1] ) / 65535.0) - 45;
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
        // flash led
        flash_led(8);
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


void flash_led(uint8_t cycles)
{
    fast_blink_counter = cycles;
    // call on led timeout
    cancel_repeating_timer(&led_timer);
    add_repeating_timer_ms(30, &on_led_timeout, nullptr, &led_timer);
}


bool on_led_timeout(repeating_timer_t* rt)
{
    // speed up blink for 8 times
    if (fast_blink_counter != 0 && --fast_blink_counter == 0)
    {
        // delay 1000ms
        rt->delay_us = 1000 * LED_BLINK_TIME_INTERVAL;
    }
    led_status = !led_status;
    gpio_put(PICO_DEFAULT_LED_PIN, led_status);
    return true;
}


void adc_read_data(uint16_t* data)
{
    adc_select_input(0);
    *data = adc_read();
}


// data acqusition logic
// 1. get raw data every two seconds
// 2. save to the data array
// 3. every 10s compare average data with the previous data, if 
// temperature no more than 0.2℃，RH no more than 0.2%，pressure no more than 5pa，do not send out data
bool on_data_acqusition_timeout(repeating_timer_t* rt)
{
    auto raw_data_pointer = static_cast<sensor_data_pointer*>(rt->user_data);
    // start sht30 and bmp data acqusition
    sht30_read_data(raw_data_pointer->sht30_raw_data);
    bmp280_read_data(raw_data_pointer->raw_temperature_data, raw_data_pointer->raw_pressure_data, 
                     raw_data_pointer->bmp_raw_data, raw_data_pointer->bmp280_calib_param_data);
    
    raw_data_pointer->sensors->sht_temp = raw_data_pointer->sht30_raw_data[0];
    raw_data_pointer->sensors->sht_rh = raw_data_pointer->sht30_raw_data[1];
    raw_data_pointer->sensors->bmp_temp = raw_data_pointer->bmp_raw_data[0];
    raw_data_pointer->sensors->bmp_pressure = raw_data_pointer->bmp_raw_data[1];


    // ============send out temp, humidity, pressure data=====================
    // check array position, send out or not
    
    if (raw_data_pointer->sensors->arr_pos > 5)
    {

        // check send out data or not
        float sum[4] = {0,0,0,0};
        float average[4] = {0,0,0,0};
        for (size_t i = 0; i < 5; i++)
        {
            sum[0] += raw_data_pointer->sensors->sht_temp_arr[i];
            sum[1] += raw_data_pointer->sensors->sht_rh_arr[i];
            sum[2] += raw_data_pointer->sensors->bmp_temp_arr[i];
            sum[3] += raw_data_pointer->sensors->bmp_pressure_arr[i];
        }

        for (size_t i = 0; i < 4; i++)
        {
            average[i] = sum[i] / 5;
        }

        bool is_perform_send_data = false;
        for (size_t i = 0; i < 4; i++)
        {
            if (std::abs((average[i] - raw_data_pointer->sensors->previous_arr[i])) >= raw_data_pointer->sensors->sendout_threshold[i])
            {
                is_perform_send_data = true;
                // update previous data
                raw_data_pointer->sensors->previous_arr[i] = average[i];
            }
        }
 
        if (is_perform_send_data)
        {
            // send out data by MQTT
            printf("AT+MQTTPUB=0,\"water_station/info\",\"{\\\"sht_temp\\\":%.1f\\,\\\"sht_rh\\\":%.1f\\,\\\"bmp_temp\\\":%.1f\\,\\\"bmp_pressure\\\":%.3f}\",1,0\r\n", 
                raw_data_pointer->sensors->previous_arr[0], raw_data_pointer->sensors->previous_arr[1], 
                raw_data_pointer->sensors->previous_arr[2], raw_data_pointer->sensors->previous_arr[3]);
        }

        raw_data_pointer->sensors->arr_pos = 0;
    }

    // save to the array
    raw_data_pointer->sensors->sht_temp_arr[raw_data_pointer->sensors->arr_pos] = raw_data_pointer->sensors->sht_temp;
    raw_data_pointer->sensors->sht_rh_arr[raw_data_pointer->sensors->arr_pos] = raw_data_pointer->sensors->sht_rh;
    raw_data_pointer->sensors->bmp_temp_arr[raw_data_pointer->sensors->arr_pos] = raw_data_pointer->sensors->bmp_temp;
    raw_data_pointer->sensors->bmp_pressure_arr[raw_data_pointer->sensors->arr_pos] = raw_data_pointer->sensors->bmp_pressure;
    
    raw_data_pointer->sensors->arr_pos++;

    // ============send out temp, humidity, pressure data=====================


    // =========================get and out soil data============================
    // add delay time: 30 * timer interval
    if (raw_data_pointer->sensors->soil_send_out_counter++ > SOIL_DATA_DELAY_CYCLES)
    {
        raw_data_pointer->sensors->soil_send_out_counter = 0;
        adc_read_data(&(raw_data_pointer->sensors->soil_value));
        printf("AT+MQTTPUB=0,\"water_station/info/soil\",\"{\\\"soil1\\\":%d}\",1,0\r\n", raw_data_pointer->sensors->soil_value);
    }
    
    return true;
}