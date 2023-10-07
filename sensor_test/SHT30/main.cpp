#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>

#define ADDR _u(0x44)

void sht30_init();
void sht30_read_raw();
void sht30_read_status();

int main()
{
    stdio_init_all();

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);  // 设置波特率
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sht30_init();
    sleep_ms(1000);

    while (true)
    {
        sht30_read_raw();
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
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

    // i2c_read_blocking(i2c_default, ADDR, buf, 6, false);
}


void sht30_read_raw()
{
    uint8_t buf[2];
    uint8_t buf2[6];

    buf[0] = 0xE0;
    buf[1] = 0x00;

    // read data
    i2c_write_blocking(i2c_default, ADDR, buf, 2, true);
    i2c_read_blocking(i2c_default, ADDR, buf2, 6, false);

    double rh = 100 * ( ((buf2[3] << 8) | buf2[4] ) / 65535.0);
    double t = 175 * ( ((buf2[0] << 8) | buf2[1] ) / 65535.0) - 45;

    printf("temp = %.2f℃, rh = %.2f%\n", t, rh);
}

void sht30_read_status()
{

}


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "pico/binary_info.h"
// #include "hardware/i2c.h"

// using byte = uint8_t;
// static const byte address = 0x44;
// static const byte command[2] = {0X2C,0X06};

// int main()
// {
// 	stdio_init_all();
	
// 	// baud rate 100kHz
// 	i2c_init(i2c_default, 100 * 1000);
	
// 	// default SDA and SCL pins (GP4, GP5 on a Pico)
//     gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    
//     gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
//     gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
//     // Make the I2C pins available to picotool
//     bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

// 	while(true)
// 	{
// 		//std::cout << "Sensor test" << "\n";
		
// 		// write 2 byte command
// 		int ret1 = i2c_write_blocking(i2c_default, address, command, 2, false);
// 		//std::cout << "Write returned " << ret1 << "\n";
		
// 		// reply is two (word plus crc)
// 		byte val[6];		
// 		int ret2 = i2c_read_blocking(i2c_default, address, val, 6, false);		
// 		//std::cout << "Read returned " << ret2 << "\n";

// 		// if all good get values
// 		if((ret1 == 2) && (ret2 = 6))
// 		{
// 			float temp_c = -45 + (175 * (val[0] << 8 | val[1])/ 65535.0);
// 			float hum = 100 * (val[3] << 8 | val[4]) / 65535.0;

// 			//std::cout << "Temperature " << temp_c << " Humidity " << hum << "\n";
//             printf("temp is %.2f, hum is %.2f\n", temp_c, hum);
            
// 		}
		
// 		sleep_ms(3000);
// 	}
// }