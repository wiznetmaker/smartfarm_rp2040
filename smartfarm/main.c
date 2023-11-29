/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "port_common.h"
#include "hardware/adc.h"
#include <dht.h>
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"
#include "loopback.h"
#include "hardware/pwm.h"

// extern uint16_t ai_temp;
// extern uint16_t ai_humid;
// extern uint16_t ai_brightness;

uint16_t ai_temp_main;
uint16_t ai_humid_main;
uint16_t ai_brightness_main;
uint16_t ai_water_flag_main;
void set_servo_angle(float angle, int servo_pin);
/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
#define PLL_SYS_KHZ (133 * 1000)
 
/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_LOOPBACK 0

/* Port */
#define PORT_LOOPBACK 5000

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x1D, 0x6B, 0x52}, // MAC address
        .ip = {192, 168, 0, 11},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 0, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

/* Loopback */
static uint8_t g_loopback_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
extern uint8_t ai_buf[2048];



static const dht_model_t DHT_MODEL = DHT22;
static const uint DATA_PIN = 15;
static float celsius_to_fahrenheit(float temperature) {
    return temperature * (9.0f / 5) + 32;
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);
void set_gpio();
/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */



int main()
{
    /* Initialize */
    int retval = 0;
    int buf_len = 0;
    set_clock_khz();
    
    stdio_init_all();
    set_gpio();
    adc_init();
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);


    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, DATA_PIN, true /* pull_up */);
    printf(" DHT TEST SIGNAL\n");



    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);

    /* Infinite loop */
    while(1){
  
       dht_start_measurement(&dht);

        float humidity;
        float temperature_c;
        uint16_t adc_result = adc_read();
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c); 
        if (result == DHT_RESULT_OK) {
            // printf("%.1f C (%.1f F), %.1f%% humidity\n", temperature_c, celsius_to_fahrenheit(temperature_c), humidity);
          //  printf("adc value = %d\n Temp = %.1f \n humi = %.1f\n", adc_result, temperature_c, humidity);
        } 
        
        else if (result == DHT_RESULT_TIMEOUT) {
            
        } 
        else {
            assert(result == DHT_RESULT_BAD_CHECKSUM);
         //   puts("Bad checksum");
        }
        sleep_ms(500);
        
                /* TCP server loopback test */
        sprintf(g_loopback_buf, "temp : %.1f, humi : %.1f, adc_result : %d\n", temperature_c, humidity, adc_result);
        
    // printf("Main Temperature: %d\n", ai_temp);
    // printf("Main Humidity: %d\n", ai_humid);
    // printf("Main Brightness: %d\n", ai_brightness);
        if ((retval = tcps(SOCKET_LOOPBACK, g_loopback_buf, PORT_LOOPBACK, &ai_temp_main, &ai_humid_main, &ai_brightness_main, &ai_water_flag_main)) < 0)
        {
            printf(" Loopback error : %d\n", retval);

            while (1)
                ;
        }


        // 출력
        printf("Main Temperature: %d\n", ai_temp_main);
        printf("Main Humidity: %d\n", ai_humid_main);
        printf("Main Brightness: %d\n", ai_brightness_main);
        printf("Main water flag: %d\n", ai_water_flag_main);
    


        if(ai_temp_main <= temperature_c)// Temp high? , FAN(GPIO 0) ON 
        {
                gpio_put(0, 1);
        }
        else                                    // else FAN(GPIO 0) Off
        {
                gpio_put(0, 0);
        }

        if(ai_humid_main <= humidity)      //Humi high? Window(GPIO 1) OPEN
        {
                set_servo_angle(90, 1);
        }
        else                                    //else? Window(GPIO 1) Close
        {
                set_servo_angle(0, 1);
        }


        if(ai_brightness_main >= adc_result)      //Brightness? Light(GPIO 2) ON
        {
                gpio_put(2, 1);
        }
        else                                    //else? Light(GPIO 2) OFF
        {
                gpio_put(2, 0);
        }

        if(ai_water_flag_main == 0)      //Water flag off?      Pump (GPIO 3) OFF 
        {
                gpio_put(3, 0);
        }
        else                             //else? pump (GPIO 3) ON for 2 sec
        {
                gpio_put(3, 1);
                sleep_ms(2000);
        }


    }
}
/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

void set_gpio(){
        
         gpio_init(0);
         gpio_init(1);
        gpio_init(2);
        gpio_init(3);      
        gpio_init(4);

        gpio_set_dir(0, GPIO_OUT);
        gpio_set_dir(1, GPIO_OUT);

        gpio_set_dir(2, GPIO_OUT);
        gpio_set_dir(3, GPIO_OUT);
        gpio_set_dir(4, GPIO_OUT);
}

void set_servo_angle(float angle, int servo_pin) {
    if (angle < 0.0) {
        angle = 0.0;
    } else if (angle > 90.0) {
        angle = 90.0;
    }

    // 각도를 펄스 폭으로 변환 (0도: 0.5ms, 90도: 2.5ms)
    float pulse_width = 0.5 + (angle / 90.0) * 2.0;

    // GPIO 설정
    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servo_pin);

    // PWM 설정
    pwm_set_wrap(slice_num, 1000000);  // 주파수 설정 (1e6 = 1MHz)
    pwm_set_chan_level(slice_num, PWM_CHAN_A, (uint16_t)(pulse_width * 50000));  // 펄스 폭 설정 (20ms 주기)

    // 대기
    sleep_ms(20);  // 20ms 대기 (주기)

    // PWM 해제
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    sleep_ms(20);  // 추가적인 대기
}

