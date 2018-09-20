/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (GPIO_NUM_12)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (128)

#define UART_RX_BUFFER 32
typedef struct
{
    unsigned char       update;										    // 置一时更新
	float               value;										    // 当前值
    float               MAX_value;										// 最大值

}HCHO_t;

/* Private variables ---------------------------------------------------------*/
HCHO_t     HCHO;
static unsigned char receive_data[UART_RX_BUFFER];
static unsigned char receive_data_S[256];


#define GPIO_OUTPUT_IO_0    34
#define GPIO_OUTPUT_IO_1    35
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
static unsigned char FucCheckSum(unsigned char *data, unsigned char ln)
{

    unsigned char j, tempq = 0;


    for(j = 0; j < (ln - 2); j++)
    {

        tempq += data[j + 1];

    }
    // LOG("\r\n计算校验值tempq = 0x%02x\r\n", tempq);
    tempq = (~tempq);
    // LOG("\r\n计算校验值~tempq = 0x%02x\r\n", tempq);
    return(tempq + 1);

}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    uint32_t cnt = 0;
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    for(;;)
    {
    	printf("cnt: %d\n", cnt);
        vTaskDelay(500 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_IO_1, (cnt + 1)% 2);
        cnt++;
    }
}
static void HCHO_Read(void)
{
    int i,j, received_len;
    // 接收数据
    memset(receive_data, 0, UART_RX_BUFFER);
    memset(receive_data_S, 0, 256);

    received_len = uart_read_bytes(1, receive_data, UART_RX_BUFFER, 100 / portTICK_RATE_MS);
    if(received_len > 0)
    {
        for (i = 0, j = 0; i < received_len; i++)
        {
            sprintf((char*)&receive_data_S[j], "0x%02X ", receive_data[i]);
            j += 5;
        }
        receive_data_S[j] = 0;
        printf("\r\n接收到数据: %s\r\n", receive_data_S);
        // 长度正确
        if(received_len == 9)
        {

            if ((receive_data[0] == 0xFF) && (receive_data[1] = 0x17))
            {
                // 校验
            	printf("\r\n计算校验值\r\n");

                if (FucCheckSum(receive_data, received_len) == receive_data[received_len - 1])
                {
                    HCHO.value      = (receive_data[5] + (receive_data[4] * 256));
                    HCHO.value     *= 0.00125;
                    HCHO.MAX_value  = (receive_data[7] + (receive_data[6] * 256));
                    HCHO.MAX_value *= 0.00125;
                    HCHO.update     = 1;
                    printf("\r\n甲醛 = %04.3f mg/M3， 最大量程 = %04.3f mg/M3\r\n", HCHO.value, HCHO.MAX_value);
                }
                else
                {
                	printf("\r\n甲醛校验出错\r\n");
                }
            }
        }
        else if (received_len > 0)
        {
        	printf("\r\n接收到错误甲醛数据, 长度 = %d \r\n", received_len);
        }

    }
}
static void echo_task()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
    	HCHO_Read();
//        // Read data from the UART
//        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);
//        // Write data back to the UART
////        uart_write_bytes(UART_NUM_0, (const char *) data, len);
//        if (len > 0)
//        {
//        	printf("接收数据长度%d字节\r\n",len);
//        }
    }
}

void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
}
