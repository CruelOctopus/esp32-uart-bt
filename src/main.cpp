
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bootloader_random.h"

#include "crc32.h"
#include "protocol.h"

#define ECHO_TEST_TXD (GPIO_NUM_17)
#define ECHO_TEST_RXD (GPIO_NUM_16)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define BT_STATE GPIO_NUM_35
#define BT_KEY GPIO_NUM_32
#define BT_RESET GPIO_NUM_33

#define GPIO_OUTPUT_PIN_MASK ((1ULL << BT_KEY) | (1ULL << BT_RESET))
#define GPIO_INPUT_PIN_MASK (1ULL << BT_STATE)

#define ESP_GPIO_LEVEL_HIGH 1
#define ESP_GPIO_LEVEL_LOW 0

#define UART_BUF_SIZE 128

uint8_t RandomNumberBuffer[32];
uint8_t IV[16];

void GPIO_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_MASK;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; //GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    //io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //gpio_set_direction(BT_STATE,GPIO_MODE_INPUT);
    //gpio_set_direction(BT_KEY,GPIO_MODE_OUTPUT);
    //gpio_set_direction(BT_RESET,GPIO_MODE_OUTPUT);

    gpio_set_level(BT_RESET, ESP_GPIO_LEVEL_HIGH);
}
void UARTConfig()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

void GetRandomNumbers(uint8_t *buffer, uint8_t size)
{
    void *Vp = buffer;
    uint32_t *Bp = static_cast<uint32_t *>(Vp);

    for (int i = 0; i < (size / 4); i++)
    {
        *(Bp + i) = esp_random();
    }
}
extern "C" void app_main()
{
    // Configure a temporary buffer for the incoming data
    uint8_t UART_data[UART_BUF_SIZE];
    //uint8_t * UART_data_ptr = data[128];
    const char *CMD_AT = "AT\r\n";                              //6
    const char *CMD_AT_VERSION = "AT+VERSION?\r\n";             //15
    const char *CMD_AT_ROLE_SET = "AT+ROLE=1\r\n";              //13
    const char *CMD_AT_INQUIRE_SET = "AT+INQM=1,5,4\r\n";       //17
    const char *CMD_AT_PASSWORD_SET = "AT+PSWD=\"5555\"\r\n";   //18
    const char *CMD_AT_BAUD_RATE_SET = "AT+UART=38400,0,0\r\n"; //21
    const char *CMD_AT_CONNECTION_MODE_SET = "AT+CMODE=1\r\n";  //14
    const char *CMD_AT_SAFE_ENCRYPTION_SET = "AT+SENM=3,2\r\n"; //15
    const char *CMD_AT_GET_STATUS = "AT+STATE?\r\n";            //13

    int len = 0;

    GPIO_init();
    UARTConfig();
    //vTaskDelay(10000 / portTICK_RATE_MS);
    gpio_set_level(BT_RESET, ESP_GPIO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BT_KEY, ESP_GPIO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BT_RESET, ESP_GPIO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    //----------- AT-CMD-MODE------
    ESP_LOGI("UART1", "Enter AT-CMD-MODE");
    ESP_LOGI("UART1", "%s", CMD_AT);
    uart_write_bytes(UART_NUM_1, CMD_AT, 6);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    //ESP_LOGI("UART1", "len - %d",len);
    //esp_log_buffer_hex("UART", UART_data, len);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_VERSION);
    uart_write_bytes(UART_NUM_1, CMD_AT_VERSION, 15);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_ROLE_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_ROLE_SET, 13);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_INQUIRE_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_INQUIRE_SET, 17);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_PASSWORD_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_PASSWORD_SET, 18);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_BAUD_RATE_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_BAUD_RATE_SET, 21);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_CONNECTION_MODE_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_CONNECTION_MODE_SET, 14);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    ESP_LOGI("UART1", "%s", CMD_AT_SAFE_ENCRYPTION_SET);
    uart_write_bytes(UART_NUM_1, CMD_AT_SAFE_ENCRYPTION_SET, 15);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    gpio_set_level(BT_KEY, ESP_GPIO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(BT_RESET, ESP_GPIO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(BT_RESET, ESP_GPIO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BT_KEY, ESP_GPIO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("UART1", "%s", CMD_AT_GET_STATUS);
    uart_write_bytes(UART_NUM_1, CMD_AT_GET_STATUS, 13);
    if (uart_wait_tx_done(UART_NUM_1, 100) != ESP_OK)
    {
        ESP_LOGI("UART1", "Can not send data!");
    }
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
    printf("%s", UART_data);

    gpio_set_level(BT_KEY, ESP_GPIO_LEVEL_LOW);

    //------------------main loop---------------
    protocol frame;
    bootloader_random_enable(); // when use wifi or bluetooth it must me disable
    GetRandomNumbers(frame.,32);
    GetRandomNumbers(IV,16);
    
    while (true)
    {
        if (gpio_get_level(BT_STATE) == ESP_GPIO_LEVEL_HIGH)
        {
            len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);
            if (len != 0)
            {
                printf("%s", UART_data);
            }
        }else
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        
    }
}
