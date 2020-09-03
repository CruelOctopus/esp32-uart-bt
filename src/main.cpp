
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bootloader_random.h"

#include "crc32.h"
#include "protocol.h"


#include "mbedtls/aes.h"
#include "mbedtls/md.h"




#include <string.h>

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

const char *key = "11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff";
const char *HexStringKeygAES = "11223344556677889900aabbccddeeff11223344556677889900aabbccddeeff";
uint8_t RandomNumberBuffer32[32];
uint8_t RandomNumberBuffer16[16];
uint8_t IV[16];
uint8_t AESKEY[32]; //256 bit key
//uint8_t AESKEY[16]; //128 bit key
uint8_t AESDEcryptResult[64];
uint8_t hmacSHA256Result[32];

uint8_t BKeyLen = 64;
uint8_t HMAC256ByteKey[64];
uint8_t HMAC256ByteKeyForIV[64];

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
void Hmac256Calculate(uint8_t *key, uint8_t keyLength, uint8_t *payload, uint8_t payloadLength, uint8_t *hmacResult)
{
  //char *key = "secretKey";
  //char *payload = "Hello HMAC SHA 256!";

  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

  //const size_t payloadLength = strlen((char*)payload);
  //const size_t keyLength = strlen((char*)key);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key, keyLength);
  mbedtls_md_hmac_update(&ctx, (const unsigned char *)payload, payloadLength);
  mbedtls_md_hmac_finish(&ctx, hmacResult);
  mbedtls_md_free(&ctx);
}
void AESEncrypt(uint8_t *key, uint8_t DataLength, uint8_t *IV, uint8_t *payload, uint8_t *EncryptResult)
{
    mbedtls_aes_context ctx;
	mbedtls_aes_init( &ctx );
	mbedtls_aes_setkey_enc( &ctx, key, 256 );
    mbedtls_aes_crypt_cbc( &ctx, MBEDTLS_AES_ENCRYPT, DataLength, IV, payload, EncryptResult );
    mbedtls_aes_free( &ctx );
}
void AESDecrypt(uint8_t *key, uint8_t DataLength, uint8_t *IV, uint8_t *payload, uint8_t *DecryptResult)
{
    mbedtls_aes_context ctx;
	mbedtls_aes_init( &ctx );
	mbedtls_aes_setkey_enc( &ctx, key, 256 );
    mbedtls_aes_crypt_cbc( &ctx, MBEDTLS_AES_DECRYPT, DataLength, IV, payload, DecryptResult );
    mbedtls_aes_free( &ctx );
}
void StringHexToByte(uint8_t *InputData, uint8_t *bytearray, uint8_t arraylen)
{
  uint8_t b = 0, bt = 0;
  uint8_t halfbyte = 0;
  uint8_t j = 0;
  if (arraylen % 2 == 0)
  {
    for (int i = 0; i < arraylen; i++)
    {
      bytearray[i] = 0;
    }
    for (int i = 0; i < arraylen * 2; i++)
    {
      switch (InputData[i])
      {
      case '0':
      {
        b = 0;
        break;
      };
      case '1':
      {
        b = 1;
        break;
      };
      case '2':
      {
        b = 2;
        break;
      };
      case '3':
      {
        b = 3;
        break;
      };
      case '4':
      {
        b = 4;
        break;
      };
      case '5':
      {
        b = 5;
        break;
      };
      case '6':
      {
        b = 6;
        break;
      };
      case '7':
      {
        b = 7;
        break;
      };
      case '8':
      {
        b = 8;
        break;
      };
      case '9':
      {
        b = 9;
        break;
      };
      case 'a':
      {
        b = 10;
        break;
      };
      case 'b':
      {
        b = 11;
        break;
      };
      case 'c':
      {
        b = 12;
        break;
      };
      case 'd':
      {
        b = 13;
        break;
      };
      case 'e':
      {
        b = 14;
        break;
      };
      case 'f':
      {
        b = 15;
        break;
      };
      }
      if (halfbyte == 0)
      {

        bt = b << 4;
        halfbyte = 1;
      }
      else
      {
        bt |= b;
        bytearray[j] = bt;
        j++;
        halfbyte = 0;
        bt = 0;
      }
    }
  }
}
int XorByteArray(uint8_t * first, uint8_t *second,uint8_t length)
{
    int counter = 0;
    for (int i = 0; i < length; i++)
    {
        counter += first[i] ^ second[i];
    }
  return counter;
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
    /*
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
*/
    gpio_set_level(BT_KEY, ESP_GPIO_LEVEL_LOW);

    //------------------main loop---------------
    
    FrameSetup FSetup;
    FrameFirst FFirst;

    printf(" 1 ");
    bootloader_random_enable(); // when use wifi or bluetooth it must me disable
    printf(" 2 ");
     
    GetRandomNumbers(RandomNumberBuffer16,16); //fill random buffer for IV 16 bytes
    StringHexToByte((uint8_t *)key,HMAC256ByteKeyForIV,BKeyLen); // Load HMAC key for IV 64 bytes
    Hmac256Calculate(HMAC256ByteKeyForIV,sizeof(HMAC256ByteKeyForIV),RandomNumberBuffer16,sizeof(RandomNumberBuffer16),hmacSHA256Result); //Calculate HMAC for IV
    memcpy(&IV,&hmacSHA256Result,16); //Copy Half HMAC into IV aes
    
    printf(" 3 ");
    
    GetRandomNumbers(RandomNumberBuffer32,32); //fill random buffer 32 bytes
    StringHexToByte((uint8_t *)key,HMAC256ByteKey,BKeyLen); // Load HMAC key for Cheack commands 64 bytes
    Hmac256Calculate(HMAC256ByteKey,sizeof(HMAC256ByteKey),RandomNumberBuffer32,sizeof(RandomNumberBuffer32),hmacSHA256Result); //Calculate HMAC for Cheack commands
    
    StringHexToByte((uint8_t *)HexStringKeygAES,AESKEY,32); // Load AES key

    printf(" 3 ");
    
    //memset(FFirst.Synchro,SYNCHRO,8);
    //memcpy(&FFirst.AESIV,&RandomNumberBuffer16,16);
    memcpy(&FFirst.RandomData,&RandomNumberBuffer32,32);
    FFirst.CRC32 = crc32(&FFirst,56,0);
    
    memcpy(&FSetup.RandomData,&RandomNumberBuffer16,16);
    FSetup.CRC32 = crc32(&FSetup,sizeof(FSetup.RandomData),0);

    //-----------------Start dialog
    uint8_t 
    uart_write_bytes(UART_NUM_1,(const char *)&FSetup ,sizeof(FSetup));
    len = uart_read_bytes(UART_NUM_1, UART_data, UART_BUF_SIZE, 300 / portTICK_RATE_MS);
    if (len == 20)
    {
        FrameSetup Response;
        memcpy(&Response,&UART_data,len);
        if (Response.CRC32 == crc32(&Response,sizeof(Response.RandomData),0);
        {
            AESDecrypt(AESKEY,sizeof(Response.RandomData),IV,Response.RandomData,AESDEcryptResult);
            if (XorByteArray((uint8_t*)&FSetup, AESDEcryptResult,sizeof(Response.RandomData)) == 0)
            {
                uart_write_bytes(UART_NUM_1,(const char *)&FFirst ,sizeof(FFirst));
            }
        }
        
    }
    /*
    for (int i=0;i<32;i++){
        RandomNumberBuffer32[i] = i;
    }
    for (int i=0;i<16;i++){
        IV[i] = i;
    }
    */
    uint8_t buffer[1024];
    memset(buffer,0x00,sizeof(buffer));

    //AESEncrypt(AESKEY,sizeof(),IV,

     printf(" 7 ");
     printf(" RB ");
    //esp_log_buffer_hex("Random",RandomNumberBuffer,32);
    printf(" FR ");
    //esp_log_buffer_hex("frame",&frame,60);
    esp_log_buffer_hex("bufer",&buffer,1024);
   //-----------debug---------
  /*
  char str[3];
  for (int i = 0; i < sizeof(frame.Synchro); i++)
  {
    sprintf(str, "%02x", (frame.Synchro[i]));   
    printf("%s ",str);
  }
    for (int i = 0; i < sizeof(frame.AESIV); i++)
  {
    sprintf(str, "%02x", (frame.AESIV[i]));   
    printf("%s ",str);
  }
  printf(" 6.5 ");
    for (int i = 0; i < 32; i++)
  {
    sprintf(str, "%02x", (frame.RandomData[i]));   
    printf("%s ",str);
  }
  printf(" 7 ");
      for (int i = 0; i < sizeof(frame.CRC32); i++)
  {
    sprintf(str, "%02x", *(int *)(frame.CRC32));   
    printf("%s ",str);
  }
  */
//-----------debug---------
   // uart_write_bytes(UART_NUM_1,(const char *)&frame ,sizeof(frame));
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
