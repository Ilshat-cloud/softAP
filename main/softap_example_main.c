#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/gpio.h"

static const char *TAG = "wifi_sta";
#define WIFI_SSID       CONFIG_ESP_WIFI_SSID
#define WIFI_PASS       ""
#define MAXIMUM_RETRY   5000

/* TCP client settings */
#define SERVER_IP       "192.168.4.1"   // <-- подставь IP сервера
// #define SERVER_HOSTNAME "myserver.local" // <-- можно вместо IP раскомментировать и использовать getaddrinfo
#define SERVER_PORT     3333
#define RECONNECT_DELAY_MS 5000

static int8_t client_status=0;                     //Status для воторого ESP
static int8_t server_status=0;                     //Status для wifi AP
static uint8_t gpioX_state[13] = {0};  // индексы 1..10 для GPIO 1..10  ебаный костыль, т.к. чтение состояния ноги не работает
static uint8_t control_byte=0, PWM_setpoint=0;      //PWM для сидух 0-255
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static TaskHandle_t tcp_client_handle = NULL;

static void tcp_client_task(void *pvParameters);
void PWM_task(void *pvParameters); 
void ctrl_byte_gpio_set(uint8_t control_byte);
void gpio_set_zero();
void gpio_init_();
/* Обработчик событий WiFi / IP */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> esp_wifi_connect()");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (s_retry_num < MAXIMUM_RETRY) {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED -> reconnect attempt %d", s_retry_num);
            } else {
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED -> reached max retries");
            }
            wifi_event_sta_disconnected_t* ev = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGI(TAG, "Disconnected. Reason: %d", ((wifi_event_sta_disconnected_t*)event_data)->reason);
            server_status=0;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/* Инициализация WiFi в режиме STA и попытка подключения */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // Инициализация TCP/IP и NVS должна быть выполнена до
    esp_netif_init();
    esp_event_loop_create_default();

    // создаём netif для STA
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Регистрируем обработчики
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Конфигурация STA
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,  
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

/* Пример app_main, ждём подключения, затем выполняем дальнейшие действия */
void app_main(void)
{
    gpio_init_();  
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, &tcp_client_handle);

    while (1) {
        ctrl_byte_gpio_set(control_byte);
        control_byte=0;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* --- TCP client task --- */
static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[256];
    char tx_buffer[64];
    char host_ip[64];
    int sock = -1;

    // Можно использовать SERVER_HOSTNAME через getaddrinfo; здесь пока используем IP строку:
    strncpy(host_ip, SERVER_IP, sizeof(host_ip) - 1);
    host_ip[sizeof(host_ip)-1] = 0;

    while (1) {
        // Ждём подключения к WiFi (блокирующе)
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "TCP client: WiFi connected, trying to connect to %s:%d", host_ip, SERVER_PORT);

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);

        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            continue;
        }

        // По желанию: таймауты подключения/приема
        struct timeval tv;
        tv.tv_sec = 20;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        control_byte='x';
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected");
        server_status=1;

        // Основной цикл обмена: читаем и отправляем данные
        while (1) {
            // Чтение от сервера
            int len = recv(sock, rx_buffer, sizeof(rx_buffer)-1, 0);
            if (len > 0) {
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "Received from server: %s", rx_buffer);
                control_byte=rx_buffer[len-1];                // тут беерм последний байт который пришел, ответ приходит сразу как закинем запрос
                
            } else if (len == 0) {
                ESP_LOGW(TAG, "Connection closed by server");
                break;
            } else {
                // errno может быть EWOULDBLOCK/EAGAIN если таймаут
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // ничего не пришло в таймаут - можно послать heartbeat
                    static uint8_t loop_status=0;
                    if (loop_status>5){
                        loop_status=0;
                        
                        uint8_t bitmapL = 0;
                        uint8_t bitmapH = 0;

                        for (uint8_t i = 1; i <= 8; i++)
                        {
                            if (gpioX_state[i])
                            {
                                bitmapL |= (1 << (i - 1));
                            }
                        }

                        for (uint8_t i = 9; i <= 12; i++)
                        {
                            if (gpioX_state[i])
                            {
                                bitmapH |= (1 << (i - 9));
                            }
                        }   
                        
                        tx_buffer[0]=0xAA;
                        tx_buffer[1]=bitmapL;
                        tx_buffer[2]=bitmapH;
                        tx_buffer[3]=PWM_setpoint;
                        send(sock, tx_buffer, sizeof(tx_buffer), 0);
                    }
                    loop_status++;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Закрываем и переподключаемся
        if (sock != -1) {
            server_status=0;
            close(sock);
            sock = -1;
        }
        ESP_LOGI(TAG, "Reconnecting in %d ms...", RECONNECT_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }

    vTaskDelete(NULL);
}


void PWM_task(void *pvParameters){
    uint8_t loop=0;
    int8_t gpio_perv_state=0;
    while(1){

        if(server_status){
            gpio_set_level(13, true);  
        }else{
            gpio_set_level(13, false);  
        }
        if(loop>PWM_setpoint){
            gpio_set_level(0, false);
        }else{
            gpio_set_level(0, true);
        }    
        loop++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






void gpio_init_(){
    gpio_reset_pin(13);
    gpio_reset_pin(12);

    gpio_reset_pin(0);
    gpio_reset_pin(1);
    gpio_reset_pin(2);
    gpio_reset_pin(3);
    gpio_reset_pin(4);
    gpio_reset_pin(5);
    gpio_reset_pin(6);
    gpio_reset_pin(7);    
    gpio_reset_pin(8);
    gpio_reset_pin(9);
    gpio_reset_pin(10);  
    gpio_reset_pin(11);  
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_direction(12, GPIO_MODE_OUTPUT);  //светодиод и выход 
    gpio_set_direction(0, GPIO_MODE_OUTPUT);
    gpio_set_direction(1, GPIO_MODE_OUTPUT);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_direction(3, GPIO_MODE_OUTPUT);
    gpio_set_direction(4, GPIO_MODE_OUTPUT);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);    
    gpio_set_direction(6, GPIO_MODE_OUTPUT);
    gpio_set_direction(7, GPIO_MODE_OUTPUT);
    gpio_set_direction(8, GPIO_MODE_OUTPUT);
    gpio_set_direction(9, GPIO_MODE_OUTPUT); 
    gpio_set_direction(10, GPIO_MODE_OUTPUT); 
    gpio_set_direction(11, GPIO_MODE_INPUT); 
    memset(&gpioX_state[1],0,11);
    gpio_set_level(1, gpioX_state[1]);
    gpio_set_level(2, gpioX_state[2]);
    gpio_set_level(3, gpioX_state[3]);
    gpio_set_level(4, gpioX_state[4]);
    gpio_set_level(5, gpioX_state[5]);
    gpio_set_level(6, gpioX_state[6]);
    gpio_set_level(7, gpioX_state[7]);
    gpio_set_level(8, gpioX_state[8]);
    gpio_set_level(9, gpioX_state[9]);
    gpio_set_level(10, gpioX_state[10]);
    gpio_set_level(12, gpioX_state[12]);
}

void gpio_set_zero(){
    memset(&gpioX_state[1],0,11);
    gpio_set_level(1, gpioX_state[1]);
    gpio_set_level(2, gpioX_state[2]);
    gpio_set_level(3, gpioX_state[3]);
    gpio_set_level(4, gpioX_state[4]);
    gpio_set_level(5, gpioX_state[5]);
    gpio_set_level(6, gpioX_state[6]);
    gpio_set_level(7, gpioX_state[7]);
    gpio_set_level(8, gpioX_state[8]);
    gpio_set_level(9, gpioX_state[9]);
    gpio_set_level(10, gpioX_state[10]);
    gpio_set_level(12, gpioX_state[12]);
}

//функция для обработки байта управления, который может прийти от пульта, в зависимости от значения байта выполняются разные действия, например включение определенных GPIO или отправка байта в очередь для второго ESP
void ctrl_byte_gpio_set(uint8_t control_byte){
       switch (control_byte){
        case 0:
            break;
        case 'a': // front minus E+D H=0      тоже самое что и 10 только с другой комбинацией реле, возможно не те реле включил, надо проверить
            gpioX_state[5] = !gpioX_state[5];       //E
            gpio_set_level(5, gpioX_state[5]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'b': // front plus E+H D=0
            gpioX_state[5] = !gpioX_state[5];       //E
            gpio_set_level(5, gpioX_state[5]);
            gpioX_state[4] = 0;
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = !gpioX_state[8];
            gpio_set_level(8, gpioX_state[8]);
            break;

        case 'c': // front minus C+D H=0
            gpioX_state[3] = !gpioX_state[3];       //C
            gpio_set_level(3, gpioX_state[3]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'd': // front plus C+H D=0
            gpioX_state[3] = !gpioX_state[3];
            gpio_set_level(3, gpioX_state[3]);
            gpioX_state[4] = 0;
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = !gpioX_state[8];
            gpio_set_level(8, gpioX_state[8]);
            break;
        case 'e': // back minus G+D H=0
            gpioX_state[7] = !gpioX_state[7];       //G
            gpio_set_level(7, gpioX_state[7]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'f': // back plus G+H D=0
            gpioX_state[7] = !gpioX_state[7];    //G
            gpio_set_level(7, gpioX_state[7]);
            gpioX_state[4] = 0;                  //D   
            gpio_set_level(4, gpioX_state[4]);         

            gpioX_state[8] = !gpioX_state[8];   //H
            gpio_set_level(8, gpioX_state[8]);
            break;
        case 'g': // headrest minus B+J A=0
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[2] = !gpioX_state[2];       //B
            gpio_set_level(2, gpioX_state[2]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;    
        case 'h': // headrest plus B+A J=0
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[2] = !gpioX_state[2];       //B
            gpio_set_level(2, gpioX_state[2]);
            gpioX_state[10] = 0;
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'i': // backrest minus F+J A=0
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[6] = !gpioX_state[6];
            gpio_set_level(6, gpioX_state[6]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'j': // backrest plus F+A J=0
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[6] = !gpioX_state[6];       //F
            gpio_set_level(6, gpioX_state[6]);
            gpioX_state[10] = 0;                    //J
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'k': // xx minus I+J A=0        //наклон задний
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[9] = !gpioX_state[9];       //I
            gpio_set_level(9, gpioX_state[9]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;        
        case 'l': // xx Plus I+A J=0        //наклон задний 
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[9] = !gpioX_state[9];       //I
            gpio_set_level(9, gpioX_state[9]);
            gpioX_state[10] = 0;     //J
            gpio_set_level(10, gpioX_state[10]);
            break;
        case 'm': // all off
            PWM_setpoint=0;
            gpio_set_zero();    
            break;
        case 'n':
                PWM_setpoint=0;
            break;  
        case 'o':
                PWM_setpoint=85;
            break;  
        case 'p':
                PWM_setpoint=170;
            break;  
        case 'q':
                PWM_setpoint=255;
            break;  
        case 'r': // GPIO10 toggle
            gpioX_state[12] = !gpioX_state[12];
            gpio_set_level(12, gpioX_state[12]);
            break;
        default:
            gpio_set_zero();
            break;
        }
        
}