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
static void tcp_client_task(void *pvParameters);
void PWM_task(void *pvParameters); 
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
            //.threshold.authmode = WIFI_AUTH_WPA2_PSK,     //TODO Убрать если нужен пароль
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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
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
    gpio_reset_pin(12);    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_direction(12, GPIO_MODE_OUTPUT);
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
    gpio_set_direction(12, GPIO_MODE_OUTPUT); 
    memset(&gpioX_state[1],0,12);
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
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    // Ждём подключения (по событию) и затем стартуем TCP клиент как таск.
    // Можно запускать сразу — в клиенте стоит waitBits.
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    // // Ждём подключения (с тайм-аутом можно вариативно)  просто подключался с этим
    // EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
    //                                        WIFI_CONNECTED_BIT,
    //                                        pdFALSE,
    //                                        pdFALSE,
    //                                        pdMS_TO_TICKS(15000)); // ждём 15 сек

    // if (bits & WIFI_CONNECTED_BIT) {
    //     ESP_LOGI(TAG, "Connected to AP, now can start other tasks (e.g. TCP client/server)");
    //     // TODO: создать задачи/инициализировать клиент/сервер
    // } else {
    //     ESP_LOGW(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    //     // можно пробовать повторно или fallback
    // }

    // пример: оставляем таск живым (или можно запускать своё приложение)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
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
        // Пример: при подключении отправляем идентификацию
        const char *hello = "ESP32 client connected\n";
        send(sock, hello, strlen(hello), 0);

        // Основной цикл обмена: читаем и отправляем данные
        while (1) {
            // Чтение от сервера
            sprintf(tx_buffer,"Active outputs: ");
            uint8_t len2 = strlen(tx_buffer);  
            for (uint8_t i =0; i<sizeof(gpioX_state);i++){
                if (gpioX_state[i]){
                    tx_buffer[len2++] = 'a' + i-1;
                }
            }
            tx_buffer[len2++] = '\r';
            tx_buffer[len2++] = '\n';
            tx_buffer[len2]   = '\0';
            send(sock, tx_buffer, strlen(tx_buffer), 0); 

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
                    const char hb[] = "HB\n";
                    send(sock, hb, sizeof(hb)-1, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        control_byte='x';
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
        switch (control_byte){
        case 'a': // GPIO1 toggle
            gpioX_state[1] = !gpioX_state[1];
            gpio_set_level(1, gpioX_state[1]);
            ESP_LOGI(TAG, "GPIO 1 A toggle %d", gpioX_state[1]);
            break;

        case 'b': // GPIO2 toggle
            gpioX_state[2] = !gpioX_state[2];
            gpio_set_level(2, gpioX_state[2]);
            ESP_LOGI(TAG, "GPIO 2 B toggle %d", gpioX_state[2]);
            break;

        case 'c': // GPIO3 toggle
            gpioX_state[3] = !gpioX_state[3];
            gpio_set_level(3, gpioX_state[3]);
            ESP_LOGI(TAG, "GPIO 3 C toggle %d", gpioX_state[3]);
            break;

        case 'd': // GPIO4 toggle
            gpioX_state[4] = !gpioX_state[4];
            gpio_set_level(4, gpioX_state[4]);
            ESP_LOGI(TAG, "GPIO 4 D toggle %d", gpioX_state[4]);
            break;

        case 'e': // GPIO5 toggle
            gpioX_state[5] = !gpioX_state[5];
            gpio_set_level(5, gpioX_state[5]);
            ESP_LOGI(TAG, "GPIO 5 E toggle %d", gpioX_state[5]);
            break;

        case 'f': // GPIO6 toggle
            gpioX_state[6] = !gpioX_state[6];
            gpio_set_level(6, gpioX_state[6]);
            break;

        case 'g': // GPIO7 toggle
            gpioX_state[7] = !gpioX_state[7];
            gpio_set_level(7, gpioX_state[7]);
            break;

        case 'h': // GPIO8 toggle
            gpioX_state[8] = !gpioX_state[8];
            gpio_set_level(8, gpioX_state[8]);
            break;

        case 'i': // GPIO9 toggle
            gpioX_state[9] = !gpioX_state[9];
            gpio_set_level(9, gpioX_state[9]);
            break;

        case 'j': // GPIO10 toggle
            gpioX_state[10] = !gpioX_state[10];
            gpio_set_level(10, gpioX_state[10]);
            break;
        case 'k':
                PWM_setpoint=0;
            break;  
        case 'o':
                PWM_setpoint=85;
            break;  
        case 'm':
                PWM_setpoint=170;
            break;  
        case 'n':
                PWM_setpoint=255;
            break;  
        case 'l': // GPIO10 toggle
            gpioX_state[12] = !gpioX_state[12];
            gpio_set_level(12, gpioX_state[12]);
            break;
        case 'x':
            memset(&gpioX_state[1],0,10);
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
            break;             
        default:

            break;
        }
        control_byte=0;

        if(server_status){
            gpio_set_level(13, true);  
        }else{
            gpio_set_level(13, false);  
        }

        if(gpio_get_level(11)){
            PWM_setpoint=255;
            gpio_perv_state=1;
        }else if (gpio_perv_state){
            PWM_setpoint=0;
            gpio_perv_state=0;
        }
        if(loop>PWM_setpoint){
            gpio_set_level(0, false);
        }else{
            gpio_set_level(0, true);
        }    
        loop++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}