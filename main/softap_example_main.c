/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "freertos/queue.h"


/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN
#define PORT 3333               //для второй ESP ведомой
#define PORT2 4444            //для пульта управления  
#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

static const char *TAG = "wifi softAP";
static volatile uint8_t control_byte=0, PWM_setpoint=0;      //PWM для сидух 0-255
static int8_t client_status=0;                     //Status для воторого ESP
static int8_t server_status=0;                     //Status для wifi AP
static uint8_t gpioX_state[14] = {0};  // индексы 1..10 для GPIO 1..10  ебаный костыль, т.к. чтение состояния ноги не работает
static uint8_t bitmapL_slave = 0; // для отправки на пульт
static uint8_t bitmapH_slave = 0; // для отправки на пульт
static uint8_t PWM_slave = 0; // для отправки на пульт
static int client_sock_4444 = -1;
void ctrl_byte_gpio_set(uint8_t control_byte);
void gpio_set_zero();
void gpio_init_();

QueueHandle_t tx_queue;
static TaskHandle_t tcp_server_handle = NULL;
static TaskHandle_t tcp_server2_handle = NULL;
static TaskHandle_t PWM_task_handle = NULL;
static TaskHandle_t status_tx_task_handle = NULL;
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}
void tcp_server_task(void *pvParameters); 
void tcp_server_task2(void *pvParameters); 
void PWM_task(void *pvParameters); 
void handle_received_data_from_phone(const char *rx_buffer, size_t len) ;
void status_tx_task(void *pvParameters);

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            
        },
    };
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

void app_main(void)
{
    gpio_init_();   
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init_softap();
    tx_queue = xQueueCreate(10, sizeof(uint8_t)); // очередь для байтов
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, &tcp_server_handle);
    xTaskCreate(tcp_server_task2, "tcp_server2", 4096, NULL, 5, &tcp_server2_handle);
    xTaskCreate(PWM_task, "pwm", 4096, NULL, 6, &PWM_task_handle);
    xTaskCreate(status_tx_task, "status_tx", 4096, NULL, 4, &status_tx_task_handle);
    uint8_t repeat_slave_x=0;
    while (1) {
        ctrl_byte_gpio_set(control_byte);
        control_byte=0;
        if(client_status<=-2){
            if (tcp_server_handle == NULL)
                {
                    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, &tcp_server_handle);
                    client_status=0;
                }
            
        }else if ((server_status!=1)&&(client_status==1)){
            if(repeat_slave_x%10==0){
                control_byte='x';
            }
            repeat_slave_x++;
        }
        if(server_status<=-2){
            if (tcp_server2_handle == NULL)
                {
                    xTaskCreate(tcp_server_task2, "tcp_server2", 4096, NULL, 5, &tcp_server2_handle);
                    server_status=0;
                }
            
        }else if (server_status!=1){
            gpio_set_zero();
        }
        if ((client_status==1)||(server_status==1)){
            gpio_set_level(13, true);
            if (server_status==1){
                gpio_set_level(13, false);
                vTaskDelay(30);
                gpio_set_level(13, true);
             }
        }else{
            gpio_set_level(13, false);
        }

        vTaskDelay(100);
    }
    
}

//AP to slave
//передает данные между двумя ESP32 3333, чтобы принять данные надо триггернуть чем нибудь отправку, отправка при получении
void tcp_server_task(void *pvParameters)
{
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        client_status=-2;
        tcp_server_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        client_status=-3;
        tcp_server_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        client_status=-4;
        tcp_server_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", PORT);

    while (1) {
        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);

        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        client_status=1;
        ESP_LOGI(TAG, "Client connected");


        char rx_buffer[128];
        int len;
        uint8_t byte_to_send;
        struct timeval timeout;
        timeout.tv_sec = 50;
        timeout.tv_usec = 0;
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        while (1) {
            // --- 1. Прием данных от клиента ---
            len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len > 0) {

                rx_buffer[len] = 0; // null terminate
                ESP_LOGI(TAG, "Received: %s", rx_buffer);
                if(rx_buffer[0]==0xAA){
                    bitmapL_slave = rx_buffer[1];
                    bitmapH_slave = rx_buffer[2];
                    PWM_slave = rx_buffer[3];
                }
                // --- 2. Отправка данных из очереди ---
                client_status=1;

            } else if (len == 0) {
                client_status=-1;
                ESP_LOGI(TAG, "Client disconnected");
                break;
            } else if (errno != EWOULDBLOCK && errno != EAGAIN){
                client_status=-1;
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }

            while (xQueueReceive(tx_queue, &byte_to_send, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Control byte send 3333: 0x%02X", byte_to_send);
                send(client_sock, &byte_to_send, 1, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(10)); // небольшая пауза
        }
        ESP_LOGI(TAG, "Client disconnected");
        close(client_sock);
    }
}

//PC to AP
//пульт принимает данные по 4444
void tcp_server_task2(void *pvParameters)
{
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    uint8_t status_loop=0;
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        server_status=-2;
        tcp_server2_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT2);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        server_status=-3;
        tcp_server2_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        server_status=-4;
        tcp_server2_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", PORT2);

    while (1) {
        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            server_status=-1;
            continue;
        }
        status_loop++;
        ESP_LOGI(TAG, "Client connected");
        client_sock_4444 = client_sock;
        server_status=1;
        char rx_buffer[128];
        int len;
        struct timeval timeout;
        timeout.tv_sec = 60;
        timeout.tv_usec = 0;
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        while ((len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0)) > 0) {
            rx_buffer[len] = 0; // null terminate
            ESP_LOGI(TAG, "Received: %s", rx_buffer);

            handle_received_data_from_phone(rx_buffer, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        server_status=0;
        ESP_LOGI(TAG, "Client disconnected");
        client_sock_4444 = -1;
        close(client_sock);
    }
}

void status_tx_task(void *pvParameters)
{
    uint8_t packet[8];

    while (1)
    {
        if ((server_status == 1) && (client_sock_4444 >= 0))
        {
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
            if(client_status<=0){
                    bitmapL_slave = 0xFF;
                    bitmapH_slave = 0xFF;
                    PWM_slave = 0xFF;
            }
            packet[0] = 0xAA;
            packet[1] = bitmapL;
            packet[2] = bitmapH;
            packet[3] = PWM_setpoint;
            packet[4] = bitmapL_slave;
            packet[5] = bitmapH_slave;
            packet[6] = PWM_slave;
            packet[7] = packet[1] ^ packet[2] ^ packet[3]^ packet[4]^ packet[5]^ packet[6];

            send(client_sock_4444, packet, sizeof(packet), 0);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void handle_received_data_from_phone(const char *rx_buffer, size_t len) {
    const char prefix[] = "Ch232";
    size_t prefix_len = strlen(prefix);

    // Проверка, что длина данных больше, чем префикс
    if (len > prefix_len && strncmp(rx_buffer, prefix, prefix_len) == 0) {
        // Сохраняем следующий байт после префикса в глобальную переменную
        control_byte = (uint8_t)rx_buffer[prefix_len];
        ESP_LOGI(TAG, "Control byte updated: 0x%02X", control_byte);
    }
}


void PWM_task(void *pvParameters){
    uint8_t loop=0,gpio_perv_state=0;
    while(1){
        if(loop>PWM_setpoint){
            gpio_set_level(0, false);
        }else{
            gpio_set_level(0, true);
        }    
        loop++;
        loop=(loop>250)?0:loop;
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
        case 'A': // front minus E+D H=0      тоже самое что и 10 только с другой комбинацией реле, возможно не те реле включил, надо проверить
            gpioX_state[5] = !gpioX_state[5];       //E
            gpio_set_level(5, gpioX_state[5]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'B': // front plus E+H D=0
            gpioX_state[5] = !gpioX_state[5];       //E
            gpio_set_level(5, gpioX_state[5]);
            gpioX_state[4] = 0;
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = !gpioX_state[8];
            gpio_set_level(8, gpioX_state[8]);
            break;

        case 'C': // front minus C+D H=0
            gpioX_state[3] = !gpioX_state[3];       //C
            gpio_set_level(3, gpioX_state[3]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'D': // front plus C+H D=0
            gpioX_state[3] = !gpioX_state[3];
            gpio_set_level(3, gpioX_state[3]);
            gpioX_state[4] = 0;
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = !gpioX_state[8];
            gpio_set_level(8, gpioX_state[8]);
            break;
        case 'E': // back minus G+D H=0
            gpioX_state[7] = !gpioX_state[7];       //G
            gpio_set_level(7, gpioX_state[7]);
            gpioX_state[4] = !gpioX_state[4];       //D
            gpio_set_level(4, gpioX_state[4]);
            gpioX_state[8] = 0;
            gpio_set_level(8, gpioX_state[8]);      //H
            break;
        case 'F': // back plus G+H D=0
            gpioX_state[7] = !gpioX_state[7];    //G
            gpio_set_level(7, gpioX_state[7]);
            gpioX_state[4] = 0;                  //D   
            gpio_set_level(4, gpioX_state[4]);         

            gpioX_state[8] = !gpioX_state[8];   //H
            gpio_set_level(8, gpioX_state[8]);
            break;
        case 'G': // headrest minus B+J A=0
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[2] = !gpioX_state[2];       //B
            gpio_set_level(2, gpioX_state[2]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;    
        case 'H': // headrest plus B+A J=0
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[2] = !gpioX_state[2];       //B
            gpio_set_level(2, gpioX_state[2]);
            gpioX_state[10] = 0;
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'I': // backrest minus F+J A=0
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[6] = !gpioX_state[6];
            gpio_set_level(6, gpioX_state[6]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'J': // backrest plus F+A J=0
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[6] = !gpioX_state[6];       //F
            gpio_set_level(6, gpioX_state[6]);
            gpioX_state[10] = 0;                    //J
            gpio_set_level(10, gpioX_state[10]);
            break;  
        case 'K': // xx minus I+J A=0        //TODO проверить возможно не те реле включил тут 
            gpioX_state[1] = 0;       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[9] = !gpioX_state[9];       //I
            gpio_set_level(9, gpioX_state[9]);
            gpioX_state[10] = !gpioX_state[10];     //J
            gpio_set_level(10, gpioX_state[10]);
            break;        
        case 'L': // xx Plus I+A J=0        //TODO проверить возможно не те реле включил тут 
            gpioX_state[1] = !gpioX_state[1];       //A
            gpio_set_level(1, gpioX_state[1]);
            gpioX_state[9] = !gpioX_state[9];       //I
            gpio_set_level(9, gpioX_state[9]);
            gpioX_state[10] = 0;     //J
            gpio_set_level(10, gpioX_state[10]);
            break;
        case 'M': // all off
            PWM_setpoint=0;
            gpio_set_zero();    
            break;
        case 'N':
                PWM_setpoint=0;
            break;  
        case 'O':
                PWM_setpoint=85;
            break;  
        case 'P':
                PWM_setpoint=170;
            break;  
        case 'Q':
                PWM_setpoint=255;
            break;  
        case 'R': // GPIO10 toggle
            gpioX_state[12] = !gpioX_state[12];
            gpio_set_level(12, gpioX_state[12]);
            break;
        default:
            ESP_LOGI(TAG, "Control byte in queue: 0x%02X", control_byte);
            xQueueSend(tx_queue, &control_byte, 10);
            break;
        }
        
}