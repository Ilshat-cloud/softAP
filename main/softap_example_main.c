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
#include "driver/uart.h"

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN
#define PORT 3333               //Порт для TCP сервера
#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

#define UART_NUM UART_NUM_1
#define BUF_SIZE 5000
#define RD_BUF_SIZE 5000

#define TXD_PIN GPIO_NUM_4
#define RXD_PIN GPIO_NUM_5
#define RTS_PIN 6  // Для управления направлением RS485 (опционально)

static const char *TAG = "wifi softAP";
static int8_t client_status=0;                     //Status для воторого ESP32
static uint8_t gpioX_state[14] = {0};  // индексы 1..10 для GPIO 1..10  ебаный костыль, т.к. чтение состояния ноги не работает
QueueHandle_t tx_queue_tcp;
QueueHandle_t tx_queue_uart;
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
void PWM_task(void *pvParameters); 
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
            .gtk_rekey_interval = EXAMPLE_GTK_REKEY_INTERVAL,
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
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(13);
    gpio_reset_pin(12);
 
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_direction(12, GPIO_MODE_OUTPUT);  //светодиод и выход 
    gpio_set_level(12, gpioX_state[12]);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");   
    wifi_init_softap();
    tx_queue_tcp = xQueueCreate(4096, sizeof(uint8_t)); // очередь для байтов
    tx_queue_uart = xQueueCreate(4096, sizeof(uint8_t)); // очередь для байтов
    xTaskCreate(tcp_server_task, "tcp_server", 10000, NULL, 1, NULL);
    xTaskCreate(PWM_task, "pwm", 10000, NULL, 1, NULL);

    while (1) {
 

        if (client_status==1){
                gpio_set_level(13, false);
                vTaskDelay(30);
                gpio_set_level(13, true);
        }else{
            gpio_set_level(13, false);
        }

        vTaskDelay(100);
    }
    
}

//AP
void tcp_server_task(void *pvParameters)
{
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);


    while (1) {
        // Принудительно закрыть все сокеты на этом порту
        client_status=0;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock >= 0) {
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            close(sock);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            client_status=-2;
            vTaskDelete(NULL);
            continue;
        }


        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(listen_sock, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));

        // Установка таймаута на accept
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(PORT);
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(listen_sock);
            client_status=-3;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (listen(listen_sock, 1) < 0) {
            ESP_LOGE(TAG, "Error during listen: errno %d", errno);
            close(listen_sock);
            client_status=-4;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "TCP server listening on port %d", PORT);
        client_status = 0; // Сервер готов к подключениям

        while (1) {
            client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);

            if (client_sock < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // Таймаут accept, продолжаем ждать
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break; // Выходим из цикла accept, пересоздаем сокет
            }

            client_status=1;

            int flag = 1;
            setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));

            char rx_buffer[4096];
            int len;
            uint8_t byte_to_send;

            struct timeval timeout;
            timeout.tv_sec = 30;
            timeout.tv_usec = 0;
            setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            fcntl(client_sock, F_SETFL, O_NONBLOCK);
            ESP_LOGI(TAG, "Client connected from %s:%d",inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
            while (1) {
                // --- 1. Прием данных от клиента ---
                len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                if (len > 0) {
                    client_status=1;
                    //xQueueReset(tx_queue_uart);
                    for (uint8_t i=0;i<len;i++){
                        xQueueSend(tx_queue_uart,&rx_buffer[i],0);
                    }
                    

                } else if (len == 0) {
                    client_status=-1;
                    ESP_LOGI(TAG, "Client disconnected gracefully");
                    break;
                } else if (errno != EWOULDBLOCK && errno != EAGAIN){
                    client_status=-1;
                    ESP_LOGE(TAG, "recv failed: errno %d", errno);
                    break;
                }

                while (xQueueReceive(tx_queue_tcp, &byte_to_send, 2) == pdTRUE) {
                    ESP_LOGI(TAG, "ReceivedUart: %d", byte_to_send);
                    if (send(client_sock, &byte_to_send, 1, MSG_DONTWAIT) < 0) {
                        if (errno != EWOULDBLOCK && errno != EAGAIN) {
                            ESP_LOGE(TAG, "send failed: errno %d", errno);
                            break;
                        }
                    }
                }
                //vTaskDelay(pdMS_TO_TICKS(2)); // небольшая пауза
            }
            client_status=0;
            ESP_LOGI(TAG, "Connection closed, waiting for new client...");
            close(client_sock);
        }
        // Закрытие слушающего сокета перед повторной попыткой
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Пауза перед пересозданием
    }
}




void PWM_task(void *pvParameters){
// Настройка UART для RS485
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 120,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
    
    // Установка размера буфера
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 4, BUF_SIZE * 4, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX));

    gpio_set_direction(RTS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RTS_PIN, 0);  // По умолчанию режим приема
    uint8_t uart_rx_buffer[RD_BUF_SIZE];
    uint8_t queue_byte;
    
    while(1) {
        // --- 1. Чтение из очереди tx_queue_uart и отправка в UART ---
        while (xQueueReceive(tx_queue_uart, &queue_byte, 2) == pdTRUE) {
             // Включить передачу (DE/RE = HIGH)
            gpio_set_level(RTS_PIN, 1);  // Включить передачу

            // Отправка байта в UART
            uart_write_bytes(UART_NUM, (const char*)&queue_byte, 1);
            uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(10));
            //ESP_LOGI(TAG, "ReceivedWiFi: %d", queue_byte);



        }
        gpio_set_level(RTS_PIN, 0);  // Выключить передачу
        // --- 2. Чтение из UART и отправка в очередь tx_queue_tcp ---
        int length = uart_read_bytes(UART_NUM, uart_rx_buffer, RD_BUF_SIZE - 1, pdMS_TO_TICKS(50));
        
        if (length > 0) {
            for (int i = 0; i < length; i++) {
                xQueueSend(tx_queue_tcp, &uart_rx_buffer[i], portMAX_DELAY);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Небольшая пауза
    }
}