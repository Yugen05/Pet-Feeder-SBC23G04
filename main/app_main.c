#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#include "driver/timer.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "cJSON.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"

#include "ssd1306.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define PWM_FREQ 50 // 50 Hz
#define SERVO_PIN 5 // GPIO16
#define INF 27

#define WIFI_CONNECTED_EVENT 0
#define WIFI_DISCONNECTED_EVENT 1
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

uint8_t esp32_mac[6];
static EventGroupHandle_t event_group;
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "Pet-Feeder";

adc_channel_t adc_carga = ADC_CHANNEL_6; // GPIO34
int FOOD, carga = 0;

SSD1306_t dev;
char wifi[14] = "Esperando WiFi";

uint64_t task_counter_value; //valor timer0

/* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 0,
    }; // default clock source is APB

// Logo de la ETSISI
uint8_t etsisi[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff,
    0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xe0, 0x1f, 0xff,
    0xff, 0xe6, 0x4f, 0xe7, 0xff, 0xc7, 0x27, 0x1f, 0xff, 0xc7, 0x50, 0xff, 0xff, 0x87, 0x43, 0xff,
    0xf0, 0x20, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xff, 0xc1, 0x71, 0xff, 0xff, 0x01, 0x71, 0xff,
    0xfc, 0x00, 0x61, 0xff, 0xf0, 0x70, 0x03, 0xff, 0xc0, 0xf8, 0x07, 0xff, 0x8f, 0xfe, 0x1f, 0xff,
    0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff,
    0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff

};

uint8_t logo[] = {
    0x8c, 0x21, 0xc2, 0x18, 0xc7, 0x18, 0xc0, 0xb5, 0x85, 0xe8, 0x4b, 0x5b, 0x6a, 0x40, 0xb5, 0x57,
    0xea, 0x3a, 0xd9, 0x5a, 0x40, 0x8c, 0x77, 0x62, 0x38, 0xd9, 0x18, 0xc0, 0xbd, 0xb7, 0xee, 0x6b,
    0x5b, 0x6a, 0x40, 0x9c, 0x33, 0xce, 0x08, 0x43, 0x0a, 0x40, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff,
    0xff, 0xf9, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xe0,
    0x01, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xe0, 0x03, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xc0, 0x01, 0xff,
    0xff, 0xc0, 0xff, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xc0, 0xff, 0xff, 0x80, 0x30, 0xff, 0xff, 0xc0,
    0xff, 0xff, 0x84, 0xfe, 0x7f, 0xff, 0xc0, 0xff, 0xff, 0x89, 0xc6, 0x7f, 0xff, 0xc0, 0xff, 0xff,
    0x8f, 0xe3, 0x7f, 0xff, 0xc0, 0xff, 0xff, 0x8e, 0x71, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xce, 0x31,
    0xff, 0xff, 0xc0, 0xff, 0xff, 0xfe, 0x71, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xe1, 0xff, 0xff,
    0xc0, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0x07, 0xff, 0xff, 0xc0, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc0

};

// Bit map del circulo
uint8_t circulo[2][108] = {
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xc0, 0x07, 0xff, 0xff, 0x0f, 0xe1, 0xff,
     0xfe, 0x3f, 0xf8, 0xff, 0xfc, 0x7f, 0xfe, 0x7f, 0xfc, 0xff, 0xff, 0x3f, 0xf9, 0xff, 0xff, 0x3f,
     0xf3, 0xff, 0xff, 0x9f, 0xf3, 0xff, 0xff, 0x9f, 0xf3, 0xff, 0xff, 0xcf, 0xf7, 0xff, 0xff, 0xcf,
     0xe7, 0xff, 0xff, 0xcf, 0xe7, 0xff, 0xff, 0xcf, 0xe7, 0xff, 0xff, 0xcf, 0xf7, 0xff, 0xff, 0xcf,
     0xf3, 0xff, 0xff, 0xdf, 0xf3, 0xff, 0xff, 0x9f, 0xf9, 0xff, 0xff, 0x9f, 0xf9, 0xff, 0xff, 0x3f,
     0xfc, 0xff, 0xfe, 0x7f, 0xfe, 0x7f, 0xfc, 0x7f, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0x83, 0x83, 0xff,
     0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},

    {0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0x00, 0x00, 0xff,
     0xfe, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x0f,
     0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07,
     0xf0, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07,
     0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x1f,
     0xfc, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0xff, 0xff, 0x80, 0x01, 0xff,
     0xff, 0xe0, 0x07, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff}};

void notify_wifi_connected()
{
    xEventGroupClearBits(event_group, WIFI_DISCONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_CONNECTED_EVENT);
}

void notify_wifi_disconnected()
{
    xEventGroupClearBits(event_group, WIFI_CONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_DISCONNECTED_EVENT);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            notify_wifi_connected();
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(const char *running_partition_label)
{
    assert(running_partition_label != NULL);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // APP_ABORT_ON_ERROR(esp_wifi_init(&cfg));
    // APP_ABORT_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    wifi_config_t wifi_config = {};
    // APP_ABORT_ON_ERROR(esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config));

    wifi_sta_config_t wifi_sta_config = {
        .ssid = "Yugen",
        .password = "SBCwifi$",
    };

    wifi_config.sta = wifi_sta_config;

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // APP_ABORT_ON_ERROR(esp_wifi_get_mac(ESP_IF_WIFI_STA, esp32_mac));
    ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", esp32_mac[0], esp32_mac[1], esp32_mac[2], esp32_mac[3], esp32_mac[4], esp32_mac[5]);
    // APP_ABORT_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA));

    // APP_ABORT_ON_ERROR(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    // APP_ABORT_ON_ERROR(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to WIFI");
        notify_wifi_connected();
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", "CONFIG_EXAMPLE_WIFI_SSID", "CONFIG_EXAMPLE_WIFI_PASSWORD");
        notify_wifi_disconnected();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_mqtt_client_handle_t mClient;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");

        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        mClient = client;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mClient = NULL;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Insertar los datos del thingsBoard para establecer la conexión básica
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://demo.thingsboard.io",
        .event_handle = mqtt_event_handler,
        .port = 1883,
        .username = "2fIaEZVYzTaJ86worRb7",
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void sendData(esp_mqtt_client_handle_t client, int value)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "Carga", value);
    char *post_data = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0);
    cJSON_Delete(root);
    free(post_data);
}

static void init_mqtt(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();
}

static void init_oled(void)
{

#if CONFIG_I2C_INTERFACE
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
    dev._flip = true;
    ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
    ESP_LOGI(TAG, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
}

void init_servo(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };

    ledc_channel_config(&ledc_channel);
}
static void control_servo(int angle)
{
    int duty = (angle * 11.11111111) + 500; // Convert the angle to the duty cycle
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

static void giro_motor(void)
{
    for (int j = 0; j <= 60; j += 20)
    {
        control_servo(j);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    for (int j = 60; j >= 0; j -= 20)
    {
        control_servo(j);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void wakeUp(void)
{

    const int ext_wakeup_pin_0 = 27;

    printf("Enabling EXT0 wakeup on pin GPIO%d\n", ext_wakeup_pin_0);
    esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 0);

    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    rtc_gpio_pullup_dis(ext_wakeup_pin_0);
    rtc_gpio_pulldown_en(ext_wakeup_pin_0);
}

static void apagar(void)
{
    esp_deep_sleep_start();
    ESP_LOGI(TAG, "Deep sleep");
}

void app_main(void)
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 1, wifi, 14, false);

    init_oled();
    init_mqtt();
    init_servo();
    wakeUp();

    
    timer_init(TIMER_GROUP_0,TIMER_0,&config);
    
    char lineChar[14];

    int bitmapWidth = 4 * 8;
    int width = ssd1306_get_width(&dev);
    int xpos = width / 2; // centro del eje X
    xpos = xpos - bitmapWidth / 2;
    int height = ssd1306_get_height(&dev);
    int ypos = height / 2; // centro del eje Y

    ssd1306_clear_screen(&dev, false);
    timer_start(TIMER_GROUP_0,0);

    while (1)
    {
        
        timer_get_counter_value(TIMER_GROUP_0, 0, &task_counter_value);
        if (mClient)
        {
            carga = adc1_get_raw(adc_carga); // Sensor de carga
            sendData(mClient, carga);
            printf("INF:%d\r\nCARGA:%d\r\n",INF,carga);

            lineChar[0] = 0x00;
            sprintf(&lineChar[1], "Comida: %d", FOOD);
            ssd1306_display_text(&dev, 1, lineChar, 10, false);

            ssd1306_bitmaps(&dev, xpos, ypos, etsisi, 32, 29, true);

            if (!gpio_get_level(INF) && carga > 3400) // Si se detecta un animal
            {
                printf("ANIMAL DETECTADO\n");
                timer_set_counter_value(TIMER_GROUP_0,0,0);
                // wakeUp();
                FOOD = FOOD + 1;
                ssd1306_bitmaps(&dev, 5, ypos, circulo[1], 32, 27, true);
                // Gira motor
                giro_motor();
                ssd1306_display_text(&dev, 1, lineChar, 10, false);

                // vTaskDelay(10000 / portTICK_PERIOD_MS);
                ssd1306_bitmaps(&dev, 5, ypos, circulo[0], 32, 27, true);
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        else
        {
            ssd1306_display_text(&dev, 1, wifi, 14, false);
        }

        // hacer con timer
                        
        if (task_counter_value > 300000000) 
        {
            ssd1306_clear_screen(&dev, false);
            apagar();
        }
    }
}

esp_err_t setup(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_carga, ADC_ATTEN_DB_11);
    return ESP_OK;
}
