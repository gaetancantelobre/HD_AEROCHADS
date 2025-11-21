#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "nvs_flash.h" 

// --- WS2812B Includes/Config ---
#include "driver/rmt.h"
#include "led_strip.h" // Requires the 'led_strip' component
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_STRIP_GPIO 5        // *** CHECK THIS: GPIO pin for WS2812B Data ***
#define LED_STRIP_LENGTH 25      // 5 columns * 5 rows

// --- Wi-Fi Configuration ---
#define WIFI_SSID      "OTR5B" 
#define WIFI_PASS      "5bruedeschenes" 
#define TCP_PORT       3333
#define RX_BUF_SIZE    5              // Expecting 5 bytes (uint8_t: servo angle)

// --- PCA9685/I2C Configuration ---
#define TAG "PCA9685_TCP"
#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define I2C_MASTER_TIMEOUT_MS 1000

// Event group to signal when we are connected
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG_WIFI = "WIFI";
static const char *TAG_SERVER = "TCP_SERVER";

// WS2812B Handle
static led_strip_t *strip;

// --- PCA9685 Servo Helpers ---

static void pca9685_write(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_write failed (0x%X): %s", reg, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
}

static void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = 0x06 + 4 * channel;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_write_byte(cmd, on & 0xFF, true);
    i2c_master_write_byte(cmd, on >> 8, true);
    i2c_master_write_byte(cmd, off & 0xFF, true);
    i2c_master_write_byte(cmd, off >> 8, true);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_set_pwm failed for channel %d: %s", channel, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
}

// Map 0-180 angle to PCA9685 12-bit counts (102-512) for 50Hz PWM
static uint16_t angle_to_pwm(int angle) {
    const int MIN_COUNT = 102; // 0 degrees (approx 500µs)
    const int MAX_COUNT = 512; // 180 degrees (approx 2500µs)
    const int RANGE = MAX_COUNT - MIN_COUNT;

    // Clamp to 0-180 range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    float pwm_float = MIN_COUNT + ((float)angle / 180.0f) * RANGE;
    
    return (uint16_t)pwm_float;
}

// --- WS2812B LED Helpers ---

/**
 * @brief Maps an angle (0-180) to an RGB color using a Red-to-Green gradient.
 * Red (180 deg) -> Yellow (90 deg) -> Green (0 deg)
 */
void angle_to_rgb(int angle, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Clamp to 0-180
    angle = (angle < 0) ? 0 : ((angle > 180) ? 180 : angle);

    uint8_t brightness = 255; 

    if (angle <= 90) { // Green (0) to Yellow (90)
        *r = (uint8_t)((float)angle / 90.0f * brightness); // R goes from 0 to 255
        *g = brightness; // G is full
        *b = 0;
    } else { // Yellow (90) to Red (180)
        *r = brightness; // R is full
        *g = (uint8_t)((float)(180 - angle) / 90.0f * brightness); // G goes from 255 to 0
        *b = 0;
    }
}

void led_strip_init() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_STRIP_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_STRIP_LENGTH);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "LED strip initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d. Total %d LEDs.", LED_STRIP_GPIO, LED_STRIP_LENGTH);
    // Clear all LEDs initially
    ESP_ERROR_CHECK(strip->clear(strip, 100));
}

// --- Wi-Fi Event Handler ---

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Wi-Fi disconnected. Reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        
        ESP_LOGI(TAG_SERVER, "Socket Server listening on Port %d", TCP_PORT);
        ESP_LOGI(TAG_SERVER, "Client should connect to IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- Wi-Fi Initialization ---

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Wi-Fi initialization finished. Waiting for connection...");
    
    // Wait until the WIFI_CONNECTED_BIT is set
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// --- PCA9685 Initialization ---

void pca9685_init() {
    // ---- I2C Initialize ----
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // ---- PCA9685 Initialize (50 Hz PWM) ----
    uint8_t prescale = (uint8_t)((25000000.0f / (4096 * 50.0f)) - 1.0f); 

    pca9685_write(MODE1, 0x00); 
    vTaskDelay(10 / portTICK_PERIOD_MS);
    pca9685_write(MODE1, 0x10); 
    pca9685_write(PRESCALE, prescale);
    pca9685_write(MODE1, 0x00); 
    vTaskDelay(10 / portTICK_PERIOD_MS);
    pca9685_write(MODE1, 0xA0); 
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Init all 5 target servos (channels 0-4) to 0 degrees
    for (int i = 0; i < 5; i++) {
        uint16_t pwm_init = angle_to_pwm(0);
        pca9685_set_pwm(i, 0, pwm_init);
        ESP_LOGI(TAG, "Initialized Channel %d to 0 degrees (PWM=%d)", i, pwm_init);
    }
}

// --- TCP Server Task ---

static void tcp_server_task(void *pvParameters) {
    char rx_buffer[RX_BUF_SIZE];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    
    // 1. Create Socket Address and Bind (Standard TCP server setup)
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TCP_PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG_SERVER, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG_SERVER, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG_SERVER, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_SERVER, "Socket listening, waiting for client...");

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        
        // Accept Connection
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addrlen);
        if (client_sock < 0) {
            ESP_LOGE(TAG_SERVER, "Error accepting connection: errno %d", errno);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG_SERVER, "New client connected");

        // Receive Data Loop
        while (1) {
            // Receive exactly 5 bytes (uint8_t)
            int len = recv(client_sock, rx_buffer, RX_BUF_SIZE, MSG_WAITALL); 

            if (len == 0) {
                ESP_LOGI(TAG_SERVER, "Connection closed by client");
                break;
            } else if (len < 0) {
                ESP_LOGE(TAG_SERVER, "Error receiving data: errno %d", errno);
                break;
            } else if (len == RX_BUF_SIZE) {
                // Successfully received 5 bytes
                uint8_t *servo_values = (uint8_t *)rx_buffer;

                ESP_LOGI(TAG_SERVER, "Received 5 values: [%d, %d, %d, %d, %d]", 
                         servo_values[0], servo_values[1], servo_values[2], servo_values[3], servo_values[4]);

                // 7. Apply Servo and LED Values
                for (int i = 0; i < RX_BUF_SIZE; i++) {
                    int angle = (int)servo_values[i];
                    uint16_t pwm_count = angle_to_pwm(angle);
                    
                    // A. Apply to Servos (Channels 0 through 4)
                    pca9685_set_pwm(i, 0, pwm_count);

                    // B. Apply to LED Matrix (Column i)
                    uint8_t r, g, b;
                    angle_to_rgb(angle, &r, &g, &b);
                    
                    // Map 0-180 angle to a height of 0 to 5 LEDs
                    // The division by 180.0f ensures a float result between 0 and 1
                    // The multiplication by 5.0f scales it to a height between 0 and 5
                    int led_height = (int)(((float)angle / 180.0f) * 5.0f);
                    led_height = (led_height < 0) ? 0 : ((led_height > 5) ? 5 : led_height);

                    // Iterate through the 5 LEDs in the current column (i)
                    for (int j = 0; j < 5; j++) {
                        // Assuming the strip is wired vertically (0-4 in col 0, 5-9 in col 1, etc.)
                        int led_idx = (i * 5) + j; 
                        
                        if (j < led_height) {
                            // Turn ON the LED with the calculated color
                            ESP_ERROR_CHECK(strip->set_pixel(strip, led_idx, r, g, b));
                        } else {
                            // Turn OFF the remaining LEDs
                            ESP_ERROR_CHECK(strip->set_pixel(strip, led_idx, 0, 0, 0));
                        }
                    }
                }
                // Push the new colors to the hardware
                ESP_ERROR_CHECK(strip->refresh(strip, 100));

            } else {
                 ESP_LOGW(TAG_SERVER, "Received unexpected length: %d (expected %d)", len, RX_BUF_SIZE);
            }
        }
        // Close Client Socket
        close(client_sock);
    }
    close(listen_sock);
    vTaskDelete(NULL);
}


void app_main() {
    // Initialize NVS (Needed for Wi-Fi configuration storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Initialize PCA9685 and Servos
    pca9685_init();

    // 2. Initialize WS2812B LED strip
    led_strip_init();

    // 3. Connect to Wi-Fi (blocks until connected)
    wifi_init_sta();

    // 4. Start TCP Server Task
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}