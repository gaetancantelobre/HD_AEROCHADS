#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h" // LEDC driver for Servo PWM
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h" // Retained for basic ESP-IDF app initialization

// --- I2C and IMU Defines ---
#define I2C_MASTER_SCL_IO    5
#define I2C_MASTER_SDA_IO    4
#define I2C_MASTER_FREQ_HZ   400000

#define LSM6DSL_ADDR         0x6B
#define LSM6DSL_WHO_AM_I_REG 0x0F
#define LSM6DSL_CTRL1_XL     0x10
#define LSM6DSL_CTRL2_G      0x11
#define LSM6DSL_OUTX_L_XL    0x28
#define OUTX_L_G             0x22

// --- Servo Defines ---
#define SERVO_GPIO           3        // GPIO 3 used for PWM output
#define SERVO_MIN_PULSE_US   500      // 0 degrees
#define SERVO_MAX_PULSE_US   2500     // 180 degrees
#define SERVO_FREQ_HZ        50       // Standard servo frequency
#define SERVO_DUTY_RES       LEDC_TIMER_13_BIT // Resolution bits

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t lsm6dsl_dev = NULL;
static ledc_channel_config_t ledc_channel = {};
static const char *TAG = "SERVO_IMU";

static float pitch_accel = 0, roll_accel = 0;
static float pitch_kf = 0, roll_kf = 0;
static bool use_kalman = true; // Use Kalman filter by default

// === Kalman filter struct ===
typedef struct {
    float Q_angle;
    float Q_bias;
    float R;
    float angle;
    float bias;
    float P[2][2];
} KalmanFilter;

static KalmanFilter kalman_pitch, kalman_roll;

static void kalman_init(KalmanFilter *kf, float Q_angle, float Q_bias, float R, float init_angle) {
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R = R;
    kf->angle = init_angle;
    kf->bias = 0;
    memset(kf->P, 0, sizeof(kf->P));
}

static float kalman_update(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // Predict
    kf->angle += (newRate - kf->bias) * dt;
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Update
    float S = kf->P[0][0] + kf->R;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    float y = newAngle - kf->angle;
    kf->angle += K0 * y;
    kf->bias  += K1 * y;
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;
    return kf->angle;
}

// === I2C & LSM6DSL ===
static void i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .device_address = LSM6DSL_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lsm6dsl_dev));
}

static esp_err_t lsm6dsl_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(lsm6dsl_dev, buf, sizeof(buf), -1);
}

static esp_err_t lsm6dsl_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(lsm6dsl_dev, &reg, 1, data, len, -1);
}

static void lsm6dsl_init(void) {
    uint8_t whoami = 0;
    lsm6dsl_read(LSM6DSL_WHO_AM_I_REG, &whoami, 1);
    ESP_LOGI(TAG, "LSM6DSL WHO_AM_I = 0x%02X (Expected: 0x6A)", whoami);
    lsm6dsl_write(LSM6DSL_CTRL1_XL, 0x40); // Accel: 104 Hz, ±2g FS
    lsm6dsl_write(LSM6DSL_CTRL2_G,  0x40); // Gyro: 104 Hz, 245 dps FS
}

static void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];
    // Read 6 bytes starting from LSM6DSL_OUTX_L_XL (0x28)
    lsm6dsl_read(LSM6DSL_OUTX_L_XL, data, 6);
    *ax = (int16_t)(data[1] << 8 | data[0]);
    *ay = (int16_t)(data[3] << 8 | data[2]);
    *az = (int16_t)(data[5] << 8 | data[4]);
}

static void read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t data[6];
    // Read 6 bytes starting from OUTX_L_G (0x22)
    lsm6dsl_read(OUTX_L_G, data, 6);
    *gx = (int16_t)(data[1] << 8 | data[0]);
    *gy = (int16_t)(data[3] << 8 | data[2]);
    *gz = (int16_t)(data[5] << 8 | data[4]);
}

// === Servo Control Functions (LEDC PWM) ===
static void servo_init(void) {
    // 1. Prepare and apply the LEDC Timer configuration (50Hz)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = SERVO_DUTY_RES,    // Use defined resolution
        .freq_hz          = SERVO_FREQ_HZ,     // 50 Hz PWM
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Prepare and apply the LEDC Channel configuration
    ledc_channel = (ledc_channel_config_t){
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "Servo initialized on GPIO%d with 50Hz PWM.", SERVO_GPIO);
}

// Helper to calculate the duty cycle from a pulse width in microseconds
static uint32_t us_to_duty(uint32_t us) {
    // Calculate the duty value required for the given pulse width (us)
    // Duty = (pulse_width * resolution * frequency) / 1,000,000
    // Fix: Use the defined SERVO_DUTY_RES constant directly instead of accessing the struct
    uint64_t duty_us = (uint64_t)us * (1 << SERVO_DUTY_RES) * SERVO_FREQ_HZ / 1000000;
    return (uint32_t)duty_us;
}

// Sets the servo angle (0-180 degrees)
static void servo_set_angle(float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    // Linear mapping from 0-180 degrees to SERVO_MIN_PULSE_US to SERVO_MAX_PULSE_US
    uint32_t pulse_width = (uint32_t)(
        SERVO_MIN_PULSE_US + 
        (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (angle_deg / 180.0f)
    );

    uint32_t duty = us_to_duty(pulse_width);
    
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

// === IMU Task: Main control loop ===
static void imu_task(void *arg) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t_prev = esp_timer_get_time();

    // Initial position: Set servo to a starting angle (e.g., 90 degrees)
    servo_set_angle(90.0f);
    vTaskDelay(pdMS_TO_TICKS(500)); 

    while (1) {
        read_accel(&ax, &ay, &az);
        read_gyro(&gx, &gy, &gz);
        
        // Convert raw readings to physical units (g and dps)
        float ax_g = ax * 0.000061f; // Accel sensitivity for +/-2g FS
        float ay_g = ay * 0.000061f;
        float az_g = az * 0.000061f;
        float gx_dps = gx * 0.00875f; // Gyro sensitivity for +/-245dps FS
        float gy_dps = gy * 0.00875f;

        // Calculate Accel-only angles for Kalman input
        pitch_accel = atan2f(ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;
        roll_accel  = atan2f(ay_g, sqrtf(ax_g*ax_g + az_g*az_g)) * 180.0f / M_PI;

        // Calculate time step (dt)
        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) / 1e6f;
        t_prev = t_now;

        // Update Kalman filters (using gyro X/Y for Pitch/Roll rate)
        pitch_kf = kalman_update(&kalman_pitch, pitch_accel, gx_dps, dt);
        roll_kf  = kalman_update(&kalman_roll,  roll_accel,  gy_dps, dt);

        // --- Servo Control Logic ---
        float acc_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        float target_angle = 90.0f; // Default center position
        float current_roll = use_kalman ? roll_kf : roll_accel;
        
        // 1. Free Fall Detection: opens the servo completely
        if (acc_mag < 0.2f) { // If acceleration magnitude is below 0.2g
            target_angle = 180.0f; // Fully open/deployed
            ESP_LOGW(TAG, "!!! FREE FALL DETECTED !!! Mag: %.2f g. Servo: 180.0 deg", acc_mag);
        } 
        // 2. General Tilt Control: uses filtered roll angle for slight movement
        else {
            // Clamp the roll angle to +/- 90 degrees
            if (current_roll > 90.0f) current_roll = 90.0f;
            if (current_roll < -90.0f) current_roll = -90.0f;
            
            // Map angle [-90, 90] to servo range [45, 135] (90 degrees of "slight movement")
            // Map from 0 to 180 (for the roll angle + 90) -> 0 to 90 (for the servo range)
            target_angle = (current_roll + 90.0f) * (90.0f / 180.0f) + 45.0f;
            
            ESP_LOGI(TAG, "Tilt Control. Roll (KF): %.1f, Mag: %.2f g. Servo: %.1f deg", current_roll, acc_mag, target_angle);
        }

        servo_set_angle(target_angle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    // Initialize Non-Volatile Storage (required by many ESP-IDF components)
    ESP_ERROR_CHECK(nvs_flash_init()); 
    
    // Initialize Hardware
    i2c_master_init();
    lsm6dsl_init();
    servo_init(); // Initialize the servo motor on GPIO 3

    // Initialize Kalman filters with reasonable defaults
    // (Q_angle=process noise for angle, Q_bias=process noise for bias, R=measurement noise)
    kalman_init(&kalman_pitch, 0.05f, 0.009f, 1.3f, 0);
    kalman_init(&kalman_roll,  0.05f, 0.009f, 1.3f, 0);

    // Start the main IMU processing and servo control task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}