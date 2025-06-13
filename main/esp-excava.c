#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

static const char *TAG = "esp-excava";

#define BLINK_GPIO 2 // GPIO pin for LED

// TB6612FNG motor driver pins
#define MOTOR_TRACK_L_IN1 23
#define MOTOR_TRACK_L_IN2 22
#define MOTOR_TRACK_L_PWM 15

#define MOTOR_TRACK_R_IN1 21
#define MOTOR_TRACK_R_IN2 19
#define MOTOR_TRACK_R_PWM 4

// LEDC PWM configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution, 0-1023 duty values
#define LEDC_FREQUENCY 5000             // 5kHz PWM frequency
#define LEDC_CHANNEL_LEFT_PWM LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT_PWM LEDC_CHANNEL_1

static uint8_t s_led_state = 0;

typedef struct
{
    uint8_t gpio_in1;
    uint8_t gpio_in2;
    uint8_t gpio_pwm;
    uint8_t channel;
} bidirectional_motor_t;

static const bidirectional_motor_t left_motor = {
    .gpio_in1 = MOTOR_TRACK_L_IN1,
    .gpio_in2 = MOTOR_TRACK_L_IN2,
    .gpio_pwm = MOTOR_TRACK_L_PWM,
    .channel = LEDC_CHANNEL_LEFT_PWM};
static const bidirectional_motor_t right_motor = {
    .gpio_in1 = MOTOR_TRACK_R_IN1,
    .gpio_in2 = MOTOR_TRACK_R_IN2,
    .gpio_pwm = MOTOR_TRACK_R_PWM,
    .channel = LEDC_CHANNEL_RIGHT_PWM};

// Function to initialize motor control pins and PWM
static void motor_init(const bidirectional_motor_t *left_motor, const bidirectional_motor_t *right_motor)
{
    // Configure direction control pins as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_TRACK_L_IN1) | (1ULL << MOTOR_TRACK_L_IN2) |
                        (1ULL << MOTOR_TRACK_R_IN1) | (1ULL << MOTOR_TRACK_R_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set initial direction pins to LOW (motor stopped)
    gpio_set_level(left_motor->gpio_in1, 0);
    gpio_set_level(left_motor->gpio_in2, 0);
    gpio_set_level(right_motor->gpio_in1, 0);
    gpio_set_level(right_motor->gpio_in2, 0);

    // Prepare and apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_track_left = {
        .speed_mode = LEDC_MODE,
        .channel = left_motor->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = left_motor->gpio_pwm,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_track_left));

    ledc_channel_config_t ledc_channel_track_right = {
        .speed_mode = LEDC_MODE,
        .channel = right_motor->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = right_motor->gpio_pwm,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_track_right));
}

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = 2
} motor_direction_t;

static void bidirectional_motor_control(const bidirectional_motor_t *motor, motor_direction_t direction, uint32_t speed)
{
    // Ensure speed is within range (0-1023 for 10-bit resolution)
    if (speed > 1023)
    {
        speed = 1023;
    }

    ESP_LOGI(TAG, "Motor: direction=%d, speed=%lu", direction, speed);

    // Set direction pins based on desired direction
    switch (direction)
    {
    case MOTOR_FORWARD:
        gpio_set_level(motor->gpio_in1, 1);
        gpio_set_level(motor->gpio_in2, 0);
        break;

    case MOTOR_BACKWARD:
        gpio_set_level(motor->gpio_in1, 0);
        gpio_set_level(motor->gpio_in2, 1);
        break;

    case MOTOR_STOP:
    default:
        gpio_set_level(motor->gpio_in1, 0);
        gpio_set_level(motor->gpio_in2, 0);
        break;
    }

    // Set PWM speed
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, motor->channel, speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, motor->channel));
}

// Function to test motor operations (individual and combined movements)
static void motor_test_sequence(const bidirectional_motor_t *left_motor, const bidirectional_motor_t *right_motor)
{
    ESP_LOGI(TAG, "Starting comprehensive motor test sequence...");

    // Stop both motors first
    ESP_LOGI(TAG, "Both motors: STOP");
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // =============== INDIVIDUAL MOTOR TESTS ===============

    // Test left track individually
    ESP_LOGI(TAG, "LEFT TRACK TEST: Forward with increasing speed");
    for (int i = 200; i <= 1000; i += 200)
    {
        bidirectional_motor_control(left_motor, MOTOR_FORWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "LEFT TRACK TEST: Backward with increasing speed");
    for (int i = 200; i <= 1000; i += 200)
    {
        bidirectional_motor_control(left_motor, MOTOR_BACKWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Stop left motor
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Test right track individually
    ESP_LOGI(TAG, "RIGHT TRACK TEST: Forward with increasing speed");
    for (int i = 200; i <= 1000; i += 200)
    {
        bidirectional_motor_control(right_motor, MOTOR_FORWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "RIGHT TRACK TEST: Backward with increasing speed");
    for (int i = 200; i <= 1000; i += 200)
    {
        bidirectional_motor_control(right_motor, MOTOR_BACKWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Stop right motor
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // =============== COMBINED MOVEMENT TESTS ===============

    // Both tracks forward (straight line movement)
    ESP_LOGI(TAG, "COMBINED TEST: Both tracks forward (straight line)");
    for (int i = 200; i <= 800; i += 200)
    {
        bidirectional_motor_control(left_motor, MOTOR_FORWARD, i);
        bidirectional_motor_control(right_motor, MOTOR_FORWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Both tracks backward (straight line movement)
    ESP_LOGI(TAG, "COMBINED TEST: Both tracks backward (straight line)");
    for (int i = 200; i <= 800; i += 200)
    {
        bidirectional_motor_control(left_motor, MOTOR_BACKWARD, i);
        bidirectional_motor_control(right_motor, MOTOR_BACKWARD, i);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // =============== TURNING TESTS ===============

    // Turn right (left track forward, right track stopped)
    ESP_LOGI(TAG, "TURNING TEST: Turn right (pivot)");
    bidirectional_motor_control(left_motor, MOTOR_FORWARD, 600);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Turn left (right track forward, left track stopped)
    ESP_LOGI(TAG, "TURNING TEST: Turn left (pivot)");
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_FORWARD, 600);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Spin right (left track forward, right track backward)
    ESP_LOGI(TAG, "TURNING TEST: Spin right (counter-rotation)");
    bidirectional_motor_control(left_motor, MOTOR_FORWARD, 600);
    bidirectional_motor_control(right_motor, MOTOR_BACKWARD, 600);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Spin left (left track backward, right track forward)
    ESP_LOGI(TAG, "TURNING TEST: Spin left (counter-rotation)");
    bidirectional_motor_control(left_motor, MOTOR_BACKWARD, 600);
    bidirectional_motor_control(right_motor, MOTOR_FORWARD, 600);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // =============== SLIP SIMULATION TESTS ===============

    // Simulate slippage by running tracks at different speeds
    ESP_LOGI(TAG, "SLIP TEST: Left track faster than right");
    bidirectional_motor_control(left_motor, MOTOR_FORWARD, 800);
    bidirectional_motor_control(right_motor, MOTOR_FORWARD, 400);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Right track faster (gentle left turn)
    ESP_LOGI(TAG, "SLIP TEST: Right track faster than left");
    bidirectional_motor_control(left_motor, MOTOR_FORWARD, 400);
    bidirectional_motor_control(right_motor, MOTOR_FORWARD, 800);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Simulate slippage with quick speed transitions
    ESP_LOGI(TAG, "SLIP TEST: Quick speed transitions (left track)");
    bidirectional_motor_control(right_motor, MOTOR_FORWARD, 600);

    for (int i = 0; i < 5; i++)
    {
        bidirectional_motor_control(left_motor, MOTOR_FORWARD, 900);
        vTaskDelay(pdMS_TO_TICKS(500));
        bidirectional_motor_control(left_motor, MOTOR_FORWARD, 300);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Stop both motors
    bidirectional_motor_control(left_motor, MOTOR_STOP, 0);
    bidirectional_motor_control(right_motor, MOTOR_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor test sequence completed");
}

#define LED_ON_TIME_MS 100   // 100ms
#define LED_OFF_TIME_MS 1000 // 1000ms

static TaskHandle_t strobe_task_handle = NULL;

static void strobe_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Strobe LED task started");

    while (1)
    {
        // Turn LED on
        gpio_set_level(BLINK_GPIO, 1);
        s_led_state = 1;
        vTaskDelay(pdMS_TO_TICKS(LED_ON_TIME_MS));

        // Turn LED off
        gpio_set_level(BLINK_GPIO, 0);
        s_led_state = 0;
        vTaskDelay(pdMS_TO_TICKS(LED_OFF_TIME_MS));
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS (Non-Volatile Storage)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Starting ESP Excavation...");

    // Initialize LED pin
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize motor control
    motor_init(&left_motor, &right_motor);

    // Create the strobe LED task
    xTaskCreate(strobe_led_task, "strobe_led", 2048, NULL, 5, &strobe_task_handle);
    if (strobe_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create strobe LED task");
    }
    else
    {
        ESP_LOGI(TAG, "Strobe LED task created successfully");
    }

    // Run the comprehensive motor test sequence
    ESP_LOGI(TAG, "Running comprehensive motor test sequence...");
    motor_test_sequence(&left_motor, &right_motor);

    // Main loop
    while (1)
    {
        ESP_LOGI(TAG, "ESP Excavation is running...");

        vTaskDelay(pdMS_TO_TICKS(5000)); // 5-second delay between LED toggles
        motor_test_sequence(&left_motor, &right_motor);
    }
}
