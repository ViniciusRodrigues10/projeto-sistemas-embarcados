#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>
#include <math.h>
#include <stdlib.h>

// ============================================================
// --- CONFIGURAÇÃO ESP32-S3 ---
// ============================================================

#define PIN_JOY_HORIZ 4 
#define PIN_JOY_VERT  5  
#define ADC_CHAN_HORIZ ADC_CHANNEL_3 // GPIO 4 é Canal 3 no ADC1 do S3
#define ADC_CHAN_VERT  ADC_CHANNEL_4 // GPIO 5 é Canal 4 no ADC1 do S3

// Pinos dos Servos
#define PIN_SERVO_X GPIO_NUM_18
#define PIN_SERVO_Y GPIO_NUM_19

// --- Configuração PWM ---
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_X LEDC_CHANNEL_0
#define LEDC_CHANNEL_Y LEDC_CHANNEL_1
#define LEDC_FREQUENCY 50 
#define LEDC_RESOLUTION LEDC_TIMER_13_BIT 

// --- Configuração ADC ---
#define ADC_UNIT ADC_UNIT_1
#define ADC_MAX_VAL 4095

// --- Constantes do Servo ---
#define SERVO_PERIOD_US 20000 
#define MIN_PULSE_US 500  
#define MAX_PULSE_US 2500 
#define MIN_ANGLE 40
#define MAX_ANGLE 140

// --- Filtros e Ajustes ---
// Zona morta reduzida para 50 para ficar mais responsivo
#define JOY_DEADZONE 50 
#define ADC_CENTER 2048 
// Filtro Suave (0.10)
#define FILTER_ALPHA 0.10

// Tempos das Tasks
#define TASK1_DELAY_MS 20 
#define TASK2_DELAY_MS 20 
#define TASK3_DELAY_MS 200

// --- Globais ---
SemaphoreHandle_t xServoDataMutex;
float smoothed_x = ADC_CENTER;
float smoothed_y = ADC_CENTER;
uint32_t servo_duty_x = 0; 
uint32_t servo_duty_y = 0; 
adc_oneshot_unit_handle_t adc_handle;

// --- Protótipos ---
void task_joystick_read(void *pvParameters);
void task_servo_control(void *pvParameters);
void task_monitor(void *pvParameters);

long map_value(float x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

static void init_ledc() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION, .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_x = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_X,
        .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_X, .duty = 0, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_x));

    ledc_channel_config_t ledc_channel_y = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_Y,
        .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_Y, .duty = 0, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_y));
}

static void init_adc() {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11, 
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN_HORIZ, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN_VERT, &config));
}

void app_main(void) {
    printf("--- Iniciando com Pinos Corretos (GPIO 4 e 5) ---\n");
    xServoDataMutex = xSemaphoreCreateMutex();
    init_ledc();
    init_adc();

    xTaskCreate(task_joystick_read, "JoyRead", 2048, NULL, 2, NULL);
    xTaskCreate(task_servo_control, "ServoCtrl", 2048, NULL, 3, NULL);
    // Aumentado stack para 4096 para evitar crash no printf
    xTaskCreate(task_monitor, "Monitor", 4096, NULL, 1, NULL);
}

void task_joystick_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        int raw_x, raw_y;
        float current_x, current_y;
        
        adc_oneshot_read(adc_handle, ADC_CHAN_HORIZ, &raw_x);
        adc_oneshot_read(adc_handle, ADC_CHAN_VERT, &raw_y);

        // Filtro
        smoothed_x = (FILTER_ALPHA * raw_x) + ((1.0 - FILTER_ALPHA) * smoothed_x);
        smoothed_y = (FILTER_ALPHA * raw_y) + ((1.0 - FILTER_ALPHA) * smoothed_y);
        
        current_x = smoothed_x;
        current_y = smoothed_y;

        // Deadzone
        if (fabs(current_x - ADC_CENTER) < JOY_DEADZONE) current_x = ADC_CENTER;
        if (fabs(current_y - ADC_CENTER) < JOY_DEADZONE) current_y = ADC_CENTER;

        long angle_x = map_value(current_x, 0, ADC_MAX_VAL, MIN_ANGLE, MAX_ANGLE);
        long angle_y = map_value(current_y, 0, ADC_MAX_VAL, MIN_ANGLE, MAX_ANGLE);
        
        long pulse_x = map_value(angle_x, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
        long pulse_y = map_value(angle_y, 0, 180, MIN_PULSE_US, MAX_PULSE_US);

        uint32_t duty_x = (pulse_x * (1 << LEDC_RESOLUTION)) / SERVO_PERIOD_US;
        uint32_t duty_y = (pulse_y * (1 << LEDC_RESOLUTION)) / SERVO_PERIOD_US;

        if (xSemaphoreTake(xServoDataMutex, portMAX_DELAY) == pdTRUE) {
            servo_duty_x = duty_x;
            servo_duty_y = duty_y;
            xSemaphoreGive(xServoDataMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK1_DELAY_MS));
    }
}

void task_servo_control(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dx = 0, dy = 0;
        if (xSemaphoreTake(xServoDataMutex, portMAX_DELAY) == pdTRUE) {
            dx = servo_duty_x;
            dy = servo_duty_y;
            xSemaphoreGive(xServoDataMutex);
        }
        if (dx > 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, dx);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
        }
        if (dy > 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, dy);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK2_DELAY_MS));
    }
}

void task_monitor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dx = 0, dy = 0;
        float sx = 0, sy = 0;
        if (xSemaphoreTake(xServoDataMutex, portMAX_DELAY) == pdTRUE) {
            dx = servo_duty_x;
            dy = servo_duty_y;
            sx = smoothed_x;
            sy = smoothed_y;
            xSemaphoreGive(xServoDataMutex);
        }
        long ang_x = map_value((dx * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);
        long ang_y = map_value((dy * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);

        printf("ADC: %.0f, %.0f | ANGLE: %ld, %ld\n", sx, sy, ang_x, ang_y);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK3_DELAY_MS));
    }
}