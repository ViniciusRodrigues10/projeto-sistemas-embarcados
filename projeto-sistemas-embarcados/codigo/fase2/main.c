#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// --- CONFIGURAÇÃO DE PINOS (ESP32-S3) ---
// ============================================================

// Joystick
#define PIN_JOY_HORIZ 4 
#define PIN_JOY_VERT  5  
#define ADC_CHAN_HORIZ ADC_CHANNEL_3 
#define ADC_CHAN_VERT  ADC_CHANNEL_4 

// Servos
#define PIN_SERVO_X GPIO_NUM_18
#define PIN_SERVO_Y GPIO_NUM_19

// MPU6050 (I2C)
#define I2C_MASTER_SCL_IO 9       // Pino SCL do ESP32-S3
#define I2C_MASTER_SDA_IO 8       // Pino SDA do ESP32-S3
#define I2C_MASTER_NUM 0          // Porta I2C numero 0
#define I2C_MASTER_FREQ_HZ 100000 // Frequencia 100kHz
#define MPU6050_ADDR 0x68         // Endereço I2C padrão do MPU6050

// ============================================================
// --- CONSTANTES E DEFINIÇÕES ---
// ============================================================

// PWM Servos
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_X LEDC_CHANNEL_0
#define LEDC_CHANNEL_Y LEDC_CHANNEL_1
#define LEDC_FREQUENCY 50 
#define LEDC_RESOLUTION LEDC_TIMER_13_BIT 
#define SERVO_PERIOD_US 20000 
#define MIN_PULSE_US 500  
#define MAX_PULSE_US 2500 
#define MIN_ANGLE_SERVO 40
#define MAX_ANGLE_SERVO 140

// ADC Joystick
#define ADC_UNIT ADC_UNIT_1
#define ADC_MAX_VAL 4095
#define JOY_DEADZONE 50 
#define ADC_CENTER 2048 
#define FILTER_ALPHA 0.10

// MPU6050 Registradores
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define RAD_TO_DEG 57.2957795131

// Tempos das Tasks (ms)
#define TASK_JOY_DELAY 20 
#define TASK_SERVO_DELAY 20 
#define TASK_MPU_DELAY 50  // Leitura do sensor a 20Hz
#define TASK_MONITOR_DELAY 200

// ============================================================
// --- VARIÁVEIS GLOBAIS E HANDLES ---
// ============================================================

SemaphoreHandle_t xDataMutex; // Protege dados compartilhados

// Dados Joystick/Servo
float smoothed_x = ADC_CENTER;
float smoothed_y = ADC_CENTER;
uint32_t servo_duty_x = 0; 
uint32_t servo_duty_y = 0; 

// Dados MPU6050
float mpu_pitch = 0.0;
float mpu_roll = 0.0;
int16_t raw_accel_x = 0, raw_accel_y = 0, raw_accel_z = 0;
int16_t raw_gyro_x = 0, raw_gyro_y = 0, raw_gyro_z = 0;

adc_oneshot_unit_handle_t adc_handle;

// ============================================================
// --- FUNÇÕES AUXILIARES ---
// ============================================================

long map_value(float x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Inicializa I2C
static esp_err_t init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Acorda o MPU6050
static void init_mpu6050() {
    uint8_t data[2];
    data[0] = MPU6050_PWR_MGMT_1;
    data[1] = 0x00; // Zera o registro de Power Management para acordar o sensor
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
}

// Inicializa PWM
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

// Inicializa ADC
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

// ============================================================
// --- TASKS (TAREFAS) ---
// ============================================================

// Task 1: Leitura do Joystick
void task_joystick_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        int raw_x, raw_y;
        float current_x, current_y;
        
        adc_oneshot_read(adc_handle, ADC_CHAN_HORIZ, &raw_x);
        adc_oneshot_read(adc_handle, ADC_CHAN_VERT, &raw_y);

        smoothed_x = (FILTER_ALPHA * raw_x) + ((1.0 - FILTER_ALPHA) * smoothed_x);
        smoothed_y = (FILTER_ALPHA * raw_y) + ((1.0 - FILTER_ALPHA) * smoothed_y);
        
        current_x = smoothed_x;
        current_y = smoothed_y;

        if (fabs(current_x - ADC_CENTER) < JOY_DEADZONE) current_x = ADC_CENTER;
        if (fabs(current_y - ADC_CENTER) < JOY_DEADZONE) current_y = ADC_CENTER;

        long angle_x = map_value(current_x, 0, ADC_MAX_VAL, MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        long angle_y = map_value(current_y, 0, ADC_MAX_VAL, MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        
        long pulse_x = map_value(angle_x, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
        long pulse_y = map_value(angle_y, 0, 180, MIN_PULSE_US, MAX_PULSE_US);

        uint32_t duty_x = (pulse_x * (1 << LEDC_RESOLUTION)) / SERVO_PERIOD_US;
        uint32_t duty_y = (pulse_y * (1 << LEDC_RESOLUTION)) / SERVO_PERIOD_US;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            servo_duty_x = duty_x;
            servo_duty_y = duty_y;
            xSemaphoreGive(xDataMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_JOY_DELAY));
    }
}

// Task 2: Controle dos Servos
void task_servo_control(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dx = 0, dy = 0;
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            dx = servo_duty_x;
            dy = servo_duty_y;
            xSemaphoreGive(xDataMutex);
        }
        if (dx > 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, dx);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
        }
        if (dy > 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, dy);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_SERVO_DELAY));
    }
}

// Task 3: Leitura MPU6050 (NOVA)
void task_mpu_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t buffer[14]; // Buffer para ler 14 registradores de uma vez
    
    for (;;) {
        // Lê 14 bytes começando do registrador ACCEL_XOUT_H (0x3B)
        // Sequência: Accel(XYZ), Temp, Gyro(XYZ)
        uint8_t reg = MPU6050_ACCEL_XOUT_H;
        i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buffer, 14, 100 / portTICK_PERIOD_MS);

        // Combina bytes High e Low
        int16_t ax = (buffer[0] << 8) | buffer[1];
        int16_t ay = (buffer[2] << 8) | buffer[3];
        int16_t az = (buffer[4] << 8) | buffer[5];
        // Pula temperatura (buffer 6 e 7)
        int16_t gx = (buffer[8] << 8) | buffer[9];
        int16_t gy = (buffer[10] << 8) | buffer[11];
        int16_t gz = (buffer[12] << 8) | buffer[13];

        // Cálculo Básico de Pitch e Roll usando Acelerômetro (Estável para Tilt)
        // Pitch (Inclinação X) e Roll (Inclinação Y)
        float pitch_acc = atan2((-ax), sqrt(ay * ay + az * az)) * RAD_TO_DEG;
        float roll_acc = atan2(ay, az) * RAD_TO_DEG;

        // Atualiza variáveis globais protegidas
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            mpu_pitch = pitch_acc;
            mpu_roll = roll_acc;
            raw_accel_x = ax; raw_accel_y = ay; raw_accel_z = az;
            raw_gyro_x = gx; raw_gyro_y = gy; raw_gyro_z = gz;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MPU_DELAY));
    }
}

// Task 4: Monitoramento via Serial (Atualizado para JSON)
void task_monitor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dx = 0, dy = 0;
        float sx = 0, sy = 0;
        float p = 0, r = 0;
        int16_t gx = 0, gy = 0, gz = 0;

        // Coleta instantâneo seguro dos dados
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            dx = servo_duty_x;
            dy = servo_duty_y;
            sx = smoothed_x;
            sy = smoothed_y;
            p = mpu_pitch;
            r = mpu_roll;
            gx = raw_gyro_x; gy = raw_gyro_y; gz = raw_gyro_z;
            xSemaphoreGive(xDataMutex);
        }

        // Converte Duty para Angulo (Apenas para visualização)
        long ang_servo_x = map_value((dx * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);
        long ang_servo_y = map_value((dy * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);

        // Saída em JSON para facilitar o parsing no computador
        printf("{\"joy_x\":%.0f, \"joy_y\":%.0f, \"servo_x\":%ld, \"servo_y\":%ld, \"pitch\":%.2f, \"roll\":%.2f, \"gyro_x\":%d, \"gyro_y\":%d, \"gyro_z\":%d}\n",
               sx, sy, ang_servo_x, ang_servo_y, p, r, gx, gy, gz);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MONITOR_DELAY));
    }
}

// ============================================================
// --- MAIN ---
// ============================================================

void app_main(void) {
    printf("--- Fase 2: Controle de Mesa + Leitura MPU6050 ---\n");
    
    // Cria Mutex
    xDataMutex = xSemaphoreCreateMutex();

    // Inicializa Hardware
    init_ledc();
    init_adc();
    ESP_ERROR_CHECK(init_i2c());
    init_mpu6050();
    printf("Hardware inicializado com sucesso.\n");

    // Cria Tarefas
    // Prioridade Alta (2 e 3) para Controle
    // Prioridade Média (2) para Sensores
    // Prioridade Baixa (1) para Serial
    xTaskCreate(task_joystick_read, "JoyRead", 2048, NULL, 2, NULL);
    xTaskCreate(task_servo_control, "ServoCtrl", 2048, NULL, 3, NULL);
    xTaskCreate(task_mpu_read, "MpuRead", 2048, NULL, 2, NULL); 
    xTaskCreate(task_monitor, "Monitor", 4096, NULL, 1, NULL);
}