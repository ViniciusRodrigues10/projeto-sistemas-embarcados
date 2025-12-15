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
#define PIN_JOY_HORIZ 34 
#define PIN_JOY_VERT  35  
#define ADC_CHAN_HORIZ ADC_CHANNEL_6 
#define ADC_CHAN_VERT  ADC_CHANNEL_7 

// Servos
#define PIN_SERVO_X GPIO_NUM_13
#define PIN_SERVO_Y GPIO_NUM_12

// MPU6050 (I2C)
#define I2C_MASTER_SCL_IO 26       
#define I2C_MASTER_SDA_IO 25       
#define I2C_MASTER_NUM 0          
#define I2C_MASTER_FREQ_HZ 100000 
#define MPU6050_ADDR 0x68         

// ============================================================
// --- CONSTANTES ---
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

// --- AJUSTES DO USUÁRIO ---
#define MIN_ANGLE_SERVO 70      // Ângulo mínimo de segurança
#define MAX_ANGLE_SERVO 110     // Ângulo máximo de segurança
#define SERVO_SMOOTH_FACTOR 0.40 // Suavidade (0.01 lento a 1.0 instantâneo)

// ADC Joystick
#define ADC_UNIT ADC_UNIT_1
#define ADC_MAX_VAL 4095
#define JOY_DEADZONE 50 
#define FILTER_ALPHA 0.10

// MPU6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define RAD_TO_DEG 57.2957795131

// Tempos das Tasks (ms)
#define TASK_JOY_DELAY 20 
#define TASK_SERVO_DELAY 20 
#define TASK_MPU_DELAY 50
#define TASK_MONITOR_DELAY 200

// ============================================================
// --- ESTRUTURAS E VARIÁVEIS GLOBAIS ---
// ============================================================

SemaphoreHandle_t xDataMutex;

// Estrutura para guardar a calibração
typedef struct {
    int min_x, center_x, max_x;
    int min_y, center_y, max_y;
} JoystickCalib;

// Valores padrão (serão sobrescritos na inicialização)
JoystickCalib joy_calib = {
    .min_x = 0, .center_x = 2048, .max_x = 4095,
    .min_y = 0, .center_y = 2048, .max_y = 4095
};

// Dados Joystick/Servo
float smoothed_x = 2048;
float smoothed_y = 2048;
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

// Mapeamento Simples (Linear)
long map_value(float x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Mapeamento Inteligente (Assimétrico com base na calibração)
long map_joystick(float x, int in_min, int in_center, int in_max, int out_min, int out_center, int out_max) {
    // Zona Morta no Centro
    if (abs(x - in_center) < JOY_DEADZONE) return out_center;

    if (x < in_center) {
        // Lado Esquerdo/Baixo (Do Mínimo até o Centro)
        if (x < in_min) x = in_min;
        return (long)((x - in_min) * (out_center - out_min) / (in_center - in_min) + out_min);
    } else {
        // Lado Direito/Cima (Do Centro até o Máximo)
        if (x > in_max) x = in_max;
        return (long)((x - in_center) * (out_max - out_center) / (in_max - in_center) + out_center);
    }
}

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

static void init_mpu6050() {
    uint8_t data[2];
    data[0] = MPU6050_PWR_MGMT_1;
    data[1] = 0x00; 
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
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

// ============================================================
// --- FUNÇÃO DE CALIBRAÇÃO INICIAL ---
// ============================================================
void calibrar_inicial() {
    printf("\n\n==========================================\n");
    printf("   INICIANDO CALIBRACAO DO JOYSTICK\n");
    printf("==========================================\n");
    
    // --- PASSO 1: CENTRO ---
    printf(">> PASSO 1: SOLTE O JOYSTICK (CENTRO)\n");
    printf(">> Aguarde 3 segundos...\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); printf("3...\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); printf("2...\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); printf("1...\n");
    
    int soma_x = 0, soma_y = 0;
    for(int i=0; i<50; i++) {
        int rx, ry;
        adc_oneshot_read(adc_handle, ADC_CHAN_HORIZ, &rx);
        adc_oneshot_read(adc_handle, ADC_CHAN_VERT, &ry);
        soma_x += rx;
        soma_y += ry;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    joy_calib.center_x = soma_x / 50;
    joy_calib.center_y = soma_y / 50;
    printf("OK! Centro detectado: X=%d, Y=%d\n\n", joy_calib.center_x, joy_calib.center_y);

    // --- PASSO 2: EXTREMOS ---
    printf(">> PASSO 2: GIRE O JOYSTICK EM CIRCULOS (5 segundos)\n");
    printf(">> Bata nos cantos maximos e minimos...\n");
    
    int min_x = 4095, max_x = 0;
    int min_y = 4095, max_y = 0;
    
    // Loop de 5 segundos
    for(int i=0; i<250; i++) { // 250 * 20ms = 5000ms
        int rx, ry;
        adc_oneshot_read(adc_handle, ADC_CHAN_HORIZ, &rx);
        adc_oneshot_read(adc_handle, ADC_CHAN_VERT, &ry);
        
        if (rx < min_x) min_x = rx;
        if (rx > max_x) max_x = rx;
        if (ry < min_y) min_y = ry;
        if (ry > max_y) max_y = ry;
        
        if(i % 50 == 0) printf(". "); // Feedback visual
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // Atualiza a calibração com uma pequena margem de segurança
    joy_calib.min_x = min_x; joy_calib.max_x = max_x;
    joy_calib.min_y = min_y; joy_calib.max_y = max_y;
    
    printf("\n\nCALIBRACAO CONCLUIDA!\n");
    printf("X: [%d] <- %d -> [%d]\n", joy_calib.min_x, joy_calib.center_x, joy_calib.max_x);
    printf("Y: [%d] <- %d -> [%d]\n", joy_calib.min_y, joy_calib.center_y, joy_calib.max_y);
    printf("Iniciando controle em 2 segundos...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// ============================================================
// --- TASKS ---
// ============================================================

void task_joystick_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        int raw_x, raw_y;
        
        adc_oneshot_read(adc_handle, ADC_CHAN_HORIZ, &raw_x);
        adc_oneshot_read(adc_handle, ADC_CHAN_VERT, &raw_y);

        // Filtro Exponencial para tirar ruído elétrico
        smoothed_x = (FILTER_ALPHA * raw_x) + ((1.0 - FILTER_ALPHA) * smoothed_x);
        smoothed_y = (FILTER_ALPHA * raw_y) + ((1.0 - FILTER_ALPHA) * smoothed_y);
        
        // --- MAPEAMENTO COM CALIBRAÇÃO ---
        // Aqui usamos os valores lidos na inicialização para mapear perfeitamente para 70-110 graus
        long angle_x = map_joystick(smoothed_x, joy_calib.min_x, joy_calib.center_x, joy_calib.max_x, MIN_ANGLE_SERVO, 90, MAX_ANGLE_SERVO);
        long angle_y = map_joystick(smoothed_y, joy_calib.min_y, joy_calib.center_y, joy_calib.max_y, MIN_ANGLE_SERVO, 90, MAX_ANGLE_SERVO);
        
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

void task_servo_control(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    static float current_duty_x = 0; 
    static float current_duty_y = 0;

    if (current_duty_x == 0) current_duty_x = (1.5 * (1 << LEDC_RESOLUTION)) / 20.0; 
    if (current_duty_y == 0) current_duty_y = (1.5 * (1 << LEDC_RESOLUTION)) / 20.0;

    for (;;) {
        uint32_t target_dx = 0, target_dy = 0;
        
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            target_dx = servo_duty_x;
            target_dy = servo_duty_y;
            xSemaphoreGive(xDataMutex);
        }
        
        // Movimento Suave ("Manteiga")
        if (target_dx > 0) {
            float diferenca_x = (float)target_dx - current_duty_x;
            current_duty_x += diferenca_x * SERVO_SMOOTH_FACTOR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, (uint32_t)current_duty_x);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
        }

        if (target_dy > 0) {
            float diferenca_y = (float)target_dy - current_duty_y;
            current_duty_y += diferenca_y * SERVO_SMOOTH_FACTOR;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, (uint32_t)current_duty_y);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); 
    }
}

void task_mpu_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t buffer[14]; 
    
    static float angle_pitch = 0.0; 
    static float angle_roll = 0.0;  
    
    const float dt = 0.05; 
    const float alpha = 0.98; 

    for (;;) {
        uint8_t reg = MPU6050_ACCEL_XOUT_H;
        i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buffer, 14, 100 / portTICK_PERIOD_MS);

        int16_t ax = (buffer[0] << 8) | buffer[1];
        int16_t ay = (buffer[2] << 8) | buffer[3];
        int16_t az = (buffer[4] << 8) | buffer[5];
        int16_t gx = (buffer[8] << 8) | buffer[9];
        int16_t gy = (buffer[10] << 8) | buffer[11];
        int16_t gz = (buffer[12] << 8) | buffer[13];

        float acc_pitch = atan2((-ax), sqrt(ay * ay + az * az)) * RAD_TO_DEG;
        float acc_roll  = atan2(ay, az) * RAD_TO_DEG;

        float gyro_x_dps = gx / 131.0; 
        float gyro_y_dps = gy / 131.0;

        angle_pitch = alpha * (angle_pitch + gyro_y_dps * dt) + (1.0 - alpha) * acc_pitch;
        angle_roll  = alpha * (angle_roll  + gyro_x_dps * dt) + (1.0 - alpha) * acc_roll;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            mpu_pitch = angle_pitch; 
            mpu_roll = angle_roll;   
            raw_accel_x = ax; raw_accel_y = ay; raw_accel_z = az;
            raw_gyro_x = gx; raw_gyro_y = gy; raw_gyro_z = gz;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MPU_DELAY));
    }
}

void task_monitor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dx = 0, dy = 0;
        float sx = 0, sy = 0;
        float p = 0, r = 0;
        int16_t gx = 0, gy = 0, gz = 0;

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

        long ang_servo_x = map_value((dx * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);
        long ang_servo_y = map_value((dy * SERVO_PERIOD_US) / (1 << LEDC_RESOLUTION), MIN_PULSE_US, MAX_PULSE_US, 0, 180);

        printf("{\"joy_x\":%.0f, \"joy_y\":%.0f, \"servo_x\":%ld, \"servo_y\":%ld, \"pitch\":%.2f, \"roll\":%.2f, \"gyro_x\":%d, \"gyro_y\":%d, \"gyro_z\":%d}\n",
               sx, sy, ang_servo_x, ang_servo_y, p, r, gx, gy, gz);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MONITOR_DELAY));
    }
}

// ============================================================
// --- MAIN ---
// ============================================================

void app_main(void) {
    printf("--- Iniciando Sistema Mesa Labirinto ---\n");
    
    // 1. Inicializa Hardware
    xDataMutex = xSemaphoreCreateMutex();
    init_ledc();
    init_adc();
    ESP_ERROR_CHECK(init_i2c());
    init_mpu6050();
    
    // 2. Roda a Calibração (BLOQUEANTE)
    // As tasks só serão criadas DEPOIS que você calibrar
    calibrar_inicial();

    // 3. Cria Tarefas (Agora o sistema roda normalmente)
    printf("Hardware inicializado e calibrado. Iniciando Tasks...\n");
    xTaskCreate(task_joystick_read, "JoyRead", 2048, NULL, 2, NULL);
    xTaskCreate(task_servo_control, "ServoCtrl", 2048, NULL, 3, NULL);
    xTaskCreate(task_mpu_read, "MpuRead", 2048, NULL, 2, NULL); 
    xTaskCreate(task_monitor, "Monitor", 4096, NULL, 1, NULL);
}