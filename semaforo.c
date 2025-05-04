#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/np_led.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>

const uint BUTTON_A = 5;        // Botão A
const uint BUTTON_B = 6;        // Botão B
volatile uint32_t current_time; // Tempo atual (usado para debounce)
static volatile uint32_t last_time_A = 0;

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define MATRIX_LED_PIN 7 // Pino do LED neopixel

#define LED_G 11
#define LED_B 12
#define LED_R 13

const uint16_t BUZZER_A = 21;

const float DIVIDER_PWM = 16.0; // Divisor de clock para PWM
const uint16_t PERIOD = 4096;   // Período do PWM
uint16_t last_buzzer_time = 0;
uint slice_buzzer;

typedef enum
{
    NORMAL_MODE,
    NIGHT_MODE
} OperationMode;

typedef enum
{
    GREEN_SIGNAL,
    YELLOW_SIGNAL,
    RED_SIGNAL,
} NormalModeSignalState;

typedef enum
{
    OFF,
    ON
} NightModeSignalState;

volatile uint16_t OPERATION_MODE = NORMAL_MODE; // Modo de operação inicial
volatile uint16_t NORMAL_MODE_SIGNAL_FLAG = GREEN_SIGNAL;
volatile uint16_t NIGHT_MODE_SIGNAL_FLAG = ON;

TaskHandle_t xHandleNormalMode = NULL; // Handle para a tarefa do modo normal
TaskHandle_t xHandleNightMode = NULL;  // Handle para a tarefa do modo noturno

void gpio_irq_handler(uint gpio, uint32_t event)
{
    current_time = to_us_since_boot(get_absolute_time());

    if (gpio == BUTTON_A && (current_time - last_time_A > 200000))
    {
        last_time_A = current_time;
        OPERATION_MODE = (OPERATION_MODE == NORMAL_MODE) ? NIGHT_MODE : NORMAL_MODE;
    }

    // Trecho para modo BOOTSEL com botão B
    if (gpio == BUTTON_B)
    {
        reset_usb_boot(0, 0);
    }
}

void setup_pwm(uint gpio, uint *slice, uint16_t level)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(gpio, level);
    pwm_set_enabled(*slice, true);
}

// Task para controlar o LED RGB
void vLEDTask()
{
    gpio_init(LED_G);
    gpio_init(LED_B);
    gpio_init(LED_R);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);
    gpio_set_dir(LED_R, GPIO_OUT);

    while (true)
    {
        if (OPERATION_MODE == NORMAL_MODE)
        {
            switch (NORMAL_MODE_SIGNAL_FLAG)
            {
            case GREEN_SIGNAL:
                gpio_put(LED_G, true);
                gpio_put(LED_R, false);
                break;
            case YELLOW_SIGNAL:
                gpio_put(LED_G, true);
                gpio_put(LED_R, true);
                break;
            case RED_SIGNAL:
                gpio_put(LED_G, false);
                gpio_put(LED_R, true);
                break;
            }
        }
        else if (OPERATION_MODE == NIGHT_MODE)
        {
            switch (NIGHT_MODE_SIGNAL_FLAG)
            {
            case ON:
                gpio_put(LED_G, true);
                gpio_put(LED_R, true);
                break;
            case OFF:
                gpio_put(LED_G, false);
                gpio_put(LED_R, false);
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno delay para evitar uso excessivo da CPU
    }
}

// Task para controlar o buzzer
void vBuzzerTask()
{
    setup_pwm(BUZZER_A, &slice_buzzer, 0);

    while (true)
    {
        if (OPERATION_MODE == NORMAL_MODE)
        {
            switch (NORMAL_MODE_SIGNAL_FLAG)
            {
            case GREEN_SIGNAL:
                // 1 beep curto por segundo
                pwm_set_gpio_level(BUZZER_A, PERIOD / 2);
                vTaskDelay(pdMS_TO_TICKS(100));
                pwm_set_gpio_level(BUZZER_A, 0);
                vTaskDelay(pdMS_TO_TICKS(900));
                break;

            case YELLOW_SIGNAL:
                // beeps curtos intermitentes
                pwm_set_gpio_level(BUZZER_A, PERIOD / 2);
                vTaskDelay(pdMS_TO_TICKS(100));
                pwm_set_gpio_level(BUZZER_A, 0);
                vTaskDelay(pdMS_TO_TICKS(400));
                break;

            case RED_SIGNAL:
                // tom contínuo curto (500ms ligado, 1500ms desligado)
                pwm_set_gpio_level(BUZZER_A, PERIOD / 2);
                vTaskDelay(pdMS_TO_TICKS(500));
                pwm_set_gpio_level(BUZZER_A, 0);
                vTaskDelay(pdMS_TO_TICKS(1500));
                break;
            }
        }
        else if (OPERATION_MODE == NIGHT_MODE)
        {
            switch (NIGHT_MODE_SIGNAL_FLAG)
            {
            case ON:
                pwm_set_gpio_level(BUZZER_A, PERIOD / 2);
                break;
            case OFF:
                pwm_set_gpio_level(BUZZER_A, 0);
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task para modo normal
void vNormalModeControllerTask()
{
    vTaskSuspend(xHandleNightMode); // Suspende a tarefa do modo noturno
    while (true)
    {
        NORMAL_MODE_SIGNAL_FLAG = GREEN_SIGNAL;
        vTaskDelay(pdMS_TO_TICKS(6000));

        NORMAL_MODE_SIGNAL_FLAG = YELLOW_SIGNAL;
        vTaskDelay(pdMS_TO_TICKS(3000));

        NORMAL_MODE_SIGNAL_FLAG = RED_SIGNAL;
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

void vNightModeControllerTask()
{
    vTaskSuspend(xHandleNormalMode); // Suspende a tarefa do modo noturno
    while (true)
    {
        NIGHT_MODE_SIGNAL_FLAG = ON;
        vTaskDelay(pdMS_TO_TICKS(1000));

        NIGHT_MODE_SIGNAL_FLAG = OFF;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void vDisplayTask()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char str_y[5]; // Buffer para armazenar a string
    int contador = 0;
    bool cor = true;
    while (true)
    {
        sprintf(str_y, "%d", contador);                      // Converte em string
        contador++;                                          // Incrementa o contador
        ssd1306_fill(&ssd, !cor);                            // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);        // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);             // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);             // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);   // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);    // Desenha uma string
        ssd1306_draw_string(&ssd, "  FreeRTOS", 10, 28);     // Desenha uma string
        ssd1306_draw_string(&ssd, "Contador  LEDs", 10, 41); // Desenha uma string
        ssd1306_draw_string(&ssd, str_y, 40, 52);            // Desenha uma string
        ssd1306_send_data(&ssd);                             // Atualiza o display
        sleep_ms(735);
    }
}

void vAlterTaskTask()
{
    while (true)
    {
        if (OPERATION_MODE == NORMAL_MODE)
        {
            vTaskSuspend(xHandleNightMode); // Suspende a tarefa do modo noturno
            vTaskResume(xHandleNormalMode); // Retoma a tarefa do modo normal
        }
        else if (OPERATION_MODE == NIGHT_MODE)
        {
            vTaskSuspend(xHandleNormalMode); // Suspende a tarefa do modo normal
            vTaskResume(xHandleNightMode);   // Retoma a tarefa do modo noturno
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para não travar a CPU
    }
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);

    // Para ser utilizado a alteração do modo com botão A
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);

    stdio_init_all();

    xTaskCreate(vNormalModeControllerTask, "Normal mode task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, &xHandleNormalMode);
    xTaskCreate(vNightModeControllerTask, "Night mode task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, &xHandleNightMode);
    xTaskCreate(vDisplayTask, "Cont Task Disp3", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vAlterTaskTask, "Alter Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLEDTask, "LED Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
