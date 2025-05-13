#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/np_led.h"
#include "lib/neopixel_matrices.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>

/***************************************
 *                                      *
 *          Variáveis globais           *
 *                                      *
 ****************************************/

const uint BUTTON_A = 5;        // Botão A
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

const float DIVIDER_PWM = 64.0; // Divisor de clock para PWM
const uint16_t PERIOD = 4096;   // Período do PWM
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

/***************************************
 *                                      *
 *          Funções do sistema          *
 *                                      *
 ****************************************/
void setup_pwm(uint gpio, uint *slice, uint16_t level)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(gpio, level);
    pwm_set_enabled(*slice, true);
}

// Task para controlar o botão A
void vButtonATask()
{
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    while (true)
    {
        current_time = to_us_since_boot(get_absolute_time());

        if (!gpio_get(BUTTON_A) && (current_time - last_time_A > 200000))
        {
            last_time_A = current_time;
            OPERATION_MODE = (OPERATION_MODE == NORMAL_MODE) ? NIGHT_MODE : NORMAL_MODE;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno delay para evitar uso excessivo da CPU
    }
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
                // 1 beep curto por um segundo
                pwm_set_gpio_level(BUZZER_A, 128);
                vTaskDelay(pdMS_TO_TICKS(1000));
                pwm_set_gpio_level(BUZZER_A, 0);
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;

            case YELLOW_SIGNAL:
                // beeps curtos intermitentes
                pwm_set_gpio_level(BUZZER_A, 128);
                vTaskDelay(pdMS_TO_TICKS(100));
                pwm_set_gpio_level(BUZZER_A, 0);
                vTaskDelay(pdMS_TO_TICKS(400));
                break;

            case RED_SIGNAL:
                // tom contínuo curto (500ms ligado, 1500ms desligado)
                pwm_set_gpio_level(BUZZER_A, 128);
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
                pwm_set_gpio_level(BUZZER_A, 128);
                break;
            case OFF:
                pwm_set_gpio_level(BUZZER_A, 0);
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task para controlar o LED neopixel
void vNeoPixel()
{
    npInit(MATRIX_LED_PIN); // Inicializa o LED neopixel
    npClear();              // Limpa o buffer do LED neopixel

    // Aplica o brilho na matriz de pedestres e modo noturno
    applyBrightnessToMatrix(PEDESTRIANS_WALK, 0.05);
    applyBrightnessToMatrix(PEDESTRIANS_STOP, 0.05);
    applyBrightnessToMatrix(NIGHT_MODE_MATRIX, 0.05);

    // A cada atualização da matriz, checamos o modo de operação
    // Dessa forma, a matriz de LEDs não sofre delays na atualização
    while (true)
    {
        if (OPERATION_MODE == NORMAL_MODE) // Modo normal
        {
            switch (NORMAL_MODE_SIGNAL_FLAG)
            {
            case GREEN_SIGNAL:
                updateMatrix(PEDESTRIANS_STOP);
                break;

            case YELLOW_SIGNAL:
                for (size_t i = 0; i < 3; i++)
                {
                    if (OPERATION_MODE != NORMAL_MODE)
                        break;

                    npClear();
                    npWrite();
                    vTaskDelay(pdMS_TO_TICKS(500));

                    if (OPERATION_MODE != NORMAL_MODE)
                        break;

                    updateMatrix(PEDESTRIANS_STOP);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                break;

            case RED_SIGNAL:
                updateMatrix(PEDESTRIANS_WALK);
                for (size_t i = 0; i < 30; i++) // 3000ms em pedaços de 100ms
                {
                    if (OPERATION_MODE != NORMAL_MODE)
                        break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                for (size_t i = 0; i < 3; i++)
                {
                    if (OPERATION_MODE != NORMAL_MODE)
                        break;

                    npClear();
                    npWrite();
                    vTaskDelay(pdMS_TO_TICKS(500));

                    if (OPERATION_MODE != NORMAL_MODE)
                        break;

                    updateMatrix(PEDESTRIANS_WALK);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                break;
            }
        }
        else if (OPERATION_MODE == NIGHT_MODE) // Modo noturno
        {
            npClear();
            npWrite();
            vTaskDelay(pdMS_TO_TICKS(500));

            if (OPERATION_MODE != NIGHT_MODE)
                continue;

            updateMatrix(NIGHT_MODE_MATRIX);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task para modo normal
void vNormalModeControllerTask()
{
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

// Task para modo noturno
void vNightModeControllerTask()
{
    while (true)
    {
        NIGHT_MODE_SIGNAL_FLAG = ON;
        vTaskDelay(pdMS_TO_TICKS(1000));

        NIGHT_MODE_SIGNAL_FLAG = OFF;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task para o display OLED
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

    bool cor = true;
    while (true)
    {
        ssd1306_fill(&ssd, !cor);                     // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);      // Desenha uma linha
        // ssd1306_line(&ssd, 3, 37, 123, 37, cor);             // Desenha uma linha
        ssd1306_draw_string(&ssd, "SEMAFORO", 20, 6);     // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16); // Desenha uma string

        if (OPERATION_MODE == NORMAL_MODE)
        {
            ssd1306_draw_string(&ssd, "Modo Normal", 10, 28); // Desenha uma string

            switch (NORMAL_MODE_SIGNAL_FLAG)
            {
            case GREEN_SIGNAL:
                ssd1306_draw_string(&ssd, "Sinal Verde", 10, 39);   // Desenha uma string
                ssd1306_draw_string(&ssd, "Pedestre para", 10, 49); // Desenha uma string
                break;
            case YELLOW_SIGNAL:
                ssd1306_draw_string(&ssd, "Sinal Amarelo", 10, 39); // Desenha uma string
                ssd1306_draw_string(&ssd, "Pedestre para", 10, 49); // Desenha uma string
                break;
            case RED_SIGNAL:
                ssd1306_draw_string(&ssd, "Sinal Vermelho", 10, 39); // Desenha uma string
                ssd1306_draw_string(&ssd, "Pedestre anda", 10, 49);  // Desenha uma string
                break;
            }
        }
        else if (OPERATION_MODE == NIGHT_MODE)
        {
            ssd1306_draw_string(&ssd, "Modo Noturno", 10, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "Passar com", 14, 39);   // Desenha uma string
            ssd1306_draw_string(&ssd, "atencao", 14, 49);      // Desenha uma string
        }

        ssd1306_send_data(&ssd);        // Atualiza o display
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para não travar a CPU
    }
}

/***************************************
 *                                      *
 *          Função principal            *
 *                                      *
 ****************************************/

int main()
{
    stdio_init_all();

    xTaskCreate(vNormalModeControllerTask, "Normal mode task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vNightModeControllerTask, "Night mode task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplayTask, "Cont Task Disp3", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLEDTask, "LED Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vNeoPixel, "NeoPixel Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vButtonATask, "Button A Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
