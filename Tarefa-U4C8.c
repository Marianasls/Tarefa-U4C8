/**
 * Autor: Mariana Santos.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"      
#include "include/ssd1306.h"
#include "hardware/pwm.h"

// Definição dos pinos
// I2C1 em GPI14 (SDA) e GPI15 (SCL)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define address 0x3C
#define VRX_PIN 26      // Eixo X joysitck
#define VRY_PIN 27      // Eixo Y joystick
#define SW_PIN 22       // botão joystick
#define BUTTON_A 5      // Botão A
// PWM
#define PWM_WRAP 255    // Calculado para 256 níveis de brilho ( 0 a 255) 
#define CLKDIV 4.0f     // Divisor do clock, freq calculada de 122.07 kHz
#define ADC_MAX 4095    // Valor máximo do ADC
#define ADC_CENTER 2048 // Posição central do joystick
// Definição dos limites da tela do display e do tamanho do quadrado
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64
#define SQUARE_SIZE    8

// Variaveis globais
const uint8_t led_rgb_pins[3] = {13,12,11};     // PINOS LED RGB: R = 13, B = 12, G = 11
static volatile uint8_t state = 0, enable_pwm=0;
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)
ssd1306_t ssd;          // Inicializa a estrutura do display

// Prototipo das funções
static void gpio_irq_handler(uint gpio, uint32_t events);
void init_all_pins();
void move_square(ssd1306_t *ssd, int joystick_x, int joystick_y);
void update_leds();

// Definição de funções
void setup_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);  // Define o pino como PWM
    uint slice_num = pwm_gpio_to_slice_num(pin);
    
    pwm_set_clkdiv(slice_num, CLKDIV); 
    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_enabled(slice_num, true);
}

// Função para mapear valores de um intervalo para outro
uint16_t map_value(uint16_t value) {
    if (value > ADC_CENTER) {
        return (value - ADC_CENTER) * 255 / (ADC_MAX - ADC_CENTER);
    } else {
        return (ADC_CENTER - value) * 255 / ADC_CENTER;
    }
}

// Mapeia o valor do joystick para a faixa da tela
uint8_t map_value_display(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (uint8_t)((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}


int main() {
    stdio_init_all();   // inicializa comunicação serial
    init_all_pins();    // inicializa todos os pinos GPIO que serão utilizados

    // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT); 
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Definição da interrupção para acionamento dos botões
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Configura PWM nos pinos dos LEDs
    setup_pwm(led_rgb_pins[0]);
    setup_pwm(led_rgb_pins[1]);

    while (1) {
        // Verifica o estado da variavel para saber se os LEDs PWM vão ser ativados ou não
        if(enable_pwm){
            update_leds();
        }
        sleep_ms(100);  // Pequeno delay para evitar leituras excessivas
    }
    return 0;
}

void init_all_pins() {
    // Inicializa os LEDs RGB
    for (int i=0; i<3; i++){
        gpio_init(led_rgb_pins[i]);              // Inicializa o pino do LED RED
        gpio_set_dir(led_rgb_pins[i], GPIO_OUT); // Configura o pino como saída
    }
    // Inicializa os botões
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);    // Configura o pino como entrada
    gpio_pull_up(BUTTON_A);  
    gpio_init(SW_PIN);
    gpio_set_dir(SW_PIN, GPIO_IN);
    gpio_pull_up(SW_PIN); 
    // Inicializa ADC
    adc_init();
    adc_gpio_init(VRX_PIN); 
    adc_gpio_init(VRY_PIN); 
    // Inicializa I2C com frequência de 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void gpio_irq_handler(uint gpio, uint32_t events) {
    // Obtém o tempo atual em microssegundos
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    // Verifica se passou tempo suficiente desde o último evento
    if (current_time - last_time > 200000) { // 200 ms de debouncing
        last_time = current_time; // Atualiza o tempo do último evento
        if(gpio == BUTTON_A) {
            enable_pwm = !enable_pwm;
            gpio_put(led_rgb_pins[0], false);
            gpio_put(led_rgb_pins[1], false);
        } else if(gpio == SW_PIN) {
            state = gpio_get(led_rgb_pins[2]);
            gpio_put(led_rgb_pins[2], !state);
            // altera borda do display
            if(state){
                ssd1306_fill(&ssd, false);
                ssd1306_hline(&ssd, 1, 120, 1, true);
                ssd1306_hline(&ssd, 1, 120, 125, true);
            } else {
                ssd1306_fill(&ssd, false);
                ssd1306_rect(&ssd, 1, 1, 125, 60, true, false);
            }
            ssd1306_send_data(&ssd);
        }
    }

}

// Movimenta o quadrado com base nos valores do joystick
void move_square(ssd1306_t *ssd, int joystick_x, int joystick_y) {
    static uint8_t prev_x = 60; // Posição inicial
    static uint8_t prev_y = 28;

    // Converte a entrada do joystick para posição no display
    uint8_t new_x = map_value_display(joystick_x, 0, ADC_MAX, 2, DISPLAY_WIDTH-4 - SQUARE_SIZE);
    uint8_t new_y = map_value_display(joystick_y, 0, ADC_MAX, 2, DISPLAY_HEIGHT-4 - SQUARE_SIZE);

    // Apaga o quadrado na posição anterior
    ssd1306_rect(ssd, prev_y, prev_x, SQUARE_SIZE, SQUARE_SIZE, false, true);

    // Desenha o quadrado na nova posição
    ssd1306_rect(ssd, new_y, new_x, SQUARE_SIZE, SQUARE_SIZE, true, true);

    // Atualiza o display
    ssd1306_send_data(ssd);

    // Atualiza a posição anterior
    prev_x = new_x;
    prev_y = new_y;
    printf("prev_x: %u - prev_y: %u  -  new_x: %u - new_y: %u\n", prev_x, prev_y, new_x, new_y);
}

// Função para ler o ADC e atualizar o PWM dos LEDs
void update_leds() {
    adc_select_input(0);  // Seleciona o canal do eixo X 
    uint16_t adc_x = adc_read();
    
    adc_select_input(1);  // Seleciona o canal do eixo Y 
    uint16_t adc_y = adc_read();

    printf("Valor X: %u, Valor Y: %u \n", adc_x, adc_y);
    // Se a movimentação for muito pequena não realiza nenhuma ação, para evitar ruidos.
    //if (adc_x > ADC_CENTER-70 || adc_x < ADC_CENTER+70) return; 
    //if (adc_y > ADC_CENTER-70 || adc_y < ADC_CENTER+70) return; 

    // Converte os valores ADC (0-4095) para PWM (0-255)
    uint16_t brightness_r = map_value(adc_x);
    uint16_t brightness_b = map_value(adc_y); 

    printf("intensidade LED RED: %u, intensidade LED BLUE: %u \n", brightness_r, brightness_b);

    // Atualiza os valores do PWM dos LEDs
    pwm_set_gpio_level(led_rgb_pins[0], brightness_r);
    pwm_set_gpio_level(led_rgb_pins[1], brightness_b);

    // Atualiza display com a posição dos eixos do joystick
    move_square(&ssd, adc_x, adc_y);
}
