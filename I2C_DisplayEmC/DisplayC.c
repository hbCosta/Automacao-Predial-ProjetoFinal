/*
 * Por: Wilton Lacerda Silva
 *    Comunicação serial com I2C
 *  
 * Uso da interface I2C para comunicação com o Display OLED
 * 
 * Estudo da biblioteca ssd1306 com PicoW na Placa BitDogLab.
 *  
 * Este programa escreve uma mensagem no display OLED.
 * 
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "framesAnimacao.c"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pio_matrix.pio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define led_verde 11 // led verde
#define led_azul 12 // led azul
#define led_vermelho 13 // led vermelho
#define botaoA 5 
#define botaoB 6
#define VRX_PIN 26
#define OUT_PIN 7
#define NUM_PIXELS 25
#define BUZZER 21
#define SW_PIN 22

// Variáveis globais
static volatile uint a = 1;
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)


void inicializar(){
  gpio_init(led_azul);
  gpio_set_dir(led_azul, GPIO_OUT);
  gpio_put(led_azul, 1);
  gpio_init(led_verde);
  gpio_set_dir(led_verde, GPIO_OUT);
  gpio_init(led_vermelho);
  gpio_set_dir(led_vermelho, GPIO_OUT);

  gpio_init(SW_PIN);
  gpio_set_dir(SW_PIN, GPIO_IN);
  gpio_pull_up(SW_PIN);

  gpio_init(botaoA);
  gpio_set_dir(botaoA, GPIO_IN); // Configura o pino como entrada
  gpio_pull_up(botaoA);          // Habilita o pull-up interno

  gpio_init(botaoB);
  gpio_set_dir(botaoB, GPIO_IN); // Configura o pino como entrada
  gpio_pull_up(botaoB);          // Habilita o pull-up interno

  
  gpio_init(BUZZER);
  gpio_set_dir(BUZZER, GPIO_OUT);
}

// rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double b, double r, double g)
{
    unsigned char R, G, B;
    R = r * 55;
    G = g * 55;
    B = b * 20;
    return (G << 24) | (R << 16) | (B << 8);
}


// rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{
    for(int16_t i = 0; i < NUM_PIXELS; i++)
    {
        if(r==1.0 && g==1.0 && b==1.0)
        {
            valor_led = matrix_rgb(desenho[24 - i], desenho[24 - i], desenho[24 - i]);
            pio_sm_put_blocking(pio, sm, valor_led);
        } else {
            if(b == 1.0)
            {
                valor_led = matrix_rgb(desenho[24 - i], r = 0.0, g = 0.0);
                pio_sm_put_blocking(pio, sm, valor_led);
            }
            if(r == 1.0)
            {
                valor_led = matrix_rgb(b = 0.0, desenho[24 - i], g = 0.0);
                pio_sm_put_blocking(pio, sm, valor_led);
            }
            if(g == 1.0)
            {
                valor_led = matrix_rgb(b = 0.0, r = 0.0, desenho[24 - i]);
                pio_sm_put_blocking(pio, sm, valor_led);
            }
        }
    }
}

void animacaoHumbertoZigZag(PIO pio, uint sm, uint32_t valor_led, double r, double g, double b) {
  // Exibe cada frame por 100 ms (FPS = 10)
  for (int i = 0; i < 6; i++) {
      desenho_pio(framesHumberto[i], valor_led, pio, sm, r, g, b);
      sleep_ms(400); // Delay para controlar o FPS
  }

}

// Prototipação da função de interrupção
static void gpio_irq_handler(uint gpio, uint32_t events);

static int temperatura = 30;
static uint32_t last_temp_update = 0;
static bool temperatura_iniciada = false;

// Função para gerar som com uma frequência específica
void tocar_som(uint buzzer_pin, uint frequencia, uint duracao_ms) {
  uint periodo_us = 1000000  / frequencia; // Período em microssegundos

  uint metade_periodo = periodo_us / 2;   // Meio período para alternar o estado
  uint ciclos = (frequencia * duracao_ms) / 1000; // Quantos ciclos gerar no tempo

  for (uint i = 0; i < ciclos; i++) {
      gpio_put(buzzer_pin, 1); // Liga o buzzer
      sleep_us(metade_periodo);

      gpio_put(buzzer_pin, 0); // Desliga o buzzer
      sleep_us(metade_periodo);
  }
}

static bool subir_temperatura = false;
void atualizar_temperatura()
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Verifica se passaram 5 segundos desde a última atualização
    if (temperatura_iniciada && current_time - last_temp_update >= 5000)
    {
        if (temperatura > 23)
        {
            temperatura--; // Reduz a temperatura até 23
        }
        else
        {
            temperatura_iniciada = false;
        }
        last_temp_update = current_time; // Atualiza o tempo da última mudança
    }else if(subir_temperatura && current_time - last_temp_update >=3000){
      if(temperatura < 39){
        temperatura ++;
      }else{
        subir_temperatura = false;
      }
      last_temp_update = current_time;
    }
}

// Função para ajustar a temperatura com base no joystick
void ajustar_temperatura_pelo_joystick()
{
    // Só permite alterar a temperatura se o LED verde estiver ligado
    if (gpio_get(led_verde) == 1 && !temperatura_iniciada)
    {
        adc_select_input(0);
        uint16_t vrx_value = adc_read(); // Lê o valor do eixo X, de 0 a 4095.
        printf("VRX: %u\n", vrx_value);

        // Define limiares para evitar pequenas variações
        if (vrx_value > 3500) // Movimento para cima
        {
            if (temperatura < 30) // Limite superior
            {
                temperatura++;
            }
        }
        else if (vrx_value < 800) // Movimento para baixo
        {
            if (temperatura > 15) // Limite inferior
            {
                temperatura--;
            }
        }
    }
}
// Variável global para controlar a animação
bool executar_animacao = false;

int main()
{
  PIO pio = pio0;
  bool ok;
  uint16_t i;
  uint32_t valor_led;
  double r = 0.0, b = 0.0, g = 0.0;

  // coloca a frequência de clock para 128 MHz, facilitando a divisão pelo clock
  ok = set_sys_clock_khz(128000, false);

  stdio_init_all();

  
  printf("iniciando a transmissão PIO");
  if (ok)
      printf("clock set to %ld\n", clock_get_hz(clk_sys));


  // configurações da PIO
  uint offset = pio_add_program(pio, &pio_matrix_program);
  uint sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, OUT_PIN);


  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);
  inicializar();
  adc_init();
  adc_gpio_init(VRX_PIN);
  bool sw_value = gpio_get(SW_PIN) == 0;


  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display
  
  // Configuração da interrupção com callback
  gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);


  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  bool cor = true;
  char temp_str[3];
  while (true)
  {

    
    if(temperatura_iniciada || subir_temperatura){
      atualizar_temperatura();
    }

    ajustar_temperatura_pelo_joystick();
    if(executar_animacao && temperatura >=39){      
      tocar_som(BUZZER, 261, 500); 
      animacaoHumbertoZigZag(pio, sm, valor_led, r, g, 1);
    }
    // Atualiza o conteúdo do display com animações
    ssd1306_fill(&ssd, !cor); // Limpa o display
    //ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo
    ssd1306_draw_string(&ssd, "TEMPERATURA", 20, 10); // Desenha uma string        
    sprintf(temp_str, "%d", temperatura);
    ssd1306_draw_string(&ssd, temp_str, 50, 30); // Exibe a temperatura

    ssd1306_send_data(&ssd); // Atualiza o display

    sleep_ms(100);
  }
}

// Função de interrupção com debouncing
void gpio_irq_handler(uint gpio, uint32_t events)
{
    // Obtém o tempo atual em microssegundos
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    printf("A = %d\n", a);
    // Verifica se passou tempo suficiente desde o último evento
    if (current_time - last_time > 200000) // 200 ms de debouncing
    {
      if(gpio == botaoA){
        last_time = current_time; // Atualiza o tempo do último evento
        printf("Mudanca de Estado do Led. A = %d\n", a);
        gpio_put(led_verde, !gpio_get(led_verde)); // Alterna o estado
        gpio_put(led_azul, !gpio_get(led_azul));

        if(!temperatura_iniciada){
          temperatura_iniciada = true;
          last_temp_update = to_ms_since_boot(get_absolute_time());
        }
      }else if(gpio == botaoB){
        last_time = current_time;
        gpio_put(led_verde, 0);
        gpio_put(led_azul, 0);
        gpio_put(led_vermelho, 1);
        executar_animacao = true;
        if(!subir_temperatura){
          subir_temperatura = true;
          last_temp_update = to_ms_since_boot(get_absolute_time());
        }
      }else if(gpio == SW_PIN){
        temperatura = 30;
        temperatura_iniciada = false;
        subir_temperatura = false;
        executar_animacao = false;
        last_temp_update = 0;
        a = 1;

        gpio_put(led_verde, 0);
        gpio_put(led_azul, 1);
        gpio_put(led_vermelho, 0);

        printf("Sistema resetado!\n");

        sleep_ms(500); // Pequeno delay para evitar múltiplos resets
      }

      a++; 

    }
}