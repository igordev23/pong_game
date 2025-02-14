#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "ws2818b.pio.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"

// Definição dos pinos para comunicação I2C
const uint I2C_SDA = 14; // Pino GPIO para o SDA (Serial Data Line)
const uint I2C_SCL = 15; // Pino GPIO para o SCL (Serial Clock Line)


// Definição dos pinos do buzzer para efeitos sonoros
#define BUZZER_A 21 // Pino GPIO conectado ao Buzzer A (colisão com a raquete)
#define BUZZER_B 8  // Pino GPIO conectado ao Buzzer B (ponto marcado)
#define BUZZER_FREQUENCY 1000 // Frequência do som emitido pelos buzzers em Hertz


// Definição de parâmetros para os LEDs WS2812B
#define LED_COUNT 25 // Quantidade total de LEDs no strip
#define LED_PIN 7    // Pino GPIO conectado ao Data IN dos LEDs

// Definição do botão adicional
#define BUTTON_A 5   // Pino GPIO conectado ao Botão A

// Definição dos pinos do joystick
#define VRX_PIN 26  // Eixo X do joystick conectado ao ADC0
#define VRY_PIN 27  // Eixo Y do joystick conectado ao ADC1
#define SW_PIN 16   // Botão do joystick (pressionado = LOW)

// Estrutura para representar um pixel no formato GRB (Green, Red, Blue)
typedef struct {
  uint8_t G; // Componente de cor Verde
  uint8_t R; // Componente de cor Vermelha
  uint8_t B; // Componente de cor Azul
} npLED_t;

// Buffer para armazenar o estado de todos os LEDs
// Cada posição representa um LED na tira WS2812B
npLED_t leds[LED_COUNT];

// Variáveis da máquina PIO para controle dos LEDs
PIO np_pio;  // Instância da PIO utilizada para comunicação com os LEDs
uint sm;     // State machine da PIO para enviar dados aos LEDs

// Variáveis para a posição da bola no jogo
int ballX = 2, ballY = 2; // Posição inicial da bola
int ballDirX = 1, ballDirY = 1; // Direção de movimento da bola

// Variável para a posição da raquete do jogador
int paddleY = 2; // Posição inicial da raquete

// Variáveis para o placar do jogo
int playerScore = 0; // Pontuação do jogador
int cpuScore = 0;    // Pontuação da CPU (adversário)

// Função para inicializar a comunicação com os LEDs WS2812B
void npInit(uint pin) {
  // Carrega o programa PIO para controle dos LEDs e retorna o offset na memória da PIO
  uint offset = pio_add_program(pio0, &ws2818b_program);

  // Define que a PIO0 será usada inicialmente
  np_pio = pio0;

  // Tenta reivindicar uma State Machine (SM) não utilizada na PIO0
  sm = pio_claim_unused_sm(np_pio, false);

  // Se não houver SM disponível na PIO0, tenta na PIO1
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true);
  }

  // Inicializa o programa PIO para controlar os LEDs WS2812B
  // np_pio: Instância da PIO em uso
  // sm: State Machine utilizada para comunicação
  // offset: Localização do programa na memória PIO
  // pin: Pino GPIO conectado aos LEDs
  // 800000.f: Frequência de 800kHz para comunicação com os WS2812B
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Inicializa o buffer de LEDs, apagando todos (cor preta)
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = leds[i].G = leds[i].B = 0; // Define as cores como 0 (apagado)
  }
}

// Função para definir a cor de um LED específico
// index: Índice do LED na tira (0 a LED_COUNT-1)
// r, g, b: Valores de cor (0 a 255) para Vermelho, Verde e Azul
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r; // Define a intensidade do Vermelho
  leds[index].G = g; // Define a intensidade do Verde
  leds[index].B = b; // Define a intensidade do Azul
}

// Função para apagar todos os LEDs
void npClear() {
  // Percorre todos os LEDs e define a cor como preta (apagado)
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0); // Define R, G e B como 0
}

// Função para enviar os dados dos LEDs para a tira WS2812B
void npWrite() {
  // Percorre todos os LEDs no buffer e envia os dados na ordem GRB
  for (uint i = 0; i < LED_COUNT; ++i) {
    // Envia a cor Verde primeiro (formato GRB)
    pio_sm_put_blocking(np_pio, sm, leds[i].G);

    // Em seguida, envia a cor Vermelha
    pio_sm_put_blocking(np_pio, sm, leds[i].R);

    // Por último, envia a cor Azul
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }

  // Aguarda um tempo mínimo para garantir que os dados foram recebidos
  sleep_us(100); // 100 microsegundos de espera
}

// Função para converter coordenadas 2D em índice linear na tira de LEDs
// x: Coordenada X (coluna) na matriz de LEDs
// y: Coordenada Y (linha) na matriz de LEDs
int getIndex(int x, int y) {
  // Se a linha for par, os LEDs são organizados da direita para a esquerda
  if (y % 2 == 0) {
    // Calcula o índice para linhas pares (ordem normal)
    // 24 - (posição linear) para ajustar à ordem inversa da matriz
    return 24 - (y * 5 + x);
  } 
  // Se a linha for ímpar, os LEDs são organizados da esquerda para a direita
  else {
    // Calcula o índice para linhas ímpares (ordem reversa)
    // 4 - x inverte a ordem dos LEDs na linha
    return 24 - (y * 5 + (4 - x));
  }
}

/*
Explicação:
- A matriz de LEDs é organizada em um padrão "zigue-zague" (snaked).
- Para linhas pares, os LEDs são dispostos da direita para a esquerda.
- Para linhas ímpares, são dispostos da esquerda para a direita.
- A fórmula ajusta a ordem para refletir isso, garantindo que as coordenadas 2D
  sejam corretamente convertidas para o índice linear do array.
*/

// Função para inicializar o joystick
void joystickInit() {
  adc_init(); // Inicializa o ADC (Conversor Analógico-Digital)

  // Configura o pino para o eixo X do joystick como entrada analógica
  adc_gpio_init(VRX_PIN);

  // Configura o pino para o eixo Y do joystick como entrada analógica
  adc_gpio_init(VRY_PIN);

  // Inicializa o pino do botão do joystick como entrada digital
  gpio_init(SW_PIN);
  gpio_set_dir(SW_PIN, GPIO_IN);

  // Habilita o pull-up interno para o botão do joystick
  // Isso mantém o estado HIGH quando o botão não está pressionado
  gpio_pull_up(SW_PIN); 
}

/*
Explicação:
- O joystick utiliza dois potenciômetros (eixo X e Y) conectados aos ADCs.
- O botão do joystick é um simples botão de pressão conectado a um pino digital.
- O pull-up interno garante que o pino fique em estado HIGH quando o botão está solto.
*/

// Função para ler o estado do joystick e atualizar a posição da raquete
void readJoystick() {
  adc_select_input(0); // Seleciona o canal ADC0 (eixo Y do joystick)
  uint16_t yValue = adc_read(); // Lê o valor analógico do eixo Y
  
  // Verifica o movimento para cima
  // Se o valor for maior que 4000 e a raquete não estiver no topo
  if (yValue > 4000 && paddleY > 0) paddleY--; // Move a raquete para cima

  // Verifica o movimento para baixo
  // Se o valor for menor que 1000 e a raquete não estiver na parte inferior
  if (yValue < 1000 && paddleY < 4) paddleY++; // Move a raquete para baixo
}

/*
Explicação:
- O eixo Y do joystick varia de 0 a 4095, sendo o centro aproximadamente 2048.
- Se o valor estiver acima de 4000, o joystick está inclinado para cima.
- Se estiver abaixo de 1000, está inclinado para baixo.
- A raquete é movida apenas se não estiver na borda superior ou inferior.
*/

// Função para criar um efeito de flash na parede (colisão da bola)
void flashWall(int x, int y) {
  // Converte as coordenadas x, y para o índice na tira de LEDs
  int pos = getIndex(x, y);

  // Executa o flash 3 vezes para criar um efeito de piscada
  for (int i = 0; i < 3; i++) {
    // Define o LED na posição 'pos' com a cor Amarela (255, 255, 0)
    npSetLED(pos, 255, 255, 0);
    npWrite(); // Atualiza a tira de LEDs
    sleep_ms(50); // Pausa de 50 ms para manter o LED aceso

    // Apaga o LED para criar o efeito de piscada
    npSetLED(pos, 0, 0, 0); // Define a cor como preta (apagado)
    npWrite(); // Atualiza a tira de LEDs
    sleep_ms(50); // Pausa de 50 ms antes de acender novamente
  }
}

/*
Explicação:
- Quando a bola colide com a parede, um efeito de piscada amarela é exibido.
- O LED correspondente pisca 3 vezes rapidamente para indicar a colisão.
- A cor amarela é obtida configurando os componentes Vermelho e Verde para 255.
*/


// Função para emitir um beep usando PWM (Pulse Width Modulation)
// pin: Pino GPIO conectado ao buzzer
// duration_ms: Duração do beep em milissegundos
void beep(uint pin, uint duration_ms) {
    // Obtém o número do slice PWM associado ao pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Define o nível do PWM para 50% do ciclo de trabalho
    // Valor 2048 de um total de 4096 (12 bits) para gerar som
    pwm_set_gpio_level(pin, 2048);

    // Mantém o som ativo pela duração especificada
    sleep_ms(duration_ms);

    // Desliga o som configurando o nível do PWM para 0
    pwm_set_gpio_level(pin, 0);

    // Pausa para evitar que os sons fiquem muito próximos
    sleep_ms(100);
}

/*
Explicação:
- A função usa PWM para controlar o buzzer piezoelétrico.
- pwm_set_gpio_level() define o nível do PWM para gerar som.
- Um ciclo de trabalho de 50% (2048 em 4096) cria um tom audível.
- O som é desligado configurando o nível do PWM para 0.
*/

// Função para animar uma explosão nos LEDs e tocar um som correspondente
// buzzerPin: Pino GPIO do buzzer usado para o som da explosão
void explodeAnimation(uint buzzerPin) {
  // Executa a animação 5 vezes para criar um efeito de explosão
  for (int i = 0; i < 5; i++) {
    // Percorre as 5 posições na coluna mais à direita
    for (int y = 0; y < 5; y++) {
      // Obtém o índice do LED na coluna 4 (última coluna)
      int pos = getIndex(4, y);

      // Define a cor do LED como Vermelho (explosão)
      npSetLED(pos, 255, 0, 0);
    }

    // Atualiza a tira de LEDs para mostrar a explosão
    npWrite();

    // Emite um som de explosão com duração maior para dar impacto
    beep(buzzerPin, 200);

    // Pausa antes de apagar os LEDs
    sleep_ms(100);

    // Apaga todos os LEDs para criar o efeito de piscada
    npClear();
    npWrite();

    // Pausa curta para o próximo ciclo de piscada
    sleep_ms(50);
  }
}

/*
Explicação:
- A animação ilumina toda a coluna mais à direita em vermelho.
- Cada ciclo consiste em acender os LEDs, emitir um som e apagar rapidamente.
- A repetição cria um efeito de "explosão" com piscadas e som impactante.
- O som é sincronizado com a iluminação para aumentar o efeito visual.
*/

// Função para inicializar o buzzer usando PWM
// pin: Pino GPIO conectado ao buzzer
void pwm_init_buzzer(uint pin) {
    // Configura o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obtém o número do slice PWM associado ao pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Obtém a configuração padrão do PWM
    pwm_config config = pwm_get_default_config();

    // Define a divisão de clock para o PWM do buzzer
    // Calcula o divisor usando a frequência do sistema e a frequência do buzzer
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));

    // Inicializa o PWM com a configuração ajustada e inicia o sinal PWM
    pwm_init(slice_num, &config, true);

    // Define o nível inicial do PWM para 0 (silencioso)
    pwm_set_gpio_level(pin, 0);
}

/*
Explicação:
- A função inicializa o PWM no pino especificado para controlar o buzzer.
- Configura o divisor de clock para obter a frequência do som desejada.
- O ciclo de trabalho é inicialmente definido como 0 para não emitir som.
- A frequência é definida usando BUZZER_FREQUENCY, configurada no começo do código.
*/

// Flag para verificar se a bola tocou na raquete do jogador
// Usada para evitar múltiplos registros de colisão em um único toque
bool touchedPaddle = false; 

// Posição inicial da raquete da CPU (coluna da esquerda)
int cpuPaddleY = 2;  

// Flag para verificar se a bola tocou na raquete da CPU
// Usada para evitar múltiplos registros de colisão na raquete da CPU
bool touchedCpuPaddle = false;  

// Velocidade de movimento da raquete da CPU
// Valores mais altos tornam a CPU mais rápida
int cpuPaddleSpeed = 2; // 1: mais lento, 2: normal

// Variável para "travar" a CPU quando a bola tocar a parede esquerda
// Impede que a CPU continue se movendo enquanto a bola não retorna
bool cpuLocked = false; 


// Função para configurar o botão de reset
// Usado para reiniciar o placar do jogo
void setupButton() {
    gpio_init(BUTTON_A);           // Inicializa o pino do botão como GPIO
    gpio_set_dir(BUTTON_A, GPIO_IN); // Define como entrada (input)
    gpio_pull_up(BUTTON_A);        // Habilita o pull-up interno para manter o estado HIGH quando não pressionado
}

/*
Explicação:
- O botão é configurado como entrada digital.
- O pull-up interno faz com que o estado padrão seja HIGH (1).
- Quando pressionado, o estado é LOW (0), permitindo detectar a pressão.
- Isso evita interferências e leituras incorretas no botão.
*/


// Função para verificar o estado do botão de reset
// Se pressionado, reinicia o placar para 0
void checkResetButton() {
    // Verifica se o botão está pressionado (estado LOW)
    if (gpio_get(BUTTON_A) == 0) { 
        playerScore = 0; // Reinicia o placar do jogador
        cpuScore = 0;    // Reinicia o placar da CPU
    }
}

/*
Explicação:
- Quando o botão é pressionado, o estado é LOW devido ao pull-up.
- Ao detectar o estado LOW, o placar é reiniciado.
- Isso permite que o jogador reinicie a partida sem reiniciar o sistema.
*/


// Função para emitir um beep personalizado usando PWM
// pin: Pino do buzzer
// freq: Frequência do som em Hertz (Hz)
// duration_ms: Duração do som em milissegundos
void beep_custom(uint pin, uint freq, uint duration_ms) {
    // Obtém o número do slice PWM para o pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Obtém a configuração padrão do PWM
    pwm_config config = pwm_get_default_config();

    // Define o divisor de clock para o PWM usando a frequência desejada
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (freq * 4096));

    // Inicializa o PWM no slice correspondente ao pino, com a configuração definida
    pwm_init(slice_num, &config, true);

    // Define o ciclo de trabalho do PWM para 50% (2048 em 4096) para gerar som
    pwm_set_gpio_level(pin, 2048);

    // Mantém o som ativo pelo tempo especificado
    sleep_ms(duration_ms);

    // Desliga o som definindo o ciclo de trabalho para 0
    pwm_set_gpio_level(pin, 0);
}

/*
Explicação:
- A função permite customizar a frequência e a duração do som do buzzer.
- A frequência do som é ajustada pela divisão do clock do sistema.
- Um ciclo de trabalho de 50% gera um tom audível.
- O som é desligado configurando o ciclo de trabalho para 0.
- Essa função é útil para gerar diferentes efeitos sonoros no jogo.
*/

// Função para atualizar a posição da bola e a lógica do jogo
// buzzerPin: Pino do buzzer usado para efeitos sonoros
void updateBall(uint buzzerPin) {
    checkResetButton(); // Verifica se o botão de reset foi pressionado

    // Atualiza a posição da bola conforme a direção atual
    ballX += ballDirX;
    ballY += ballDirY;
    
    // Verifica colisão com as paredes superior e inferior
    if (ballY <= 0 || ballY >= 4) {
        ballDirY *= -1; // Inverte a direção vertical da bola
        flashWall(ballX, ballY); // Efeito de flash na parede atingida
        beep_custom(BUZZER_A, 1200, 100); // Som de colisão com a parede
    }

    // Verifica se a bola está na coluna 0 (lado esquerdo)
    if (ballX == 0) {
        cpuLocked = true; // Travar a raquete da CPU
    } else {
        cpuLocked = false;
    }

    // Movimentação da raquete da CPU (apenas se não estiver travada)
    if (!cpuLocked) {
        // Se a bola estiver acima da raquete da CPU, mover para cima
        if (ballY < cpuPaddleY && cpuPaddleY > 0) {
            cpuPaddleY -= cpuPaddleSpeed;
        }
        // Se a bola estiver abaixo da raquete da CPU, mover para baixo
        if (ballY > cpuPaddleY && cpuPaddleY < 4) {
            cpuPaddleY += cpuPaddleSpeed;
        }
    }

    // Verifica colisão da bola com a raquete do jogador
    if (ballX == 3 && ballY == paddleY) {
        ballDirX *= -1; // Inverte a direção horizontal da bola
        touchedPaddle = true;  // Marca colisão com a raquete do jogador
        touchedCpuPaddle = false;
        beep_custom(BUZZER_A, 800, 100); // Som grave ao tocar na raquete do jogador
    }

    // Verifica colisão da bola com a raquete da CPU
    if (ballX == 0 && ballY == cpuPaddleY) {
        ballDirX *= -1; // Inverte a direção horizontal da bola
        touchedCpuPaddle = true; // Marca colisão com a raquete da CPU
        beep_custom(BUZZER_A, 1500, 100); // Som agudo ao tocar na raquete da CPU
    }

    // Verifica se a bola passou pela raquete do jogador (ponto para a CPU)
    if (ballX > 4) {
        if (touchedCpuPaddle) {
            explodeAnimation(buzzerPin); // Animação de explosão na tela
            beep_custom(BUZZER_B, 400, 500); // Som de explosão
            cpuScore++; // Incrementa o placar da CPU
        }
        // Reinicia a posição e a direção da bola
        ballX = 2;
        ballY = 2;
        ballDirX = -1;
        touchedPaddle = false;
        touchedCpuPaddle = false;
    }

    // Verifica se a bola passou pela raquete da CPU (ponto para o jogador)
    if (ballX < 0) {
        if (touchedPaddle) {
            playerScore++; // Incrementa o placar do jogador
        }
        ballDirX *= -1; // Inverte a direção horizontal
        touchedPaddle = false;
        touchedCpuPaddle = false;
        beep_custom(BUZZER_B, 500, 500); // Som ao perder ponto
    }

    // Limita o placar a no máximo 99 pontos
    if (playerScore > 99) playerScore = 99;
    if (cpuScore > 99) cpuScore = 99;
}

/*
Explicação:
- A função controla toda a lógica de movimento da bola e colisões.
- Verifica colisões com paredes, raquetes e marca pontos.
- Usa sons personalizados para indicar eventos (colisão, ponto, explosão).
- Inclui uma animação especial ao marcar ponto na CPU.
*/


// Função para desenhar os elementos do jogo nos LEDs
void drawGame() {
    npClear(); // Limpa todos os LEDs antes de redesenhar
    
    // Desenha a bola na posição atual
    int ballPos = getIndex(ballX, ballY); // Obtém o índice do LED para a posição da bola
    npSetLED(ballPos, 255, 0, 0); // Define a cor da bola como Vermelho
    
    // Desenha a raquete do jogador na coluna direita (x = 4)
    int paddlePos = getIndex(4, paddleY);
    npSetLED(paddlePos, 0, 255, 0); // Define a cor da raquete do jogador como Verde

    // Desenha a raquete da CPU na coluna esquerda (x = 0)
    int cpuPaddlePos = getIndex(0, cpuPaddleY); 
    npSetLED(cpuPaddlePos, 0, 0, 255); // Define a cor da raquete da CPU como Azul
    
    npWrite(); // Atualiza os LEDs com as novas cores
}

/*
Explicação:
- A função desenha todos os elementos visuais do jogo:
  - Bola (vermelho)
  - Raquete do jogador (verde)
  - Raquete da CPU (azul)
- Usa a função getIndex() para calcular o índice correto do LED, 
  levando em consideração o formato de zigue-zague dos LEDs WS2812B.
- Atualiza os LEDs para mostrar o estado atual do jogo.
*/


// Função para exibir o placar no display OLED
// ssd: Buffer do display
// frame_area: Área de renderização do display
void display_score(uint8_t *ssd, struct render_area *frame_area) {
    // Limpa o buffer completamente para evitar resíduos de frames anteriores
    memset(ssd, 0, ssd1306_buffer_length);  
    
    // Buffer de texto para exibir o placar
    char scoreText[25];  // Tamanho suficiente para evitar overflow
    
    // Formata o texto do placar no formato: "You: X  CPU: Y"
    snprintf(scoreText, sizeof(scoreText), "You: %d  CPU: %d", playerScore, cpuScore);

    // Desenha o texto no display na posição (0,0)
    ssd1306_draw_string(ssd, 0, 0, scoreText);  
    
    // Atualiza o display OLED com o novo texto do placar
    render_on_display(ssd, frame_area);  
}

/*
Explicação:
- A função display_score() é responsável por mostrar o placar do jogador e da CPU no display OLED.
- Limpa o buffer do display antes de desenhar o novo placar.
- Usa snprintf() para evitar problemas de buffer overflow ao formatar o texto.
- Desenha o texto na posição (0,0) para melhor visibilidade no display.
- Usa render_on_display() para atualizar o display com o conteúdo do buffer.
*/


// Função principal do jogo
int main() {
    stdio_init_all();  // Inicializa a comunicação padrão de entrada/saída

    // Inicializa o PWM nos pinos dos buzzers para efeitos sonoros
    pwm_init_buzzer(BUZZER_A);
    pwm_init_buzzer(BUZZER_B);
  
    // Inicializa o I2C para o display OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);  // Ativa o pull-up no pino SDA
    gpio_pull_up(I2C_SCL);  // Ativa o pull-up no pino SCL
    ssd1306_init();  // Inicializa o display OLED

    // Configura a área de renderização do display OLED
    struct render_area frame_area = {0, ssd1306_width - 1, 0, ssd1306_n_pages - 1};
    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];  // Buffer para armazenar os dados do display

    // Inicializa os LEDs WS2812B
    npInit(LED_PIN);

    // Inicializa o joystick para controle da raquete do jogador
    joystickInit();
    
    // Configura o botão de reset do placar
    setupButton();

    // Define qual buzzer será usado para os sons do jogo
    uint buzzerPin = BUZZER_A;  // Pode alterar para BUZZER_B, conforme necessidade

    // Loop principal do jogo (executa continuamente)
    while (true) {
        readJoystick();  // Lê a posição do joystick para controlar a raquete do jogador
        updateBall(buzzerPin); // Atualiza a posição da bola e verifica colisões
        drawGame();      // Desenha a bola e as raquetes nos LEDs WS2812B
        display_score(ssd, &frame_area); // Mostra o placar no display OLED
        sleep_ms(100);   // Pequena pausa para controlar a velocidade do jogo
    }
}

/*
Explicação:
- O loop principal mantém o jogo em execução contínua.
- Inclui as seguintes funções:
  - readJoystick(): Lê a entrada do joystick para mover a raquete do jogador.
  - updateBall(): Atualiza a posição da bola e verifica colisões com raquetes e paredes.
  - drawGame(): Desenha a bola e as raquetes nos LEDs WS2812B.
  - display_score(): Mostra o placar do jogo no display OLED.
- Usa um buzzer para emitir sons diferentes dependendo das ações do jogo.
- Utiliza LEDs WS2812B para a representação gráfica do jogo.
*/
