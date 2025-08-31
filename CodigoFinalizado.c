/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735/st7735.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CONFIGURAÇÃO DE TELA
#define LARGURA_TELA    160     // pixels
#define ALTURA_TELA     80      // pixels

// CONSTANTES FÍSICAS
#define G               100.0   // gravidade (ms/s²) >> não está realista (=9.8) porque a simulação ficaria muito lenta
#define RHO_AR          1.225f  // densidade do ar (kg/m³)
#define CD_QUADRADO     1.05f   // coeficiente de arrasto do quadrado
#define CD_CIRCULO      0.47f   // coeficiente de arrasto da esfera
#define DT              0.1f    // passo de tempo para simulação (s)

// CORES (formato RGB565)
#define AZUL_CEU        0xAEDF
#define VERDE_GRAMA     0x03E0
#define MARROM_TRONCO   0x8A22
#define VERDE_COPA      0x05E0
#define CYAN            0x07FF
#define WHITE           0xFFFF
#define BLACK           0x0000
#define MAGENTA         0xF81F

// DIMENSÕES DE OBJETOS
#define LARGURA_AVIAO   15      // pixels
#define ALTURA_AVIAO    5       // pixels
#define TAMANHO_CARGA   4       // pixels
#define ALVO_Y          (ALTURA_TELA - 10) // posição Y do alvo no chão

// PARÂMETROS DE SIMULAÇÃO
#define DELAY_QUEDA     50      // ms entre frames da queda
#define DELAY_MENU      500     // ms para efeitos no menu
#define DELAY_TELA      5000    // ms para transições longas
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    int tentativa;
    int x_inicial, y_inicial;
    int x_alvo, y_alvo, lado_alvo;
    int forca_vento, direcao_vento;
    int x_final, y_final;
    float tempo;
    int acertou;
} Tentativa;

// funções elaboradas pelo trio
void menu(void);
int definirParametro(const char *titulo, int valor, int passo, int min, int max);
void mostrarPainelParametros(void);
void desenhar_cenario(void);
void desenhar_aviao(int x, int y);
void apagar_aviao(int x, int y);
void desenhar_cargaQuad(int x, int y);
void apagar_cargaQuad(int x, int y);
void desenhar_cargaRed(int x, int y);
void apagar_cargaRed(int x, int y);
void SinalizarTentativa(Tentativa *t);

// funções obrigatórias conforme o escopo
void ConfigurarParametros(void);
Tentativa InicializarSimulacao(int numero);
void ExecutarLancamento(Tentativa *t);
void RegistrarTentativa(Tentativa *t, float x_final, float tempo, float y_final);
void AvaliarTentativa(Tentativa *t);
void ExibirResumoFinal(Tentativa registros[], int total);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
int velocidade_inicial = 100;    // m/s (velocidade escalar do avião)
int forca_vento = 5;             // m/s (velocidade do vento na horizontal)
int direcao_vento = 0;           // 0 = da direita p/ esquerda (contra o movimento da carga), 1 = da esquerda p/ direita (a favor)
int massa_pacote = 5;            // kg
int formato_pacote = 0;          // 0 = quadrado, 1 = redondo
int tentativas = 3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  menu();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735_DC_Pin|ST7735_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ST7735_CS_Pin */
  GPIO_InitStruct.Pin = ST7735_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST7735_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_DC_Pin ST7735_RES_Pin */
  GPIO_InitStruct.Pin = ST7735_DC_Pin|ST7735_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// ==== MENU E DESENHOS ====
void menu(void) {
    ST7735_FillScreen(AZUL_CEU);
    ST7735_FillRectangle(0, ALTURA_TELA - 10, LARGURA_TELA, 10, VERDE_GRAMA);
    ST7735_FillRectangle(5, ALTURA_TELA - 14, 4, 6, MARROM_TRONCO);
    ST7735_FillRectangle(2, ALTURA_TELA - 18, 10, 6, VERDE_COPA);
    HAL_Delay(500);

    ST7735_WriteString(0, 20, "BEM-VINDO AO SIMULADOR DE LANCAMENTO DE CARGA DA TURMA", Font_7x10, BLACK, AZUL_CEU);
    ST7735_WriteString(60, 40, "34DS!", Font_7x10, MAGENTA, AZUL_CEU);
    HAL_Delay(2000);

    ST7735_FillScreen(AZUL_CEU);
    ST7735_FillRectangle(0, ALTURA_TELA - 10, LARGURA_TELA, 10, VERDE_GRAMA);
    ST7735_FillRectangle(5, ALTURA_TELA - 14, 4, 6, MARROM_TRONCO);
    ST7735_FillRectangle(2, ALTURA_TELA - 18, 10, 6, VERDE_COPA);

    ST7735_WriteString(0, 25, "Pressione qualquer", Font_7x10, BLACK, AZUL_CEU);
    ST7735_WriteString(0, 35, "botao para continuar!", Font_7x10, BLACK, AZUL_CEU);

    while (1) {
        if (
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET
        ) {
            HAL_Delay(200);
            break;
        }
    }

    ST7735_FillScreen(AZUL_CEU);
    ST7735_FillRectangle(0, ALTURA_TELA - 10, LARGURA_TELA, 10, VERDE_GRAMA);
    ST7735_FillRectangle(5, ALTURA_TELA - 14, 4, 6, MARROM_TRONCO);
    ST7735_FillRectangle(2, ALTURA_TELA - 18, 10, 6, VERDE_COPA);

    ST7735_WriteString(0, 25, "Entender Proposta (9)", Font_7x10, BLACK, AZUL_CEU);
    ST7735_WriteString(0, 35, "Iniciar Simulacao (11)", Font_7x10, BLACK, AZUL_CEU);

    while (1) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET) {
            HAL_Delay(200);
            ST7735_FillScreen(AZUL_CEU);
            ST7735_FillRectangle(0, ALTURA_TELA - 10, LARGURA_TELA, 10, VERDE_GRAMA);
            ST7735_FillRectangle(5, ALTURA_TELA - 14, 4, 6, MARROM_TRONCO);
            ST7735_FillRectangle(2, ALTURA_TELA - 18, 10, 6, VERDE_COPA);

            ST7735_WriteString(0, 10, "Projeto Integrado", Font_7x10, BLACK, AZUL_CEU);
            ST7735_WriteString(0, 20, "do 2o trimestre", Font_7x10, BLACK, AZUL_CEU);
            ST7735_WriteString(0, 30, "nas materias LPR e SE,", Font_7x10, BLACK, AZUL_CEU);
            ST7735_WriteString(0, 40, "no curso tecnico de DS", Font_7x10, BLACK, AZUL_CEU);
            ST7735_WriteString(0, 50, "da ETE FMC, em 2025.", Font_7x10, BLACK, AZUL_CEU);
            HAL_Delay(4500);
            break;
        }
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
            HAL_Delay(200);
            ConfigurarParametros();
        }
    }
}

void desenhar_cenario(void) {
    ST7735_FillScreen(AZUL_CEU);
    ST7735_FillRectangle(0, ALTURA_TELA - 10, LARGURA_TELA, 10, VERDE_GRAMA);
    ST7735_FillRectangle(5, ALTURA_TELA - 14, 4, 6, MARROM_TRONCO);
    ST7735_FillRectangle(2, ALTURA_TELA - 18, 10, 6, VERDE_COPA);
    ST7735_FillRectangle(5, 2, 10, 4, WHITE);
    ST7735_FillRectangle(8, 0, 8, 4, WHITE);
    ST7735_FillRectangle(130, 3, 12, 4, WHITE);
    ST7735_FillRectangle(133, 1, 8, 3, WHITE);
    ST7735_FillRectangle(60, 9, 12, 4, WHITE);
    ST7735_FillRectangle(63, 7, 8, 4, WHITE);
}

void desenhar_aviao(int x, int y) {
    ST7735_FillRectangle(x, y, LARGURA_AVIAO, ALTURA_AVIAO, CYAN);
}

void apagar_aviao(int x, int y) {
    ST7735_FillRectangle(x, y, LARGURA_AVIAO, ALTURA_AVIAO, AZUL_CEU);
}

void desenhar_cargaQuad(int x, int y) {
    ST7735_FillRectangle(x, y, TAMANHO_CARGA, TAMANHO_CARGA, MAGENTA);
}

void apagar_cargaQuad(int x, int y) {
    ST7735_FillRectangle(x, y, TAMANHO_CARGA, TAMANHO_CARGA, AZUL_CEU);
}

void desenhar_cargaRed(int x, int y) {
    int r = TAMANHO_CARGA / 2;
    int cx = x + r;
    int cy = y + r;

    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrt(r * r - dy * dy);
        ST7735_FillRectangle(cx - dx, cy + dy, 2 * dx, 1, RED);
    }
}

void apagar_cargaRed(int x, int y) {
    int r = TAMANHO_CARGA / 2;
    int cx = x + r;
    int cy = y + r;

    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrt(r * r - dy * dy);
        ST7735_FillRectangle(cx - dx, cy + dy, 2 * dx, 1, AZUL_CEU);
    }
}

// ==== CONFIGURAR PARÂMETROS ====
// Painel interativo para exibir os parâmetros após as definições.
void mostrarPainelParametros(void) {
    char buf[32];

    ST7735_FillScreen(WHITE);

    sprintf(buf, "Vel. Aviao: %d m/s", velocidade_inicial);
    ST7735_WriteString(0, 10, buf, Font_7x10, BLACK, WHITE);

    sprintf(buf, "Vel.do Vento: %d m/s", forca_vento);
    ST7735_WriteString(0, 20, buf, Font_7x10, BLACK, WHITE);

    sprintf(buf, "Dir. do Vento: %s", direcao_vento == 0 ? "Esq" : "Dir");
    ST7735_WriteString(0, 30, buf, Font_7x10, BLACK, WHITE);

    sprintf(buf, "Massa do Pacote: %d kg", massa_pacote);
    ST7735_WriteString(0, 40, buf, Font_7x10, BLACK, WHITE);

    sprintf(buf, "Formato do Pacote: %s", formato_pacote == 0 ? "Qua" : "Red");
    ST7735_WriteString(0, 50, buf, Font_7x10, BLACK, WHITE);

    sprintf(buf, "Tentativas: %d",tentativas);
    ST7735_WriteString(0, 60, buf, Font_7x10, BLACK, WHITE);
}

// Função genérica para editar parâmetros de entrada, como velocidade do avião e do vento, massa da carga, etc.
int definirParametro(const char *titulo, int valor, int passo, int min, int max) {
	ST7735_FillScreen(WHITE);
    uint32_t ultimo_tempo = HAL_GetTick();
    char buf[12];

    while (1) {
        ST7735_WriteString(12, 20, titulo, Font_7x10, BLACK, WHITE);

        sprintf(buf, "%d", valor);
        ST7735_WriteString(12, 40, buf, Font_7x10, BLACK, WHITE);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET &&
            (HAL_GetTick() - ultimo_tempo > 150)) {
            if (valor + passo <= max) valor += passo;
            ultimo_tempo = HAL_GetTick();
        }
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET &&
            (HAL_GetTick() - ultimo_tempo > 150)) {
            if (valor - passo >= min) valor -= passo;
            ultimo_tempo = HAL_GetTick();
        }
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET &&
            (HAL_GetTick() - ultimo_tempo > 150)) {
            return valor;
        }
    }
}

// Atribui valor às variáveis.
void ConfigurarParametros(void) {
    Tentativa registros[3];

    while (tentativas > 0) {
    	int i = 3 - tentativas;

        if (tentativas == 3) {
            ST7735_FillScreen(BLACK);
            ST7735_WriteString(0, 10, "Para iniciarmos,", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 20, "defina a seguir", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 30, "alguns parametros.", Font_7x10, WHITE, BLACK);
            HAL_Delay(3500);
        } else {
            ST7735_FillScreen(BLACK);
            ST7735_WriteString(0, 10, "Para continuarmos,", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 20, "redefina os valores", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 30, "dos parametros.", Font_7x10, WHITE, BLACK);
            HAL_Delay(3500);
        }

        velocidade_inicial = definirParametro("Vel. do aviao:", velocidade_inicial, 5, 50, 300);
        forca_vento        = definirParametro("Vel. do vento:", forca_vento, 1, 1, 20);
        massa_pacote       = definirParametro("Massa do pacote:", massa_pacote, 1, 1, 20);
        direcao_vento      = definirParametro("Direcao do vento:", direcao_vento, 1, 0, 1);
        formato_pacote     = definirParametro("Formato do pacote:", formato_pacote, 1, 0, 1);

        HAL_Delay(1000);
        mostrarPainelParametros();
        HAL_Delay(2500);

        registros[i] = InicializarSimulacao(i+1);
        tentativas--;
    }

    ExibirResumoFinal(registros, 3);
}
// ==== INICIALIZAR SIMULAÇÃO ====
Tentativa InicializarSimulacao(int numero) {
    int minL = (int)(0.7 * LARGURA_TELA);
    int maxL = (int)(0.9 * LARGURA_TELA);
    int minA = (int)(0.1 * ALTURA_TELA);
    int maxA = (int)(0.3 * ALTURA_TELA);

    Tentativa t;
    t.tentativa = numero;
    t.y_inicial = minA + rand() % (maxA - minA); // altitude aleatória do avião
    t.x_alvo    = minL + rand() % (maxL - minL); // posição X do alvo ALEATORIZADA
    //t.x_alvo    = 140; // posição X do alvo FIXA
    t.lado_alvo = 10;
    t.y_alvo    = ALVO_Y;
    t.forca_vento   = forca_vento;
    t.direcao_vento = direcao_vento;

    t.x_inicial = 0;

    ExecutarLancamento(&t);

    return t;
}
// ==== EXECUTAR LANÇAMENTO ====
void ExecutarLancamento(Tentativa *t) {
    float coefArrasto = (formato_pacote == 0) ? CD_QUADRADO : CD_CIRCULO;
    float area = (formato_pacote == 0) ? 0.04f : 0.0314f;

    // Velocidade inicial do avião
    float Vx = (float)velocidade_inicial;
    float Vy = 0.0f;

    // Posição inicial do pacote (meio do avião)
    float x = (float)t->x_inicial + LARGURA_AVIAO / 2 - TAMANHO_CARGA / 2;
    float y = (float)t->y_inicial + ALTURA_AVIAO;

    float dt = 0.1f;
    float tempoSim = 0.0f;

    desenhar_cenario();
    desenhar_aviao(t->x_inicial, t->y_inicial);

    // Desenhar alvo
    int alvoX = t->x_alvo;
    int alvoY = t->y_alvo;
    ST7735_FillRectangle(alvoX, alvoY, t->lado_alvo, t->lado_alvo, GREEN);

    HAL_Delay(500);

    bool pacoteEmQueda = true;
    bool tentativaRegistrada = false;

    while (t->x_inicial < LARGURA_TELA || pacoteEmQueda) {
        apagar_aviao(t->x_inicial, t->y_inicial);

        if (pacoteEmQueda) {
            // Apagar posição anterior do pacote
            if (y > 0 && y < ALTURA_TELA) {
                if (formato_pacote == 0) apagar_cargaQuad((int)x, (int)y);
                else apagar_cargaRed((int)x, (int)y);
            }

            // Guarda posição anterior
            float x_ant = x;
            float y_ant = y;

            // Força de arrasto no eixo X
            float Fd_x = 0.5f * RHO_AR * coefArrasto * area * Vx * Vx;
            float ax = -(Fd_x / (float)massa_pacote) * ((Vx > 0) ? 1.0f : -1.0f);

            // Força de arrasto no eixo Y
            float Fd_y = 0.5f * RHO_AR * coefArrasto * area * Vy * Vy;
            float ay = G - (Fd_y / (float)massa_pacote) * ((Vy > 0) ? 1.0f : -1.0f);

            // Aceleração do vento (constante)
            float ax_vento = (t->direcao_vento == 1 ? +t->forca_vento : -t->forca_vento) / (float)massa_pacote;
            ax += ax_vento;

            // Atualiza velocidades
            Vx += ax * dt;
            Vy += ay * dt;

            // Atualiza posições
            x += Vx * dt;
            y += Vy * dt;

            // Verifica colisão com chão ou alvo
            if (y >= ALTURA_TELA ||
                (x >= alvoX && x <= alvoX + t->lado_alvo &&
                 y >= t->y_alvo && y <= t->y_alvo + t->lado_alvo)) {

                // Interpolação linear para calcular ponto exato de impacto
                float limiteY = (y >= ALTURA_TELA) ? ALTURA_TELA : t->y_alvo;
                float frac = (limiteY - y_ant) / (y - y_ant);
                float x_col = x_ant + frac * (x - x_ant);
                float y_col = y_ant + frac * (y - y_ant);

                pacoteEmQueda = false;
                RegistrarTentativa(t, x_col, tempoSim + frac * dt, y_col);
                tentativaRegistrada = true;
            }

            // Desenha pacote se ainda está no ar
            if (pacoteEmQueda && y > 0 && y < ALTURA_TELA) {
                if (formato_pacote == 0) desenhar_cargaQuad((int)x, (int)y);
                else desenhar_cargaRed((int)x, (int)y);
            }
        }

        // Atualizar posição do avião
        t->x_inicial += (int)(velocidade_inicial * dt);
        desenhar_aviao(t->x_inicial, t->y_inicial);

        HAL_Delay(100);
        tempoSim += dt;
    }

    if (!tentativaRegistrada) {
        RegistrarTentativa(t, x, tempoSim, y);
    }

    SinalizarTentativa(t);
    HAL_Delay(2000);
}
// ==== AVALIAR TENTATIVA ====
void AvaliarTentativa(Tentativa *t) {
    // Retângulo do alvo
    int alvoMinX = t->x_alvo;
    int alvoMaxX = t->x_alvo + t->lado_alvo;
    int alvoMinY = t->y_alvo;
    int alvoMaxY = t->y_alvo + t->lado_alvo;

    // Retângulo da carga (considerando tamanho do quadrado)
    int cargaMinX = t->x_final;
    int cargaMaxX = t->x_final + TAMANHO_CARGA;
    int cargaMinY = t->y_final;
    int cargaMaxY = t->y_final + TAMANHO_CARGA;

    // Verifica interseção entre os dois retângulos
    if (cargaMaxX >= alvoMinX && cargaMinX <= alvoMaxX &&
        cargaMaxY >= alvoMinY && cargaMinY <= alvoMaxY) {
        t->acertou = 1;
    } else {
        t->acertou = 0;
    }
}

void SinalizarTentativa(Tentativa *t) {
	if (t->acertou) {
        ST7735_WriteString(30, 70, "ACERTO!", Font_7x10, GREEN, BLACK);
    } else {
        ST7735_WriteString(30, 70, "ERRO!", Font_7x10, RED, BLACK);
    }
}
// ==== REGISTRAR TENTATIVA ====
void RegistrarTentativa(Tentativa *t, float x_final, float tempo, float y_final) {
    t->x_final = (int)x_final;
    t->y_final = (int)y_final;
    t->tempo = tempo;

    AvaliarTentativa(t);
}
// ==== EXIBIR RESUMO FINAL ====
void ExibirResumoFinal(Tentativa registros[], int total) {
    ST7735_FillScreen(BLACK);
    ST7735_WriteString(0, 0, "--- Resumo Final ---", Font_7x10, WHITE, BLACK);

    int acertos = 0;
    float somaTempos = 0.0f;
    float somaErros = 0.0f;

    char buf[64];

    for (int i = 0; i < total; i++) {
        sprintf(buf, "Tentativa %d:", registros[i].tentativa);
        ST7735_WriteString(0, 10 + (i * 20), buf, Font_7x10, WHITE, BLACK);

        // Distância do impacto ao centro do alvo
        int centroAlvoX = registros[i].x_alvo + registros[i].lado_alvo / 2;
        int centroAlvoY = registros[i].y_alvo + registros[i].lado_alvo / 2;
        float dx = (float)registros[i].x_final - (float)centroAlvoX;
        float dy = (float)registros[i].y_final - (float)centroAlvoY;
        float dist = sqrtf(dx * dx + dy * dy);

        somaTempos += registros[i].tempo;
        somaErros  += dist;

        if (registros[i].acertou) {
            sprintf(buf, "  ACERTO! (%d,%d)", registros[i].x_final, registros[i].y_final);
            ST7735_WriteString(0, 20 + (i * 20), buf, Font_7x10, GREEN, BLACK);
            acertos++;
        } else {
            sprintf(buf, "  ERRO!   (%d,%d)", registros[i].x_final, registros[i].y_final);
            ST7735_WriteString(0, 20 + (i * 20), buf, Font_7x10, RED, BLACK);
        }
    }

    float tempoMedio = somaTempos / (float)total;          // segundos
    float erroMedio  = somaErros  / (float)total;          // pixels
    float porcent    = ((float)acertos * 100.0f) / (float)total;

    int t100 = (int)(tempoMedio * 100.0f + 0.5f);          // ex: 1.23s -> 123
    int e10  = (int)(erroMedio  * 10.0f  + 0.5f);          // ex: 7.4px -> 74
    int p10  = (int)(porcent    * 10.0f  + 0.5f);          // ex: 66.7% -> 667

    HAL_Delay(3000);

	ST7735_FillRectangle(0, 10, LARGURA_TELA, ALTURA_TELA-10, BLACK);

	sprintf(buf, "Acertos: %d/%d (%d.%01d%%)", acertos, total, p10/10, p10%10);
	ST7735_WriteString(0, 20, buf, Font_7x10, CYAN, BLACK);

	sprintf(buf, "Tempo medio: %d.%02ds", t100/100, t100%100);
	ST7735_WriteString(0, 35, buf, Font_7x10, WHITE, BLACK);

	sprintf(buf, "Erro medio: %d.%01d px", e10/10, e10%10);
	ST7735_WriteString(0, 50, buf, Font_7x10, WHITE, BLACK);

	HAL_Delay(3000);

	ST7735_FillRectangle(0, 10, LARGURA_TELA, ALTURA_TELA-10, BLACK);

	if (acertos == total) {
		ST7735_WriteString(0, 30, "Excelente trabalho!", Font_7x10, GREEN, BLACK);
	} else if (acertos > 0) {
		ST7735_WriteString(0, 30, "Bom trabalho!", Font_7x10, YELLOW, BLACK);
	} else {
		ST7735_WriteString(0, 30, "Continue praticando!", Font_7x10, RED, BLACK);
	}

	HAL_Delay(3000);

	ST7735_FillScreen(WHITE);
	ST7735_WriteString(0, 30, "Feito por Sabrina,    Thaiza e Yasmim. :)", Font_7x10, MAGENTA, WHITE);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
