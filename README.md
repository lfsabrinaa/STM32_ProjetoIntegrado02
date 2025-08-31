# Simulador de Lançamento de Carga em STM32

Este é um projeto de simulação de lançamento de cargas desenvolvido para a plataforma de microcontroladores STM32. O sistema foi criado como projeto integrado das disciplinas de Sistemas Embarcados (SE) e Linguagem de Programação (LPR), oferecendo uma experiência interativa em hardware embarcado, com elementos gráficos exibidos em uma tela ST7735.

### 🎯 Funcionalidades

* **Simulação Física do Lançamento:** O programa calcula a trajetória da carga considerando gravidade, vento, massa e formato do pacote, incluindo forças de arrasto.

* **Interface Visual:** O cenário (céu, avião, alvo e carga) é exibido em uma tela gráfica TFT ST7735.

* **Controle por Botões:** O usuário configura parâmetros (velocidade do avião, força/direção do vento, massa e formato da carga) diretamente pelos botões da placa.

* **Feedback Interativo:** Após cada tentativa, o sistema mostra se a carga atingiu o alvo e, ao final, apresenta um resumo com estatísticas de desempenho (acertos, erro médio e tempo médio).

### 🛠️ Tecnologias Utilizadas

* **Microcontrolador:** STM32

* **Display:** TFT ST7735 (via SPI)

* **Linguagem de Programação:** C

* **Ambiente de Desenvolvimento:** STM32CubeIDE

* **Bibliotecas:** HAL (STM32), ST7735

### 📚 Proposta e Orientação

Este projeto foi desenvolvido como parte do curso técnico de Desenvolvimento de Sistemas da ETE FMC (2025), sob orientação dos professores:

* **Profa. Ana Letícia:** Disciplina de Sistemas Embarcados
* **Prof. José Andery:** Disciplina de Linguagem de Programação

### 👩‍💻 Autoria

Projeto desenvolvido por Sabrina Lopes, Thaiza e Yasmim.
