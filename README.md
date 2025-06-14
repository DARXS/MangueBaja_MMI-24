# MangueBaja Man Machine Interface - 24: Firmware da Unidade Frontal (ECU Frontal)

Este repositório contém o firmware embarcado para a Unidade de Controle Frontal, também referida como MMI (Man-Machine Interface), do projeto Mangue Baja. Desenvolvido sobre o Mbed OS, este firmware é projetado para rodar em uma placa de desenvolvimento NUCLEO-F103RB.

A ECU Frontal é responsável por tarefas críticas de processamento de sensores, fusão de dados, controle de atuadores e comunicação com outras unidades do veículo através da rede CAN.

## Funcionalidades Técnicas

-   **Processamento e Fusão de Dados de IMU**: O sistema interfaceia com um sensor IMU (Acelerômetro e Giroscópio) de 6 eixos, o LSM6DS3, via I2C. Os dados brutos dos sensores são lidos e processados para calcular os ângulos de inclinação do veículo (pitch e roll).

-   **Filtragem Avançada com Filtro de Kalman**: Para obter uma estimativa de ângulo precisa e estável, o firmware implementa um Filtro de Kalman. Este filtro combina as leituras de curto prazo do giroscópio (baixa deriva) com as leituras de longo prazo do acelerômetro (referência gravitacional estável) para mitigar o ruído e a deriva de cada sensor individualmente.

-   **Cálculo de RPM com Filtro FIR**: A rotação por minuto (RPM) do motor é calculada a partir de um sensor de frequência externo, conectado a uma entrada de interrupção. Para suavizar a leitura de RPM, que pode ser ruidosa, um filtro digital FIR (Finite Impulse Response) customizado é aplicado.

-   **Arquitetura Orientada a Eventos com Mbed OS**: O firmware é construído sobre o Mbed OS e utiliza uma arquitetura de máquina de estados orientada a eventos. `Tickers` (temporizadores) e interrupções de hardware não executam lógica complexa diretamente, mas sim adicionam "estados" a uma fila de eventos (`EventQueue` e `CircularBuffer`). O loop principal do sistema consome esses eventos da fila de forma sequencial, garantindo um comportamento determinístico e prevenindo condições de corrida.

-   **Comunicação em Rede CAN**: Atua como um nó ativo na rede CAN do veículo, com uma taxa de 1000 Kbps. É responsável por:
    -   **Transmitir** dados processados, como os dados do IMU (aceleração, giroscópio, ângulos), RPM e status de atuadores, em pacotes com IDs específicos (`IMU_ACC_ID`, `RPM_ID`, `FLAGS_ID`).
    -   **Receber** dados de outras ECUs, como velocidade, temperatura do motor, status da bateria (SoC) e temperatura da CVT, para controle e exibição.

-   **Controle de Atuadores e Leitura de Sensores**: Gerencia I/Os digitais para ler o estado de chaves (modos "RUN" e "CHOKE" do motor) e de botões (acionamento do 4x4), implementando lógica de debounce por software para garantir leituras confiáveis.

-   **Comunicação Serial para Display**: Envia um pacote de dados estruturado (`Txtmng`) via comunicação serial, destinado a um painel ou display para visualização do piloto em tempo real.

## Hardware e Tech Stack

-   **Placa**: ST NUCLEO-F103RB
-   **Sistema Operacional**: Mbed OS
-   **Pinos de Hardware Principais**:
    -   **CAN**: `PB_8` (RD), `PB_9` (TD)
    -   **Serial**: `PA_2` (TX), `PA_3` (RX)
    -   **I2C (IMU)**: `PB_7` (SDA), `PB_6` (SCL)
    -   **Interrupts**: `PA_0` (4x4), `PB_4` (Sensor de Frequência), `PA_7` (Chave Choke), `PA_5` (Chave Run)
-   **Bibliotecas Customizadas**:
    -   `LSM6DS3`: Driver para o sensor IMU.
    -   `Kalman`: Implementação do Filtro de Kalman para fusão de sensores.
    -   `CANMsg`: Classe wrapper para a `CANMessage` do Mbed, facilitando a construção de pacotes CAN.
    -   `FIR`: Filtro FIR para suavização de sinais.
-   **Configuração de Build**:
    -   As configurações do Mbed OS, como tamanho de stack e otimizações, são definidas em `mbed_config.h` e `mbed_app.json`.

## Arquitetura do Firmware

A arquitetura é fundamentada em uma máquina de estados não-bloqueante, orquestrada pelos recursos do Mbed OS.

1.  **Geração de Eventos**:
    -   **Interrupções de Hardware** (`canISR`, `frequencyCounterISR`, `servoSwitchISR`): São disparadas por eventos externos (chegada de mensagem CAN, pulso do sensor de RPM, mudança de estado das chaves). Elas executam o mínimo de código possível e delegam a lógica principal para a `EventQueue`.
    -   **Tickers (Interrupções de Tempo)**: Três `Tickers` são configurados para disparar em frequências diferentes (1Hz, 5Hz, 20Hz), enfileirando os estados correspondentes (`FLAGS_ST`, `RPM_ST`, `IMU_ST`, etc.) no `CircularBuffer`.

2.  **Fila de Eventos (`EventQueue` & `CircularBuffer`)**:
    -   Atua como um buffer entre os geradores de eventos (interrupções) e o consumidor (loop principal). Isso garante que nenhum evento seja perdido e que a lógica de tratamento seja executada no contexto da thread principal, evitando problemas de concorrência.

3.  **Processamento no Loop Principal**:
    -   O `while(true)` do `main.cpp` verifica continuamente se há um novo estado no `CircularBuffer`.
    -   Um `switch-case` direciona a execução para a função ou bloco de código apropriado para tratar o estado atual (ex: ler o IMU no estado `IMU_ST`, calcular o RPM no estado `RPM_ST`).
    -   Após o processamento, o loop volta a esperar pelo próximo evento, mantendo o processador livre para outras tarefas do Mbed OS.

## Estrutura e Descrição dos Arquivos

```
MangueBaja_MMI/
├── main.cpp                  # Lógica principal, máquina de estados e inicializações
├── mbed-os.lib               # Referência para a biblioteca do Mbed OS
├── Other_things/             # Arquivos de exemplo e configuração do Mbed
├── BUILD/                    # Arquivos de compilação gerados, incluindo mbed_config.h
├── BAJADefs/                 # Definições específicas do projeto
│   ├── defs.h
│   ├── FIR.h
│   └── front_defs.h
├── CANMsg/                   # Wrapper para a classe CANMessage
│   └── CANMsg.h
├── Kalman/                   # Implementação do Filtro de Kalman
│   ├── Kalman.cpp
│   └── Kalman.h
└── LSM6DS3/                  # Driver para o sensor IMU
├── LSM6DS3.cpp
└── LSM6DS3.h
```
### Descrição Detalhada dos Arquivos

-   **`main.cpp`**: O coração do firmware. Contém a função `main()`, a definição de todos os objetos de hardware (CAN, Serial, I2C, IOs), a configuração das interrupções e tickers, e o loop infinito que processa a máquina de estados a partir do `CircularBuffer`.

-   **`LSM6DS3/LSM6DS3.h` & `LSM6DS3.cpp`**: Driver para o sensor IMU LSM6DS3. Abstrai a comunicação I2C e a configuração dos registradores do sensor para leitura dos dados de aceleração e giroscópio.

-   **`Kalman/Kalman.h` & `Kalman.cpp`**: Implementação do Filtro de Kalman. A classe `Kalman` recebe os novos valores de ângulo (do acelerômetro) e taxa de variação (do giroscópio) e retorna um ângulo filtrado. Inclui métodos para ajustar os parâmetros do filtro (Q_angle, Q_bias, R_measure).

-   **`CANMsg/CANMsg.h`**: Uma classe C++ que herda de `CANMessage` do Mbed. Sobrescreve os operadores `<<` e `>>` para facilitar a inserção e extração de dados de tipos variados no payload de 8 bytes da mensagem CAN.

-   **`BAJADefs/defs.h`**: Define constantes e estruturas de dados globais para a rede CAN, como os IDs de cada mensagem (ex: `RPM_ID`, `SPEED_ID`), garantindo consistência entre todas as ECUs do projeto.

-   **`BAJADefs/front_defs.h`**: Define constantes e enumerações específicas da ECU Frontal, como os fatores de conversão para o IMU (`TO_G`, `TO_DPS`) e a enumeração dos estados (`state_t`) da máquina de estados.

-   **`BAJADefs/FIR.h`**: Implementação de um filtro FIR (Finite Impulse Response) em C++. É utilizado para suavizar o sinal de RPM calculado.

-   **`BUILD/NUCLEO_F103RB/ARMC6/mbed_config.h`**: Arquivo gerado automaticamente pelo Mbed CLI durante a compilação. Contém todas as macros de configuração do Mbed OS, definindo parâmetros do kernel, stacks, drivers e bibliotecas utilizadas.
