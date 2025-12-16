# 🧩 Projeto Final – Sistemas Embarcados – 2025.2  

# Equipe

* Álisson Brener da Silva
* Caio Lívio Leite Muniz Dantas
* Lucas do Nascimento Alves
* Vinícius Gonzaga Cavalcante Rodrigues
* Vinícius Rodrigues Barros

---
## 🎬 Video Demonstrativo do Projeto
<https://youtu.be/tNxuLWGeTUs>

---
## Mesa Labirinto Controlada por Joystick (Com Gêmeo Digital no Grafana)

Este repositório contém o desenvolvimento completo do projeto final da disciplina de **Sistemas Embarcados**, envolvendo o controle físico de uma mesa labirinto utilizando **ESP32**, **servomotores**, **joystick analógico** e **sensor MPU6050**, além da criação de um **gêmeo digital** visualizado no Grafana.

---

## 🚀 Objetivo Geral

Criar um sistema embarcado interativo capaz de controlar a inclinação de uma mesa com labirinto utilizando dois servomotores acionados por joystick analógico.  
O sistema também captura a orientação da mesa via **MPU6050** e envia os dados para o computador, onde um **gêmeo digital** exibe em tempo real os ângulos da mesa utilizando **Grafana + InfluxDB**.

---

## 🛠️ Descrição do Sistema

- A mesa se movimenta nos eixos **X** e **Y** por meio de **2 servomotores 90G**, controlados pelo **ESP32**.
- O jogador manipula um **joystick analógico** para mover a mesa e conduzir uma esfera metálica pelo labirinto.
- Um **LED** indica quando o sistema está inicializado e pronto para uso.
- Um **MPU6050** mede os ângulos **pitch** e **roll** da mesa.
- O ESP32 envia os dados via **UART** para o computador.
- Um painel no **Grafana** exibe:
  - Gráficos em tempo real de pitch e roll  
  - Uma representação visual da inclinação da mesa (gêmeo digital)

---

## 🔩 Componentes de Hardware

| Quant. | Componente          | Função |
|-------|----------------------|--------|
| 1     | ESP32                | Microcontrolador principal |
| 1     | Joystick analógico   | Controle dos eixos X e Y |
| 2     | Servo motor 90G      | Movimentação da mesa |
| 1     | MPU6050              | Leitura de pitch e roll |
| –     | LED, resistores, botões, etc. | Sinalização e debug |

---

## 💻 Recursos de Software

- **Grafana** – visualização do gêmeo digital  
- **InfluxDB** – armazenamento dos dados coletados  
- Scripts auxiliares (Python/Node.js) para leitura da porta serial

---

# 🧭 Fases do Projeto

## 🔹 Fase 1 – Controle Local da Mesa  
**Entrega: 24/11/2025**

### ✔️ Funcionalidades obrigatórias
- Leitura analógica do joystick.
- Conversão das leituras em PWM para os servos.
- Movimento proporcional e suave da mesa.
- Estruturação do código em **FreeRTOS**, com:

| Tarefa | Função |
|--------|--------|
| **Task 1** | Leitura do joystick |
| **Task 2** | Controle dos servomotores |
| **Task 3** | Monitoramento/logs via Serial |

### 📝 Critérios de Avaliação
- Funcionamento do controle físico  
- Uso correto do FreeRTOS  
- Organização e clareza do código  

---

## 🔹 Fase 2 – Leitura da Orientação e Envio de Dados  
**Entrega: 01/12/2025**

### ✔️ Funcionalidades obrigatórias
- Leitura via I²C do MPU6050.
- Cálculo dos ângulos **pitch** e **roll**.
- Envio dos dados via UART (JSON ou CSV).
- Tarefa extra no FreeRTOS exclusiva para o sensor.

### 📝 Critérios de Avaliação
- Leitura estável e coerente  
- Comunicação serial confiável  
- Integração com as tarefas da Fase 1  

---

## 🔹 Fase 3 – Gêmeo Digital e Integração com Grafana  
**Entrega: 15/12/2025**

### ✔️ Funcionalidades obrigatórias
- Configuração do **Grafana + InfluxDB**.
- Script para receber dados via serial e gravar no banco.
- Criação de dashboard com:
  - Gráfico em tempo real (pitch/roll)
  - Representação visual da mesa (gauge ou modelo gráfico)

### 📝 Critérios de Avaliação
- Visualização correta no Grafana  
- Sincronização real × virtual  
- Criatividade e qualidade da apresentação  

---

# ⭐ Extras (opcionais)

- Detecção automática de “vitória” (sensores magnéticos/ópticos).  
- Interface de calibração para o joystick (via display ou serial).

---

# 📦 Entregáveis

Cada equipe deverá entregar:

### 🎛️ 1. Protótipo físico funcional  
### 💻 2. Código-fonte documentado (neste GitHub)  
Incluindo:
- FreeRTOS com **mínimo 3 tarefas**  
- Drivers/componentes desenvolvidos  

### 📘 3. Relatório técnico contendo:
- Diagrama em blocos  
- Esquemático  
- Fluxo e descrição das tarefas FreeRTOS  
- Funcionamento do sistema  
- Bibliotecas utilizadas  
- Prints do Grafana  
- Fotos e vídeos da mesa funcionando  
- Dificuldades e soluções  

---

# 🏆 Critérios de Avaliação (Total: 100 pts)

| Critério | Peso |
|----------|------|
| Funcionamento prático e estabilidade | **30** |
| Estrutura modular do código | **20** |
| Integração sensor + atuadores + visualização | **20** |
| Documentação e relatório técnico | **20** |
| Criatividade e diferenciais | **10** |

---
