# Exemplo de FreeRTOS no Arduino

Esse é um exemplo de uso do FreeRTOS, um Sistema Operacional de Tempo Real(RTOS), em um Arduino.
O sistema consiste de um Arduino Nano, um display LCD 16X02 (com o módulo LCM IC2), um potenciômetro de 100kOhm, um LED
e outros componentes(resistores 330 Ohm, cabos Jumper, etc.). O código foi adaptado para melhor funcionamento.

## Descrição
Por meio do potenciômetro, é possível regular a tensão, que é exibida no display. Se a tensão for maior do que o limiar definido(3.8 V) o LED acenderá. Caso contrário, o Led permanece apagado.
Isso é feito por meio de três tarefas: task_sensor, task_lcd e task_led.

## Tarefas
- **task_sensor**:
É responsável por ler a informação recebida pelo potenciômetro e calcular a tensão. Depois, passa a informação da tensão calculada(por meio de queues) para as outras duas tarefas.

- **task_lcd**:
Recebe a informação, por meio da fila, e exibe de forma amigável no display LCD, limpando e reescrevendo quando necessário.

- **task_led**:
Ao receber a informação da tensão, e verifica se é maior que o limiar definido em LED_THRESHOLD. Se for, acende o LED, caso contrário, apaga.

## Demonstração

[Vídeo do projeto em ação](https://www.youtube.com/watch?v=uqHNpq9N7FE)

## Uso do *Semaphore*
É utilizado o recurso *Semaphore*("Semáforo") do FreeRTOS, que permite um controle de acesso a recursos únicos para que estes possam ser compartilhados entre as tarefas.
Quando uma tarefa precisa do acesso, ela solicita o controle do Semáforo com _xSemaphoreTake()_. Após terminar, o Semáforo é liberado com  *xSemaphoreGive()*.
