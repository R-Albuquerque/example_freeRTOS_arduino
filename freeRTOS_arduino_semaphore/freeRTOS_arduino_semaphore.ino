#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
 
/* definições - LCD */
#define LCD_16X2_CLEAN_LINE                "                "
#define LCD_16X2_I2C_ADDRESS             0x27
#define LCD_16X2_COLS                      16
#define LCD_16X2_ROWS                      2
 
/* definições - LED */
#define LED_PIN                      2 /* Pino do LED, nesse caso é o D2*/
#define LED_THRESHOLD                3.58 /* Limiar Mínimo para acendimento do LED. A partir de que voltagem o LED será aceso */
 
/* definições - ADC */
#define ADC_MAX                      1023.0
#define MAX_VOLTAGE_ADC              5.0

/* Type definitions (definições de tipo) */
#define portCHAR    char
 
/* Tarefas (tasks) */
void task_lcd( void *pvParameters );
void task_sensor( void *pvParameters );
void task_led( void *pvParameters );
 
/* Definindo o LCD, de acordo com as definições */
LiquidCrystal_I2C lcd(LCD_16X2_I2C_ADDRESS,
                      LCD_16X2_COLS,
                      LCD_16X2_ROWS);
 
/* Definindo filas (queues) */
QueueHandle_t xQueue_LCD, xQueue_LED;
 
/* semaforos utilizados */
SemaphoreHandle_t xSerial_semaphore;
 
void setup() {
  
  /* Inicializa serial (baudrate 9600) */
  Serial.begin(9600);
 

  lcd.init(); /* Inicia o LCD */
  lcd.backlight(); /* Liga o backlight(luz de fundo) do LCD */
  lcd.clear(); /* Limpa o LCD */
 
  /* Inicializa e configura GPIO do LED */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  while (!Serial) {
    ; /* Esperar até a Serial estar pronta */
  }
 
  /* Criando as filas(queues) */
  xQueue_LCD = xQueueCreate( 1, sizeof( float ) ); /* Queue do LCD */
  xQueue_LED = xQueueCreate( 1, sizeof( float ) ); /* Queue do LED */
 
  /* Criando o semaforo */
  xSerial_semaphore = xSemaphoreCreateMutex();
 
  if (xSerial_semaphore == NULL)
  {
  Serial.println("Erro ao criar o semaforo");
  while(1); /* O semáforo é essencial para o funcionamento. Se não houver semáforo, não fazer nada. */
  }
  
  /* Criando as tarefas(tasks) */
  xTaskCreate(
    task_sensor                   /* Funcao que define o que a tarefa vai fazer */
    ,  (const portCHAR *)"tarefa_sensor"   /* Nome da tarefa (apenas para fins descritivos, caso seja necessário degubag) */
    ,  128                          /* Stack Size (tamanho da pilha, em words) reservada para essa tarefa */
    ,  NULL                       /* Parametros (opcional) */
    ,  3                            /* Prioridade da tarefa (quanto maior o número, maior a prioridade) */
    ,  NULL );                      /* Task Handle(opicional) */
 
  xTaskCreate(
    task_lcd
    ,  (const portCHAR *) "tarefa_LCD"
    ,  156  
    ,  NULL
    ,  2
    ,  NULL );
 
 
  xTaskCreate(
    task_led
    ,  (const portCHAR *)"tarefa_LED"
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL );
 
  /* A partir deste momento, o scheduler de tarefas entra em ação e as tarefas executam */
}
 
void loop()
{
  /* Como tudo será executado nas tarefas, a função loop() pode permanecer vazia. */
}
 



/* ====================== Tarefas (Tasks) ===================*/

 
void task_sensor( void *pvParameters )
{
    (void) pvParameters;
    int adc_read=0;
    UBaseType_t uxHighWaterMark;
    float voltage = 0.0;
    
    while(1)
    {  
        adc_read = analogRead(0); /* Faz a leitura do pino A0, aonde está ligado o potenciômetro */
        voltage = ((float)adc_read/ADC_MAX)*MAX_VOLTAGE_ADC; /* calcula a voltagem */
        
        /* Envia a voltagem calculada para as tarefas, por meio das filas */
        xQueueOverwrite(xQueue_LCD, (void *)&voltage);
        xQueueOverwrite(xQueue_LED, (void *)&voltage);
        
        /* Aguarda 1 segundo */
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );/* Tomar controle do Semáforo */

        /* Mostra na serial o high water mark (o máximo de memória usado pela task até o momento) */
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("task_sensor high water mark (words): ");
        Serial.println(uxHighWaterMark);
        Serial.println("---");
        
        xSemaphoreGive(xSerial_semaphore); /* Liberar o semáforo */
    }
}
 
void task_lcd( void *pvParameters )
{
    (void) pvParameters;
    float voltage_rcv = 0.0;
    UBaseType_t uxHighWaterMark;
 
    while(1)
    {        
        /* Esperar a queue receber alguma informação */
        xQueueReceive(xQueue_LCD, (void *)&voltage_rcv, portMAX_DELAY);
        
        /* Mostrar a informação recebida pela queue no display LCD */
        lcd.setCursor(0,0); /* Posicionar o cursor na primeira coluna da primeira linha */
        lcd.print("Tensao: "); /* Exibir o texto "Tensao:" */
        lcd.setCursor(0,1); /* Posicionar o cursor na primeira coluna da segunda linha */
        lcd.print(LCD_16X2_CLEAN_LINE); /* Limpar a linha de qualquer coisa que estiver escrita */
        lcd.setCursor(0,1); /* Posicionar o cursor na primeira coluna da segunda linha */
        lcd.print(voltage_rcv); /* Exibir o valor recebido */
        lcd.setCursor(6,1); /* Posicionar o cursor na décima quinta coluna da segunda linha */
        lcd.print("V"); /* Exibir o texto "V" */
 
        xSemaphoreTake(xSerial_semaphore, portMAX_DELAY ); /* Tomar controle do Semáforo */

        /* Mostra na serial o high water mark (o máximo de memória usado pela task até o momento) */
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("task_lcd high water mark (words): ");
        Serial.println(uxHighWaterMark);
        Serial.println("---");
        
        xSemaphoreGive(xSerial_semaphore); /* Liberar o semáforo */
    }  
}
 
void task_led( void *pvParameters )
{
    (void) pvParameters;
    float voltage_rcv = 0.0;
    UBaseType_t uxHighWaterMark;
 
    while(1)
    {    
        /* Esperar a queue receber alguma informação */
        xQueueReceive(xQueue_LED, (void *)&voltage_rcv, portMAX_DELAY);  
 
        /* Se a informação recebida for maior que o LED_THRESHOLD(limiar mínimo para acendimento do LED): */
        if (voltage_rcv > LED_THRESHOLD)
            digitalWrite(LED_PIN, HIGH);  /* Acender o LED */
        else /* Caso contrário: */
            digitalWrite(LED_PIN, LOW);  /* Apagar o LED */

 
        xSemaphoreTake(xSerial_semaphore, portMAX_DELAY ); /* Tomar controle do Semáforo */

        /* Mostra na serial o high water mark (o máximo de memória usado pela task até o momento) */
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("task_led high water mark (words): ");
        Serial.println(uxHighWaterMark);
        Serial.println("---");
        
        xSemaphoreGive(xSerial_semaphore); /* Liberar o semáforo */
    }  
}
