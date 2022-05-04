//==================================================================================================
// Includes
//==================================================================================================
#include "SystemDefinitions.h"
#include "SerialManager.h"
#include "esp_system.h"
#include "driver/adc.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

//==================================================================================================
// Variáveis globais do sistema
//==================================================================================================
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
hw_timer_t * adcTimer = NULL;

TaskHandle_t signalSamplingTask;
uint16_t signalSamples[6];
bool samplingEnabled = false;
uint16_t signal1Pos = 0;

//==================================================================================================
// Funções do sistema
//==================================================================================================
int IRAM_ATTR local_adc1_read(int channel) 
{
    uint16_t adc_value;

    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

void setupSampling(void)
{
  //signal1Pos = 0;
  timerWrite(adcTimer, 0);
  timerAlarmEnable(adcTimer);
}

//==================================================================================================
// Interrupções
//==================================================================================================
// Interrupção de timer: 1s
void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  
  if (samplingEnabled)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(signalSamplingTask, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) 
      portYIELD_FROM_ISR();
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

//==================================================================================================
// Tasks
//==================================================================================================
void signalSampling(void *param) 
{
  uint16_t sensorValue = 0;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;

  xLastWakeTime = xTaskGetTickCount();
  for(;;) 
  {
    // Aguarda um pacote de amostras
    //vTaskDelayUntil( &xLastWakeTime, xFrequency );
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    //if (samplingEnabled)
    {
      signalSamples[0] = adc1_get_raw(SENS_TENSAO1);
      signalSamples[1] = adc1_get_raw(SENS_TENSAO2);
      signalSamples[2] = adc1_get_raw(SENS_TENSAO3);
      signalSamples[3] = adc1_get_raw(SENS_CORRENTE1);
      signalSamples[4] = adc1_get_raw(SENS_CORRENTE2);
      signalSamples[5] = adc1_get_raw(SENS_CORRENTE3);
      sendCmdBuff(CMD_SAMPLE_SEND, (char *)(&signalSamples[0]), sizeof(signalSamples));

      signal1Pos++;
      if(signal1Pos > 500)
      {
        signal1Pos = 0;
        samplingEnabled = false;
      }
    }
  }
}

//==================================================================================================
// Processamento de comandos enviados pela serial
//==================================================================================================
void processReception(char *packet, unsigned int packetSize)
{
  int sensorPin = A0;
  uint16_t sensorValue = 0;
  
  switch(packet[0])
  {
    case CMD_RESET:
      sendCmdString(CMD_MESSAGE, "Reiniciando\r\n");
      delay(1000);
      ESP.restart();
      break;
    case CMD_SET_SAMPLING_STATE:
      samplingEnabled = packet[1];
      if(packet[1] == 1)
        sendCmdString(CMD_MESSAGE, "Amostrando\r\n");
      else
        sendCmdString(CMD_MESSAGE, "Parando amostragem\r\n");
      break;
    case CMD_GET_ADC_READING:
      switch(packet[1])
      {
        case 0:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 1.\r\n");
          sensorValue = adc1_get_raw(SENS_TENSAO1);
          break;
        case 1:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 2.\r\n");
          sensorValue = adc1_get_raw(SENS_TENSAO2);
          break;
        case 2:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 3.\r\n");
          sensorValue = adc1_get_raw(SENS_TENSAO3);
          break;
        case 3:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 1.\r\n");
          sensorValue = adc1_get_raw(SENS_CORRENTE1);
          break;
        case 4:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 2.\r\n");
          sensorValue = adc1_get_raw(SENS_CORRENTE2);
          break;
        case 5:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 3.\r\n");
          sensorValue = adc1_get_raw(SENS_CORRENTE3);
          break;
      }      
      sendCmdBuff(CMD_GET_ADC_READING, (char *)(&sensorValue), sizeof(sensorValue));
      break;
    case CMD_SEND_ECHO:
      sendCmdBuff(CMD_MESSAGE, &packet[1], packetSize-1);
      break;
    default:
      break;
  }
}

//==================================================================================================
// Setup do sistema para operação
//==================================================================================================
void setup() 
{
    initSerialManager();
    delay(1000);
    
    // Inicialização dos pinos dos sensores
    sendMessageWithNewLine("Inicializando sistema...");

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

    // Criação da Task para gerenciar a amostragem de sinais
    xTaskCreate(signalSampling, "Sampler Task", 8192, NULL, 1, &signalSamplingTask);
    
    // Definição de timer para rotinas de tempo
    adcTimer = timerBegin(0, 80, true);                  // Timer trabalhando a 80MHz / 80 == 1MHZ
    timerAttachInterrupt(adcTimer, &onTimer, true);      // Liga a interrupção à função definida
    timerAlarmWrite(adcTimer, 3333, true);               // Amostragem em aproximadamente 300Hz
    timerAlarmEnable(adcTimer);
    
    sendMessageWithNewLine("Inicializacao completa.");
}

//==================================================================================================
// Loop principal do programa
//==================================================================================================
void loop() 
{
  delay(10000);
}
