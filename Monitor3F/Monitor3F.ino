//==================================================================================================
// Includes
//==================================================================================================
#include "SystemDefinitions.h"
#include "SerialManager.h"
#include "esp_system.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

//==================================================================================================
// Macros
//==================================================================================================
#define DEBUG()           digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN))

//==================================================================================================
// Variáveis globais do sistema
//==================================================================================================
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * adcTimer = NULL;

TaskHandle_t signalSamplingTask;
bool samplingEnabled = false;
bool waitSignalStart = true;
uint16_t signal1Pos = 0;

//==================================================================================================
// Funções do sistema
//==================================================================================================
void setupSampling(void)
{
  signal1Pos = 0;
  timerWrite(adcTimer, 0);
  timerAlarmEnable(adcTimer);
}

uint16_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return((uint16_t)esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//==================================================================================================
// Interrupções
//==================================================================================================
// Interrupção de timer: 1s
void IRAM_ATTR onTimer() 
{  
  portENTER_CRITICAL_ISR(&timerMux);

  // Mesmo que o timer fique ativo, ele não irá amostrar nada sem que o
  // flag de leitura de sinais esteja habilitado.
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
  uint16_t sensorValue;
  analogSample signalSamples;
  
  for(;;) 
  {
    if(samplingEnabled)
    {
      if(waitSignalStart)
      {
        // Espera semi-ciclo negativo
        while(readADC_Cal(adc1_get_raw(SENS_TENSAO1)) > ZERO_VOLT_VALUE);
        sensorValue = readADC_Cal(adc1_get_raw(SENS_TENSAO1));
        // Espera semi-ciclo positivo para começar
        while(sensorValue < ZERO_VOLT_VALUE)
          sensorValue = readADC_Cal(adc1_get_raw(SENS_TENSAO1));
        
        // Envia as primeiras amostras do sinal
        signalSamples.T1 = sensorValue;
        signalSamples.T2 = readADC_Cal(adc1_get_raw(SENS_TENSAO2));
        signalSamples.T3 = readADC_Cal(adc1_get_raw(SENS_TENSAO3));
        signalSamples.I1 = readADC_Cal(adc1_get_raw(SENS_CORRENTE1));
        signalSamples.I2 = readADC_Cal(adc1_get_raw(SENS_CORRENTE2));
        signalSamples.I3 = readADC_Cal(adc1_get_raw(SENS_CORRENTE3));
        sendSample(&signalSamples);
  
        // Inicia a amostragem do sinal
        waitSignalStart = false;
        setupSampling();
      }
      else
      {
        // Aguarda um pacote de amostras
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        digitalWrite(DEBUG_PIN, HIGH);
        signalSamples.T1 = readADC_Cal(adc1_get_raw(SENS_TENSAO1));
        signalSamples.T2 = readADC_Cal(adc1_get_raw(SENS_TENSAO2));
        signalSamples.T3 = readADC_Cal(adc1_get_raw(SENS_TENSAO3));
        signalSamples.I1 = readADC_Cal(adc1_get_raw(SENS_CORRENTE1));
        signalSamples.I2 = readADC_Cal(adc1_get_raw(SENS_CORRENTE2));
        signalSamples.I3 = readADC_Cal(adc1_get_raw(SENS_CORRENTE3));
        sendSample(&signalSamples);
        digitalWrite(DEBUG_PIN, LOW);
            
        signal1Pos++;
        if(signal1Pos > 3000)
        {
          signal1Pos = 0;
          samplingEnabled = false;
        }
      }
    }
    else
    {
      // Se não estiver amostrando sinal, desliga o timer e suspende a task.
      sendCmdString(CMD_MESSAGE, "Parando amostragem\r\n");
      timerAlarmDisable(adcTimer);
      vTaskSuspend(NULL);
    }
  }
}

//==================================================================================================
// Processamento de comandos enviados pela serial
//==================================================================================================
void processReception(char *packet, unsigned int packetSize)
{
  int sensorPin = A0;
  uint16_t sensorValue[2] = {0 , 0};
  
  switch(packet[0])
  {
    case CMD_RESET:
      sendCmdString(CMD_MESSAGE, "Reiniciando\r\n");
      delay(1000);
      ESP.restart();
      break;
    case CMD_SET_SAMPLING_STATE:
      if(samplingEnabled == false && packet[1] == 1)
      {
        sendCmdString(CMD_MESSAGE, "Amostrando\r\n");
        waitSignalStart = true;
        samplingEnabled = true;
        vTaskResume(signalSamplingTask);
      }
      else if(samplingEnabled == true && packet[1] == 0)
      {
        if(clearSamples())
        {
          waitSignalStart = false;
          samplingEnabled = false;
        }
      }
      break;
    case CMD_GET_ADC_READING:
      switch(packet[1])
      {
        case 0:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 1.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_TENSAO1);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 1:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 2.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_TENSAO2);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 2:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 3.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_TENSAO3);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 3:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 1.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE1);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 4:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 2.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE2);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 5:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 3.\r\n");
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE3);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
      }      
      sendCmdBuff(CMD_GET_ADC_READING, (char *)sensorValue, sizeof(sensorValue));
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

    pinMode(DEBUG_PIN, OUTPUT);
    
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

    // Criação da Task para gerenciar a amostragem de sinais
    xTaskCreate(signalSampling, "Sampler Task", 8192, NULL, tskIDLE_PRIORITY, &signalSamplingTask);
    vTaskSuspend(signalSamplingTask);
    
    // Definição de timer para rotinas de tempo
    adcTimer = timerBegin(0, 80, true);                  // Timer trabalhando a 80MHz / 80 == 1MHZ
    timerAttachInterrupt(adcTimer, &onTimer, true);      // Liga a interrupção à função definida
    timerAlarmWrite(adcTimer, 1000, true);               // Amostragem em aproximadamente 1000Hz
    //timerAlarmEnable(adcTimer);
    
    sendMessageWithNewLine("Inicializacao completa.");
}

//==================================================================================================
// Loop principal do programa
//==================================================================================================
void loop() 
{
  delay(10000);
}
