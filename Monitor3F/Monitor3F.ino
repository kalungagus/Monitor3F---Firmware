//==================================================================================================
// Includes
//==================================================================================================
#include "SystemDefinitions.h"
#include "SerialManager.h"
#include "WiFiManager.h"
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
QueueHandle_t samplesQueue;
bool samplingEnabled = false;
uint16_t signalPos = 0;
uint16_t quantSamples = 1000;

//==================================================================================================
// Funções do sistema
//==================================================================================================
void setupSampling(void)
{
  signalPos = 0;
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
void IRAM_ATTR onButton() 
{
  if(samplingEnabled == false)
  {
    BaseType_t xYieldRequired;
    
    quantSamples = 1000;
    samplingEnabled = true;
    setupSampling();
    xYieldRequired = xTaskResumeFromISR(signalSamplingTask);
    portYIELD_FROM_ISR(xYieldRequired);
  }
}

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
      // Aguarda um pacote de amostras
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      //digitalWrite(DEBUG_PIN, HIGH);
      signalSamples.T1 = readADC_Cal(adc1_get_raw(SENS_TENSAO1));
      signalSamples.T2 = readADC_Cal(adc1_get_raw(SENS_TENSAO2));
      signalSamples.T3 = readADC_Cal(adc1_get_raw(SENS_TENSAO3));
      signalSamples.I1 = readADC_Cal(adc1_get_raw(SENS_CORRENTE1));
      signalSamples.I2 = readADC_Cal(adc1_get_raw(SENS_CORRENTE2));
      signalSamples.I3 = readADC_Cal(adc1_get_raw(SENS_CORRENTE3));
      sendSample(&signalSamples);
      //digitalWrite(DEBUG_PIN, LOW);
          
      signalPos++;
      if(signalPos > quantSamples)
      {
        signalPos = 0;
        samplingEnabled = false;
      }
    }
    else
    {
      // Se não estiver amostrando sinal, desliga o timer e suspende a task.
      sendCmdString(CMD_MESSAGE, "Parando amostragem\r\n", DIRECT_TO_SERIAL);
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
  char s[5];
  char tmpbuff[20];
  unsigned int messageSize = packetSize-1;
  int index;
  
  switch(packet[0])
  {
    case CMD_RESET:
      sendCmdString(CMD_MESSAGE, "Reiniciando\r\n", DIRECT_TO_SERIAL);
      delay(1000);
      ESP.restart();
      break;
    case CMD_SET_SAMPLING_STATE:
      if(samplingEnabled == false && packet[1] == 1)
      {
        quantSamples = *((uint16_t *)(&packet[2]));
        snprintf(s, 5, "%d", quantSamples);
        sendMessage("Lendo ", DIRECT_TO_SERIAL);
        sendMessage(s, DIRECT_TO_SERIAL);
        sendMessageWithNewLine(" amostras.", DIRECT_TO_SERIAL);
        samplingEnabled = true;
        setupSampling();
        vTaskResume(signalSamplingTask);
      }
      else if(samplingEnabled == true && packet[1] == 0)
      {
        samplingEnabled = false;
        clearSamples();
      }
      break;
    case CMD_GET_ADC_READING:
      switch(packet[1])
      {
        case 0:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 1.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_TENSAO1);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 1:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 2.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_TENSAO2);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 2:
          sendCmdString(CMD_MESSAGE, "Lendo tensao 3.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_TENSAO3);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 3:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 1.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE1);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 4:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 2.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE2);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
        case 5:
          sendCmdString(CMD_MESSAGE, "Lendo corrente 3.\r\n", DIRECT_TO_SERIAL);
          sensorValue[0] = adc1_get_raw(SENS_CORRENTE3);
          sensorValue[1] = readADC_Cal(sensorValue[0]);
          break;
      }      
      sendCmdBuff(CMD_GET_ADC_READING, (char *)sensorValue, sizeof(sensorValue), PRIORITY_SELECT);
      break;
    case CMD_SEND_ECHO:
      sendCmdBuff(CMD_MESSAGE, &packet[1], messageSize, PRIORITY_SELECT);
      break;
    case CMD_SEND_SSID:
      if(messageSize > SSID_LENGHT) messageSize = SSID_LENGHT;
      for(index=0; index<messageSize; index++)
        tmpbuff[index] = packet[index+1];
      for(; index<SSID_LENGHT; index++)
        tmpbuff[index] = 0;
      tmpbuff[messageSize] = 0;
      saveWiFiSSID(tmpbuff);
      sendCmdString(CMD_MESSAGE, "SSID Alterado.\r\n", DIRECT_TO_SERIAL);
      break;
    case CMD_GET_SSID:
      loadWiFiSSID(tmpbuff);
      sendCmdString(CMD_GET_SSID, tmpbuff, PRIORITY_SELECT);
      break;
    case CMD_SEND_PASSWORD:
      if(messageSize > SSID_LENGHT) messageSize = PASSWORD_LENGTH;
      for(index=0; index<messageSize; index++)
        tmpbuff[index] = packet[index+1];
      for(; index<PASSWORD_LENGTH; index++)
        tmpbuff[index] = 0;
      tmpbuff[messageSize] = 0;
      saveWiFiPassword(tmpbuff);
      sendCmdString(CMD_MESSAGE, "Password Alterado.\r\n", DIRECT_TO_SERIAL);
      break;
    case CMD_GET_PASSWORD:
      loadWiFiPassword(tmpbuff);
      sendCmdString(CMD_GET_PASSWORD, tmpbuff, PRIORITY_SELECT);
      break;
    default:
      break;
  }
}

void processCharReception(unsigned char data, serialBuffer *thisSerial)
{
  switch(thisSerial->state)
  {
    case 0:
      if(data == 0xAA) 
        thisSerial->state++;
      break;
    case 1:
      if(data == 0x55) 
        thisSerial->state++;
      break;
    case 2:
      thisSerial->messageSize = (data > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : data;
      thisSerial->bytesReaded=0;
      thisSerial->state++;
      break;
    default:
      if(thisSerial->bytesReaded < thisSerial->messageSize)
      {
        thisSerial->packet[thisSerial->bytesReaded++] = data;
        if(thisSerial->bytesReaded >= thisSerial->messageSize)
        {
          processReception(thisSerial->packet, thisSerial->messageSize);
          thisSerial->state = 0;
        }
      }
      else
        thisSerial->state = 0;
      break;
  }
}

//==================================================================================================
// Setup do sistema para operação
//==================================================================================================
void setup() 
{
    samplesQueue = xQueueCreate(SAMPLE_QUEUE_SIZE, sizeof(analogSample));
    initSerialManager();
    delay(1000);
    
    // Inicialização dos pinos dos sensores
    sendMessageWithNewLine("Inicializando sistema...", PRIORITY_SELECT);

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

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButton, FALLING);

    initWiFiManager();
    
    sendMessageWithNewLine("Inicializacao completa.", PRIORITY_SELECT);
}

//==================================================================================================
// Loop principal do programa
//==================================================================================================
void loop() 
{
  delay(10000);
}
