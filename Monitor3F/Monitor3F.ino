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
int16_t signal1Samples[ADC_SAMPLES_COUNT];
int16_t signal1Pos = 0;

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
  signal1Pos = 0;
  timerWrite(adcTimer, 0);
  timerAlarmEnable(adcTimer);
}

//==================================================================================================
// Interrupções
//==================================================================================================
// Interrupção de timer: 1s
void IRAM_ATTR onTimer() 
{
  // O mutex é usado para segurança, para que a interrupção não se sobreescreva
  portENTER_CRITICAL_ISR(&timerMux);

  signal1Samples[signal1Pos++] = local_adc1_read(ADC1_CHANNEL_0);
  if (signal1Pos >= ADC_SAMPLES_COUNT) 
  {
    timerAlarmDisable(adcTimer);
    signal1Pos = 0;

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
  for(;;) 
  {
    // Aguarda um pacote de amostras
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);  
    sendCmdBuff(CMD_GET_SAMPLES, (char *)(&signal1Samples), sizeof(signal1Samples));
  }
}

//==================================================================================================
// Setup do sistema para operação
//==================================================================================================
void setup() 
{
    initSerialManager();
    
    // Inicialização dos pinos dos sensores
    sendMessageWithNewLine("Inicializando sistema...");

    // Criação da Task para gerenciar a amostragem de sinais
    xTaskCreate(signalSampling, "Sampler Task", 8192, NULL, 1, &signalSamplingTask);
    
    // Definição de timer para rotinas de tempo
    adcTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(adcTimer, &onTimer, true);
    timerAlarmWrite(adcTimer, 1000000, true);
    
    sendMessageWithNewLine("Inicializacao completa.");
}

//==================================================================================================
// Loop principal do programa
//==================================================================================================
void loop() 
{
  delay(10000);
}
