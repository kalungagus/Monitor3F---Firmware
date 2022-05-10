//**************************************************************************************************
//                                     Classe de sistema
// Descrição: Aqui está organizado o sistema de controle para os projetos
//**************************************************************************************************

//==================================================================================================
// Includes
//==================================================================================================
#include <Arduino.h>

//==================================================================================================
// Comandos
//==================================================================================================
#define CMD_RESET                0x80
#define CMD_MESSAGE              0x81
#define CMD_SET_SAMPLING_STATE   0x82
#define CMD_SEND_ECHO            0x83
#define CMD_GET_ADC_READING      0x84
#define CMD_SAMPLE_SEND          0x85

//==================================================================================================
// Configurações
//==================================================================================================
#define SERIAL_SEND_HEADER            1
#define ADC_SAMPLES_COUNT             1000
#define MAX_PACKET_SIZE               20
#define ZERO_VOLT_VALUE               1650

#define SENS_TENSAO1                  ADC1_CHANNEL_0
#define SENS_TENSAO2                  ADC1_CHANNEL_3
#define SENS_TENSAO3                  ADC1_CHANNEL_6
#define SENS_CORRENTE1                ADC1_CHANNEL_7
#define SENS_CORRENTE2                ADC1_CHANNEL_4
#define SENS_CORRENTE3                ADC1_CHANNEL_5
