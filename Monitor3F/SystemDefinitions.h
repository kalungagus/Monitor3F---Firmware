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
#define CMD_SEND_SSID            0x86
#define CMD_GET_SSID             0x87
#define CMD_SEND_PASSWORD        0x88
#define CMD_GET_PASSWORD         0x89

//==================================================================================================
// Configurações
//==================================================================================================
#define SERIAL_SEND_HEADER            1
#define ADC_SAMPLES_COUNT             1000
#define MAX_PACKET_SIZE               50
#define ZERO_VOLT_VALUE               1650
#define MESSAGE_QUEUE_SIZE            10
#define SAMPLE_QUEUE_SIZE             10000
#define MAX_CONNECTION_IDLE_STATUS    5
#define MAX_DISCONNECTED_MESSAGES     10

#define DEBUG_PIN                     26
#define BUTTON_PIN                    15

#define SENS_TENSAO1                  ADC1_CHANNEL_0
#define SENS_TENSAO2                  ADC1_CHANNEL_3
#define SENS_TENSAO3                  ADC1_CHANNEL_6
#define SENS_CORRENTE1                ADC1_CHANNEL_7
#define SENS_CORRENTE2                ADC1_CHANNEL_4
#define SENS_CORRENTE3                ADC1_CHANNEL_5

//--------------------------------------------------------------------------------------------------
// Parâmetros de SSID e Senha de rede
//--------------------------------------------------------------------------------------------------
#define SSID_ADDRESS                 0
#define SSID_LENGHT                  20
#define PASSWORD_ADDRESS             SSID_ADDRESS+SSID_LENGHT
#define PASSWORD_LENGTH              20
#define EEPROM_SIZE                  PASSWORD_ADDRESS+PASSWORD_LENGTH

//--------------------------------------------------------------------------------------------------
// Credenciais de rede
//--------------------------------------------------------------------------------------------------
#define DEFAULT_SSID                 " \0"
#define DEFAULT_PASSWORD             " \0"

//==================================================================================================
// Tipos de dados padrão para todo o projeto
//==================================================================================================
typedef struct __attribute__((__packed__))
{
  uint16_t T1 : 12;
  uint16_t T2 : 12;
  uint16_t T3 : 12;
  uint16_t I1 : 12;
  uint16_t I2 : 12;
  uint16_t I3 : 12;
} analogSample;

typedef struct
{
  unsigned char state;
  unsigned char messageSize;
  unsigned char bytesReaded;
  char packet[MAX_PACKET_SIZE];
} serialBuffer;
