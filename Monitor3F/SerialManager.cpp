//==================================================================================================
// Includes
//==================================================================================================
#include "SystemDefinitions.h"

//==================================================================================================
// Funções do módulo, declaradas aqui para melhor organização do arquivo
//==================================================================================================
void initSerialManager(void);
void sendMessage(String);
void sendMessageWithNewLine(String);
void sendCmdBuff(unsigned char, char *, unsigned int);
void sendCmdString(unsigned char, String);
static void sendAck(unsigned char);
static void taskSerialReceive(void *);
static void loadIntoArray(uint32_t, char *);

// Esta função deve ser declarada em um arquivo de projeto para tratar os comandos recebidos por este
// módulo
extern void processReception(char *packet, unsigned int packetSize);

//==================================================================================================
// Variáveis do módulo
//==================================================================================================
SemaphoreHandle_t serialSem;
char rxPacket[MAX_PACKET_SIZE];

//==================================================================================================
// Funções
//==================================================================================================
void initSerialManager(void)
{
  Serial.begin(115200);
  serialSem = xSemaphoreCreateMutex();
  xTaskCreate(taskSerialReceive, "SerialEvent", 2000, NULL, 1, NULL);
}

void sendMessage(String s)
{
  sendCmdString(CMD_MESSAGE, s);
}

void sendMessageWithNewLine(String s)
{
  String msg = s + "\r\n";
  sendCmdString(CMD_MESSAGE, msg);
}

void sendCmdString(unsigned char cmd, String s)
{
  unsigned int packetSize = s.length() + 1;
  unsigned char header[5] = {0xAA, 0x55, ((char*)(&packetSize))[0], ((char*)(&packetSize))[1] , cmd};

  xSemaphoreTake(serialSem, portMAX_DELAY);
  #if (SERIAL_SEND_HEADER == 1)
  Serial.write(header, 5);
  #endif
  Serial.write(s.c_str());
  xSemaphoreGive(serialSem);
}

static void taskSerialReceive(void *pvParameters)
{
  char CharSerialRX;
  unsigned int messageSize=0;
  int serialState = 0;
  int size=0;

  for(;;)
  {
    if(Serial.available())
    {
      CharSerialRX = (char)Serial.read();
      
      switch(serialState)
      {
        case 0:
          if(CharSerialRX == 0xAA) 
            serialState++;
          break;
        case 1:
          if(CharSerialRX == 0x55) 
            serialState++;
          break;
        case 2:
          ((char*)(&messageSize))[0] = CharSerialRX;
          serialState++;
          break;
        case 3:
          ((char*)(&messageSize))[1] = CharSerialRX;
          serialState++;
          messageSize = (messageSize > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : messageSize;
          size=0;
          break;
        default:
          if(size < messageSize)
          {
            rxPacket[size++] = CharSerialRX;
            if(size >= messageSize)
            {
              processReception(rxPacket, size);
              serialState = 0;
            }
          }
          else
            serialState = 0;
          break;
      }
    }
    else
    {
      vTaskDelay( 20 / portTICK_PERIOD_MS );
    }
  }
}

void sendCmdBuff(unsigned char cmd, char *buff, unsigned int length)
{
  unsigned char header[5] = {0xAA, 0x55, 0x00, 0x00, cmd};
  unsigned int packetSize = length + 1;
  
  // Insere o tamanho no buffer
  header[2] = ((char*)(&packetSize))[0];
  header[3] = ((char*)(&packetSize))[1];
  
  xSemaphoreTake(serialSem, portMAX_DELAY);
  #if (SERIAL_SEND_HEADER == 1)
  Serial.write(header, 5);
  #endif
  Serial.write((const unsigned char *)buff, length);
  xSemaphoreGive(serialSem);
}

void sendAck(unsigned char cmd)
{
  #if (SERIAL_SEND_HEADER == 1)
  unsigned char txBuffer[6] = {0xAA, 0x55, 0x01, 0x00, 0x06};
  #else
  unsigned char txBuffer[4] = {'O', 'K', '\r', '\n'};
  #endif

  xSemaphoreTake(serialSem, portMAX_DELAY);
  Serial.write(txBuffer, 4);
  xSemaphoreGive(serialSem);
}

static void loadIntoArray(uint32_t value, char *buffer)
{
  for(int i=0; i<sizeof(uint32_t); i++)
    buffer[i] = ((char *)(&value))[i];
}

//==================================================================================================
