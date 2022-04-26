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
static void sendCmdString(unsigned char, String);
static void sendCmdBuff(unsigned char, char *, unsigned char);
static void sendAck(unsigned char);
static void taskSerialReceive(void *);
static void loadIntoArray(uint32_t, char *);

extern void setupSampling(void);

//==================================================================================================
// Variáveis do módulo
//==================================================================================================
SemaphoreHandle_t serialSem;

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

static void sendCmdString(unsigned char cmd, String s)
{
  unsigned char header[4] = {0xAA, 0x55, cmd, s.length()};

  xSemaphoreTake(serialSem, portMAX_DELAY);
  #if (SERIAL_SEND_HEADER == 1)
  Serial.write(header, 4);
  #endif
  Serial.write(s.c_str());
  xSemaphoreGive(serialSem);
}

static void taskSerialReceive(void *pvParameters)
{
  char CharSerialRX;
  char messageSize;
  char tmpbuff[20];
  int serialState = 0;
  long int address;
  int size;

  for(;;)
  {
    if(Serial.available())
    {
      CharSerialRX = (char)Serial.read();
      if(serialState == 0)
      {
        if(CharSerialRX == 0xAA) serialState++;
      }
      else if(serialState == 1)
      {
        if(CharSerialRX == 0x55) 
          serialState++;
        else
          serialState = 0;
      }
      else
      {
        int index;
        
        switch(CharSerialRX)
        {
          case CMD_RESET:
            sendCmdString(CMD_MESSAGE, "Reiniciando\r\n");
            delay(1000);
            ESP.restart();
            break;
          case CMD_GET_SAMPLES:
          default:
            break;
        }
        serialState = 0;
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
  unsigned char header[5] = {0xAA, 0x55, cmd, ((char*)(&length))[1], ((char*)(&length))[0]};

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
  unsigned char txBuffer[6] = {0xAA, 0x55, cmd, 0x00, 0x01, 0x06};
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
