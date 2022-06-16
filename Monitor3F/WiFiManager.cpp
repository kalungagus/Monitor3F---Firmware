//==================================================================================================
// Includes
//==================================================================================================
#include <WiFi.h>
#include <EEPROM.h>
#include <NetBIOS.h>
#include <AsyncUDP.h>
#include "SystemDefinitions.h"
#include "SerialManager.h"
#include "System.h"

//==================================================================================================
// Macros
//==================================================================================================
#define DEBUG()           digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN))

//==================================================================================================
// Funções do módulo, declaradas aqui para melhor organização do arquivo
//==================================================================================================
void initWiFiManager(void);
void connectToWiFi(void);
void loadWiFiSSID(char *);
void saveWiFiSSID(char *);
void loadWiFiPassword(char *);
void saveWiFiPassword(char *);
static void saveDataEEPROM(char *, int, int);
static void getDataEEPROM(char *, int, int);
void taskCheckWiFiStatus(void *);
bool isWiFiConnected(void);
void resetWiFiConnection(void);
void setupWiFi(void);
void transmissionScheduler(void *);
void listNetworks(void);
void printEncryptionType(int thisType);
void udpProcessing(void *pvParameters);

//==================================================================================================
// Variáveis do módulo
//==================================================================================================
SemaphoreHandle_t internalEEPROMSem;

xTaskHandle taskOnline;
xTaskHandle taskOffline;
xTaskHandle taskUdp;

extern QueueHandle_t messageQueue;

int connectionIdleCounter = 0;
int disconnectedCounter = 0;
bool connectedToClient=false;

AsyncUDP udp;
serialBuffer wifiReceivedData;

//==================================================================================================
// Funções
//==================================================================================================
void initWiFiManager(void)
{
  sendMessageWithNewLine("Inicializando gerenciador de WiFi.", DIRECT_TO_SERIAL);
  
  internalEEPROMSem = xSemaphoreCreateMutex();
  memset(&wifiReceivedData, 0, sizeof(wifiReceivedData));

  EEPROM.begin(EEPROM_SIZE);

  xTaskCreate(taskCheckWiFiStatus, "CheckWiFiStatus", 2000, NULL, 1, &taskOffline);
  xTaskCreate(transmissionScheduler, "TransmissionScheduler", 8192, NULL, 1, &taskOnline);
  vTaskSuspend(taskOnline);
  xTaskCreate(udpProcessing, "udpProcessing", 2000, NULL, 1, &taskUdp);
  vTaskSuspend(taskUdp);
  setupWiFi();
}

void loadWiFiSSID(char *buffer)
{
  getDataEEPROM(buffer, SSID_ADDRESS, SSID_LENGHT);
}

void saveWiFiSSID(char *buffer)
{
  saveDataEEPROM(buffer, SSID_ADDRESS, SSID_LENGHT);
}

void loadWiFiPassword(char *buffer)
{
  getDataEEPROM(buffer, PASSWORD_ADDRESS, PASSWORD_LENGTH);
}

void saveWiFiPassword(char *buffer)
{
  saveDataEEPROM(buffer, PASSWORD_ADDRESS, PASSWORD_LENGTH);
}

static void saveDataEEPROM(char *data, int address, int length)
{
  int i;
  
  xSemaphoreTake(internalEEPROMSem, portMAX_DELAY);
  for(i=0; i<length; i++)
  {
    EEPROM.put(address+i, data[i]);
  }
  EEPROM.commit();
  xSemaphoreGive(internalEEPROMSem);
}

static void getDataEEPROM(char *data, int address, int length)
{
  int i;

  xSemaphoreTake(internalEEPROMSem, portMAX_DELAY);
  for(i=0; i<length; i++)
  {
    data[i] = EEPROM.read(address+i);
  }
  xSemaphoreGive(internalEEPROMSem);
}

bool isWiFiConnected(void)
{
  if(WiFi.status() == WL_CONNECTED)
    return(true);
  else
    return(false);
}

bool isClientConnected(void)
{
  return(connectedToClient);
}

void resetWiFiConnection(void)
{
  WiFi.disconnect();
  vTaskSuspend(taskOnline);
  vTaskResume(taskOffline);
}

void taskCheckWiFiStatus(void *pvParameters)
{
  int updatedWiFiStatus;

  for(;;)
  {
    vTaskDelay( 5000 / portTICK_PERIOD_MS );
    switch(WiFi.status())
    {
      case WL_CONNECTED:
        sendMessageWithNewLine("WiFi conectado.", DIRECT_TO_SERIAL);
        sendMessage("Endereco IP: ", DIRECT_TO_SERIAL);
        sendMessageWithNewLine(WiFi.localIP().toString(), DIRECT_TO_SERIAL);
        sendMessage("Hostname: ", DIRECT_TO_SERIAL);
        sendMessageWithNewLine(WiFi.getHostname(), DIRECT_TO_SERIAL);
        vTaskResume(taskOnline);
        vTaskResume(taskUdp);
        vTaskSuspend(taskOffline);
        break;
      case WL_NO_SHIELD:
        sendMessageWithNewLine("WL_NO_SHIELD retornado.", DIRECT_TO_SERIAL);
        connectToWiFi();
        break;
      case WL_IDLE_STATUS:
        sendMessageWithNewLine("WL_IDLE_STATUS retornado.", DIRECT_TO_SERIAL);
        connectionIdleCounter++;
        if(connectionIdleCounter >= MAX_CONNECTION_IDLE_STATUS)
        {
          connectionIdleCounter = 0;
          connectToWiFi();
        }
        break;
      case WL_NO_SSID_AVAIL:
        sendMessageWithNewLine("WL_NO_SSID_AVAIL retornado.", DIRECT_TO_SERIAL);
        connectToWiFi();
        break;
      case WL_SCAN_COMPLETED:
        sendMessageWithNewLine("WL_SCAN_COMPLETED retornado.", DIRECT_TO_SERIAL);
        break;
      case WL_CONNECT_FAILED:
        sendMessageWithNewLine("WL_CONNECT_FAILED retornado.", DIRECT_TO_SERIAL);
        break;
      case WL_CONNECTION_LOST:
        sendMessageWithNewLine("WL_CONNECTION_LOST retornado.", DIRECT_TO_SERIAL);
        connectToWiFi();
        break;
      case WL_DISCONNECTED:
        disconnectedCounter++;
        if(disconnectedCounter >= MAX_DISCONNECTED_MESSAGES)
        {
          sendMessageWithNewLine("Sistema nao consegue se conectar. Reiniciando para teste.", DIRECT_TO_SERIAL);
          delay(1000);
          ESP.restart();
        }
        else
        {
          setupWiFi();
          connectToWiFi();          
        }
        break;
    }
  }
}

// Detalhes em https://github.com/esp8266/Arduino/issues/4352
void setupWiFi(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.setAutoReconnect(false);
  if( WiFi.getAutoConnect() ) WiFi.setAutoConnect( false );
}

void connectToWiFi(void)
{
  char ssid[SSID_LENGHT] = DEFAULT_SSID;
  char password[PASSWORD_LENGTH] = DEFAULT_PASSWORD;
  String hostname = "ifgmonitor";

  loadWiFiSSID(ssid);
  loadWiFiPassword(password);
  sendMessageWithNewLine("Iniciando conexao com Wifi", DIRECT_TO_SERIAL);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  delay(500);
  NBNS.begin(hostname.c_str());
}

void listNetworks(void) 
{
  char s[16];
  sendMessageWithNewLine("** Scan Networks **", DIRECT_TO_SERIAL);
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) 
  {
    sendMessageWithNewLine("Nao encontrou conexao de rede.", DIRECT_TO_SERIAL);
  }
  else
  {
    sendMessage("Redes disponiveis:", DIRECT_TO_SERIAL);
    snprintf(s, 16, "%d", numSsid);
    sendMessageWithNewLine(s, DIRECT_TO_SERIAL);
  
    for (int thisNet = 0; thisNet < numSsid; thisNet++) 
    {
      sendMessage(String(thisNet), DIRECT_TO_SERIAL);
      sendMessage(") ", DIRECT_TO_SERIAL);
      sendMessage(WiFi.SSID(thisNet), DIRECT_TO_SERIAL);
      sendMessage("\tSignal: ", DIRECT_TO_SERIAL);
      snprintf(s, 16, "%d", WiFi.RSSI(thisNet));
      sendMessage(s, DIRECT_TO_SERIAL);
      sendMessage(" dBm", DIRECT_TO_SERIAL);
      sendMessage("\tEncryption: ", DIRECT_TO_SERIAL);
      snprintf(s, 16, "%d", WiFi.encryptionType(thisNet));
      sendMessageWithNewLine(s, DIRECT_TO_SERIAL);
      vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
  }
}

void transmissionScheduler(void *pvParameters)
{
  WiFiServer server(5000);
  WiFiClient client;
  analogSample sampleToSend;
  const unsigned char sampleHeader[4] = {0xAA, 0x55, sizeof(analogSample)+1, CMD_SAMPLE_SEND};
  char txPacket[MAX_PACKET_SIZE];

  sendMessageWithNewLine("Task servidor iniciada.", DIRECT_TO_SERIAL);
  server.begin();
  for(;;)
  {
    client = server.available();

    if(client)
    {
      sendMessageWithNewLine("Conexao com cliente estabelecida.", DIRECT_TO_SERIAL);
      connectedToClient = true;
      while (client.connected()) 
      {
        if(xQueueReceive(samplesQueue, &sampleToSend, (TickType_t)0) == pdPASS)
        {
          #if (SERIAL_SEND_HEADER == 1)
          client.write(sampleHeader, 4);
          client.flush();
          #endif
          client.write((char *)(&sampleToSend), sizeof(analogSample));
          client.flush();
        }

        if(xQueueReceive(messageQueue, &txPacket, (TickType_t)0) == pdPASS)
        {
          int length = txPacket[2] + 3;
          
          client.write(txPacket, length);
          client.flush();
        }
        
        if(client.available())
        {
          char receivedData = client.read();
          processCharReception(receivedData, &wifiReceivedData);
        }
        
        vTaskDelay( 10 / portTICK_PERIOD_MS );
      }
      sendMessageWithNewLine("Conexao com cliente encerrada.", DIRECT_TO_SERIAL);
      client.stop();
      connectedToClient = false;
    }
    else
    {
      vTaskDelay( 10 / portTICK_PERIOD_MS );
    }
  }
}

void udpProcessing(void *pvParameters)
{
  for(;;)
  {
    if(udp.listen(30000)) 
    {
      udp.onPacket([](AsyncUDPPacket packet) {
        if(packet.isBroadcast())
        {
          if(packet.length() == 3 && packet.data()[0] == 'I' && packet.data()[1] == 'F' && packet.data()[2] == 'G')
          {
            IPAddress myIP = WiFi.localIP();
            unsigned char decodedIP[7] = {'I', 'F', 'G', myIP[0], myIP[1], myIP[2], myIP[3]};
            
            sendMessageWithNewLine("Broadcast recebido.", DIRECT_TO_SERIAL);
            udp.broadcastTo(decodedIP, 7, 30000);
          }
        }
      });
    }
    else
      vTaskDelay( 100 / portTICK_PERIOD_MS );
  }
}

//==================================================================================================
