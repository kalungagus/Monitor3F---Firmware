//==================================================================================================
//                           MÓDULO DE GERENCIAMENTO DA CONEXÃO WIFI
//==================================================================================================

//==================================================================================================
// Funções
//==================================================================================================
extern void initWiFiManager(void);
extern void connectToWiFi(void);
extern void loadWiFiSSID(char *);
extern void saveWiFiSSID(char *);
extern void loadWiFiPassword(char *);
extern void saveWiFiPassword(char *);
extern bool isWiFiConnected(void);
extern void resetWiFiConnection(void);
extern bool isClientConnected(void);

//==================================================================================================
// Variáveis
//==================================================================================================
extern QueueHandle_t transmissionQueue;

//==================================================================================================
