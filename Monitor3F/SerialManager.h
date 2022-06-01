//==================================================================================================
//                           MÓDULO DE GERENCIAMENTO DA SERIAL
//==================================================================================================

//==================================================================================================
// Funções
//==================================================================================================
extern void sendMessage(String);
extern void sendMessageWithNewLine(String);
extern void sendCmdString(unsigned char, String);
extern void sendAck(unsigned char cmd);
extern bool sendSample(analogSample *theSample);
extern bool clearSamples(void);
extern void initSerialManager(void);
extern void sendCmdBuff(unsigned char cmd, char *buff, unsigned int length);

//==================================================================================================
