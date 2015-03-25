#ifndef EZ_ARDUINO_SERIAL_H
#define EZ_ARDUINO_SERIAL_H
#ifndef MAX_ARDUINO_BUFFER
#define MAX_ARDUINO_BUFFER 1000
#endif
#ifndef DEBUG_EZ_ARDUINO
#define DEBUG_EZ_ARDUINO 1
#endif
typedef struct ezArduinoSerial *EzArduinoSerial;
EzArduinoSerial createEzArduinoSerial(char*,int);
int readFromEzArduino(EzArduinoSerial,char*,int,char);
int readFromArduino2(EzArduinoSerial,char* buffer,int bufferSize,char splitChar);
int readFromArduino(int,char*,int,char);
void writeToEzArduino(EzArduinoSerial,char*);
int linkWithArduino(char*,int,int *input,int *output); 
int linkWithArduino2(char*,int,int *input,int *output);
void delinkArduino(int);
#endif
