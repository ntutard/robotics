#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ezArduinoSerial.h"

int main(int argc,char **argv)
{

  
  EzArduinoSerial arduino_serial=createEzArduinoSerial("/dev/ttyACM0",9600);
  char buffer[100];
  int read=0;

  while(1){
    if((read=readFromEzArduino(arduino_serial,buffer,100,'\n'))>0){
      write(1,buffer,sizeof(char)*read);

    }
  
  }
  return EXIT_SUCCESS;
}
