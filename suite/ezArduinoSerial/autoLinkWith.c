#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ezArduinoSerial.h"
#include <stdbool.h>
static bool usage(int argc,char **argv)
{
  if(argc==2)
    return true;
  return false;
}
static void executeWithArduino(int input,int output,int childpid,char **programme)
{
  pid_t execPid;
  if((execPid=fork())==0)
    {
      dup2(input,1);
      dup2(output,0);
      execvp(programme[0],programme);
      exit(1);
    }
  else{
    close(input);
    close(output);
    waitpid(execPid,NULL);

  }
  delinkArduino(childpid);

}
int main(int argc,char **argv)
{
  if(!usage(argc,argv))
    return EXIT_FAILURE;
  
  int input,output;
  EzArduinoSerial arduino_serial=createEzArduinoSerial("/dev/ttyACM0",9600);
  

  return EXIT_SUCCESS;
}
