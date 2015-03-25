#include "ezArduinoSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <errno.h>
#define DEFAULT_TIME_OUT 200
#define TMP_ARDUINO_BUFFER 100
struct ezArduinoSerial{
  int fd;
};

/*
TODO : All of this function should be in another process because there is usleep call which will block the main process .
TODO : No split char 
 */
int readFromArduinoV2(int fd,char *buffer,int bufferSize,char splitChar)
{
  int timeout=DEFAULT_TIME_OUT;
  size_t nbOfCharRead=0;
  size_t totalRead=0;
  char tmpChar=0;
  size_t try=0;
  while(timeout>0 && tmpChar != splitChar && totalRead < bufferSize){
    ++try;
    /*
      fd O_NONBLOCK , only check if 1 Byte is available and read it . 
      return -1 otherwise and errno is set on EAGAIN/EWOULDBLOCK
      TODO : return -1 ?
     */
    nbOfCharRead=read(fd,&tmpChar,sizeof(char));

    /* Nothing to read */
    if(errno == EWOULDBLOCK || errno == EAGAIN )
      {
	/*XXX
	  Currently block the main process ( should be in another thread/process)
	  (thread could be better ).
	 */
	fprintf(stderr,"Try number %d ,nothing to read \n",try);
	usleep(1000);
	--timeout;
	
	
      }
    else if(nbOfCharRead!=-1)
      {
	/*XXX
	  Shouldn't be between [0,1[ .Reference : man 2 read .
	  May be an useless check .
	 */
	if(nbOfCharRead>=1){
	  buffer[totalRead++]=tmpChar;
	  //fprintf(stderr,"%c",tmpChar);
	  
	}
	

      }
    else{
      perror(strerror(errno));
      return -1;
    }
    
  }
  if(tmpChar==splitChar)
    return totalRead;
  /*
    if splitChar wasn't found in  t = usleep(DEFAULT_WAIT_READ) * DEFAULT_TIME_OUT , return -1 . Undefined content of buffer .
   */
  return -1;


}
int readFromArduino(int fd, char *buffer,int size,char splitChar)
{
  int timeout=DEFAULT_TIME_OUT;
  int n=0;
  char tmpChar[1];
  int nbCaracterRead=0;
  bool valid=false;
  while(timeout>0 &&  n>=0 && nbCaracterRead < size && tmpChar !=splitChar ){

    n=read(fd,tmpChar,sizeof(char));
    if(tmpChar==splitChar)
      valid=true;
    if(n>0){
      nbCaracterRead+=n;
      buffer[nbCaracterRead-1]=tmpChar[0];
      
    }
    else if (n==0){
      usleep(1000);
      timeout--;
    }
    else{
      perror(strerror(errno));
      return -1;
    }
    
  }
  if(valid)
    return nbCaracterRead;
  return 0;
}
int readFromEzArduino(EzArduinoSerial this,char *buffer,int size,char splitChar)
{
  return readFromArduinoV2(this->fd,buffer,size,splitChar);
}


  
void writeToEzArduino(EzArduinoSerial this,char *msg)
{
  write(this->fd,msg,strlen(msg)*sizeof(char));
  

}
static void addToBuffer(char *b1,char *b2,int size)
{
  int i;
  for(i=0;i<size;i++)
    b1[i]=b2[i];

}
void delinkArduino(int childPid)
{
  /**
     xxx
  */

  kill(childPid,9);
  waitpid(childPid,NULL);
}
/**
   Create an interface between an arduino and the 
   'main' process ( process calling this function ).
   At the end :
   -input will contain the file descriptor (O_WRONLY|O_TRUNC) associated with 
   arduino .
   -output will contain the file descriptor (O_RDONLY|O_APPEND) associated with 
   arduino .
   -return the pid of the interface.(dont forget to kill him before leaving !!)
   
   Current limitation :
   -output can BLOCK because it's a pipe . Therefore if the arduino is not sending information , read will BLOCK indefinitly . You can change this by using fcntl(output,O_NONBLOCK) .
   -MAX_ARDUINO_BUFFER should be used to read from input and to write to output.
   However you can redefine MAX_ARDUINO_BUFFER size before including ezArduino.h
   
*/
static int initFileDescriptor(char *path,int bitrate)
{
  struct termios t;
  int fd=open(path,O_RDWR);
  tcgetattr(fd,&t);
  cfsetispeed(&t,bitrate);
  cfsetospeed(&t,bitrate);

  
  t.c_cflag &= ~PARENB;
  t.c_cflag &= ~CSTOPB;
  t.c_cflag &= ~CSIZE;
  t.c_cflag |= CS8;
  t.c_cflag &= ~CRTSCTS;
  
  t.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  t.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  t.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  t.c_cc[VMIN]  = 4;
  t.c_cc[VTIME] = 0;
    //t.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &t);
   /**
     Let the arduino time to reset .
  */
  sleep(2);
  return fd;
}
static int readProcess(int *writeToInterface,int *readFromInterface,int fd)
{/*
  pid_t pidreadProcess;
  if((pidreadProcess=fork())==0){
    close(writeToInterface[1]);
    close(writeToInterface[0]);
    close(readFromInterface[0]);


    int nbReception=0;
    int n;
    char buffer[TMP_ARDUINO_BUFFER];
    
    char bigbufferReception[MAX_ARDUINO_BUFFER];
    char bufferReception[TMP_ARDUINO_BUFFER];
    int bigNbReception=0;
    if(DEBUG_EZ_ARDUINO==1){
      char *bufferDEBUG="TEST MESSAGE : \nA B C D E F G J 1 2 3\n";
      write(readFromInterface[1],bufferDEBUG,sizeof(char)*strlen(bufferDEBUG));

	    }
    while((nbReception=readFromArduino(fd,bufferReception,sizeof(char)*TMP_ARDUINO_BUFFER))>=0){
      if(nbReception>0)
	write(readFromInterface[1],bufferReception,sizeof(char)*nbReception);
    }
    perror(strerror(errno));
    exit(1);
  }
  return pidreadProcess;*/
  abort();
  }
static int writeProcess(int *writeToInterface,int *readFromInterface,int fd)
{
  pid_t pidwriteProcess;
  if((pidwriteProcess=fork())==0){
   
    close(writeToInterface[1]);
    close(readFromInterface[0]);
    close(readFromInterface[1]);
  
    exit(1);
  }
  return pidwriteProcess;

}
int linkWithArduino2(char *path,int bitrate,int *input,int *output)
{
  int fd=initFileDescriptor(path,bitrate);

  /**
     Pipe creation 
     used to abstract arduino as a file.
  */
  int writeToInterface[2];
  int readFromInterface[2];
  pipe(writeToInterface);
  pipe(readFromInterface);
  int pidRead=readProcess(writeToInterface,readFromInterface,fd);
  close(writeToInterface[1]);
   close(writeToInterface[0]);
  close(readFromInterface[1]);
  *input=writeToInterface[1];
  *output=readFromInterface[0];
  return pidRead;

}
/*int linkWithArduino(char *path,int bitrate,int *input,int *output)
{

 
  
  int fd=initFileDescriptor(path,bitrate);

  //**
     Pipe creation 
     used to abstract arduino as a file.
       //
  int writeToInterface[2];
  int readFromInterface[2];
  // /**
     Pipe O_NONBLOCK ? ? ?
       // 
  pipe(writeToInterface);
  pipe(readFromInterface);
  
  int pidfils;
  if((pidfils=fork())==0)
    {
      
      // /**
	 'main' process's pipes closed 
      /// 
      close(writeToInterface[1]);
      close(readFromInterface[0]);
     
      int n;
      char buffer[100];
      char bigbufferReception[MAX_ARDUINO_BUFFER];
      char bufferReception[100];
      int nbReception=0;
      int bigNbReception=0;
     // /*XXX
	For now there is a bug because you must write to this process before having something to read .
      //
      while((n=read(writeToInterface[0],buffer,sizeof(char)*100))>0)
	{
	  write(fd,buffer,sizeof(char)*n);
	  while((nbReception=readFromArduino(fd,bufferReception,sizeof(char)*100))>0){
	    addToBuffer(bigbufferReception+bigNbReception,bufferReception,nbReception);
	    bigNbReception+=nbReception;

	  }
	  write(readFromInterface[1],bigbufferReception,sizeof(char)*bigNbReception);
	  if(DEBUG_EZ_ARDUINO==1){
	    write(2,bigbufferReception,sizeof(char)*bigNbReception);

	  }
	  bigNbReception=0;

	}
      perror(strerror(errno));
      exit(1);
	
      //Pipe closed implicitly there

    }
//  /*
    'interface' process's pipes closed
  //
  close(writeToInterface[0]);
  close(readFromInterface[1]);
  *input=writeToInterface[1];
  *output=readFromInterface[0];
  return pidfils;

  }*/
EzArduinoSerial createEzArduinoSerial(char *path,int bitrate)
{
  struct termios t;
  EzArduinoSerial this=malloc(sizeof(struct ezArduinoSerial));
  this->fd=initFileDescriptor(path,bitrate);
  return this;

}

