CFLAGS=-g
all:testEzArduinoSerial 

testEzArduinoSerial : testEzArduinoSerial.o ezArduinoSerial.o

ezArduinoSerial.o: ezArduinoSerial.c ezArduinoSerial.h 

testEzArduinoSerial.o : testEzArduinoSerial.c ezArduinoSerial.h

clean:
	/bin/rm -rf *.o *.a testEzArduinoSerial