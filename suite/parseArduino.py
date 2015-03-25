from subprocess import Popen
import fileinput
import subprocess
def parseArduino(msg):
    return msg.split('#',1)


while(True):
    #call('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial')
    p=Popen('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial',stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    while(True):
          value=p.stdout.readline()
          print parseArduino(value)
