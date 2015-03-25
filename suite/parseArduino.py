from subprocess import Popen
import fileinput
import subprocess


def direction(tabPosition):
    directionX=0
    directionY=0
    if(tabPosition[0]> 10 ):
        directionX=1
    elif (tabPosition[0]< -10):
        directionX=-1
    if(tabPosition[1]>10):
        directionY=1
    elif(tabPosition[1]< -10):
        directionY=-1
    tabDirection=[-directionY,-directionX]
    if(tabDirection == [0,0]):
        return "idle"
    elif (tabDirection == [0,1]):
        return "right"
    elif (tabDirection == [0,-1]):
        return "left"
    elif (tabDirection == [1,0]):
        return "forward"
    elif (tabDirection == [1,1]):
        return "forwardRight"
    elif (tabDirection == [1,-1]):
        return "forwardLeft"
    elif (tabDirection == [-1,0]):
        return "backward"
    elif (tabDirection == [-1,1]):
        return "backwardRight"
    elif (tabDirection == [-1,-1]):
        return "backwardLeft"
    
def parseArduino(msg):
    msgsplited=msg.split('#',1)
    if(len(msgsplited)==2):
        return [int(msgsplited[0]),int(msgsplited[1])]
    return None


while(True):
    #call('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial')
    p=Popen('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial',stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    while(True):
          value=p.stdout.readline()
          print value
          parsed=parseArduino(value)
          if parsed != None:
             print direction(parsed)
